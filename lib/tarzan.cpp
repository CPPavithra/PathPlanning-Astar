#include <asio.hpp>
#include <cstddef>
#include <stdlib.h>
#include <cobs.h>
#include <liburing.h>
#include <rerun.hpp>
#include <cstdio>

#include "alloc.hpp"
#include "crc.h"
#include "tarzan.hpp"

namespace tarzan {
const char* getError(enum Error err) {
  switch (err) {
  case Error::NoError:
    return "No Error";
  case Error::InvalidHandle:
    return "Invalid Handle Pointer Passed";
  case Error::UringRegisterFailure:
    return "Could not register device with io_uring";
  case Error::UringWaitCqe:
    return "Error in io_uring_wait_cqe";
  case Error::FileOpFailure:
    return "Device IO error";
  case Error::CobsEncodeError:
    return "Could not encode data to COBS";
  case Error::CouldNotOpenFile:
    return "Could not open file";
  case Error::AllocationError:
    return "Error allocating memory";
  case Error::NotImplemented:
    return "Function not implemented";
  }
  return "Unknown Error";
}
struct DiffDriveTwist {
  float linear_x, angular_z;
};
struct all_msg {
  struct DiffDriveTwist auto_cmd;
  uint32_t crc;
};

struct Handle {
  struct io_uring ring;
  asio::io_context io_ctx;
  asio::serial_port uart;
  int fd;
  mem::Allocator* allocator;
  int write_rq, read_rq;
};
constexpr size_t TARZAN_MAX_MSG_LEN = sizeof(struct all_msg) + 2;
constexpr size_t TARZAN_BLOCK_SIZE = 512;
constexpr size_t QUEUE_DEPTH = 2;

auto initHandle(mem::Allocator& allocator, const char* serial_path)
  -> std::variant<Error, struct Handle*>
{
  std::variant<Error, struct Handle*> result;
  struct Handle* handle =
    (struct Handle*)allocator.alloc(sizeof(struct Handle));
  if (not handle) {
    result.emplace<0>(Error::AllocationError);
    return result;
  }
  io_uring_queue_init(QUEUE_DEPTH, &handle->ring, 0);
  // TODO: add msg table, pre-allocating iovecs and msg buffers
  int fd = open(serial_path, O_RDWR);
  if (fd == -1) {
    result.emplace<0>(Error::CouldNotOpenFile);
    return result;
  }
  handle->fd = fd;
  handle->allocator = &allocator;
  if (io_uring_register_files(&handle->ring, &fd, 1)) {
    result.emplace<0>(Error::UringRegisterFailure);
    return result;    
  }
  handle->write_rq = handle->read_rq = 0;

  return result.emplace<1>(handle);
}
auto initHandleAsio(mem::Allocator& allocator, const char* serial_path)
  -> std::variant<Error, struct Handle*>
{
  std::variant<Error, struct Handle*> result;
  struct Handle* handle =
    (struct Handle*)allocator.alloc(sizeof(struct Handle));
  if (not handle) {
    result.emplace<0>(Error::AllocationError);
    return result;
  }
  new (&handle->io_ctx) asio::io_context;
  new (&handle->uart) asio::serial_port(handle->io_ctx, serial_path);
  handle->uart.set_option(asio::serial_port_base::baud_rate(9600));
  result.emplace<1>(handle);
  return result;
}
auto destroyHandle(struct Handle* handle) -> Error {
  if (not handle) return Error::InvalidHandle;
  close(handle->fd);
  io_uring_queue_exit(&handle->ring);
  return Error::NoError;
}
auto submitWrite(Handle* handle, void* msg, size_t msg_len, int) -> Error {
  asio::async_write(handle->uart, asio::buffer(msg, msg_len), [&handle, msg_len](std::error_code, size_t sz) {
                      if (sz != msg_len) {
                        printf("Wrote less than sent\n");
                      }
                    });
    return Error::NoError;
}
auto submitWrite(Handle *handle,
                 void* msg, size_t msg_len) -> Error {
  if (not handle)
    return Error::InvalidHandle;
  if (handle->write_rq){
    auto err = update(handle);
    if (err != Error::NoError) return err;
  }
  struct iovec *send_iovec =
      (struct iovec *)handle->allocator->alloc(sizeof(struct iovec));
  send_iovec->iov_base = msg;
  send_iovec->iov_len = msg_len;

  struct io_uring_sqe *sqe = io_uring_get_sqe(&handle->ring);
  io_uring_prep_writev(sqe, 0, send_iovec, 1, 0);
  io_uring_sqe_set_data(sqe, send_iovec);
  sqe->flags |= IOSQE_FIXED_FILE;
  io_uring_submit(&handle->ring);
  handle->write_rq++;
  return Error::NoError;
}
auto update(struct Handle* handle) -> Error {
  if (not handle) return Error::InvalidHandle;
  if (handle->read_rq == 0 and handle->write_rq == 0) return Error::NoError;
  struct io_uring_cqe *cqe;
  if (io_uring_wait_cqe(&handle->ring, &cqe) < 0) return Error::UringWaitCqe;
  if (cqe->res < 0) return Error::FileOpFailure;
  if (cqe->res != TARZAN_MAX_MSG_LEN) {
    // TODO: Return a fallible error here
    // printf("Wrote less than msg_len\n");
  }
  struct iovec* ret_iovec = (struct iovec*) io_uring_cqe_get_data(cqe);
  handle->write_rq--;
  return Error::NoError;
}
auto submitReadRequest(mem::Allocator &alloc, const char *file_path,
                       struct io_uring *ring) -> Error {
  size_t bytes_remaining = TARZAN_MAX_MSG_LEN;
  size_t offset = 0;
  int current_block = 0;
  int blocks = TARZAN_MAX_MSG_LEN / TARZAN_BLOCK_SIZE;
  if (TARZAN_MAX_MSG_LEN % TARZAN_BLOCK_SIZE) blocks++;

  int fd = open(file_path, O_RDONLY);
  if (fd == -1) return Error::CouldNotOpenFile;

  struct iovec* iovecs =
    (struct iovec*)alloc.alloc(sizeof(struct iovec) * blocks);
  if (not iovecs) return Error::AllocationError;

  while (bytes_remaining) {
    size_t bytes_to_read = bytes_remaining;
    if (bytes_to_read > TARZAN_BLOCK_SIZE) bytes_to_read = TARZAN_BLOCK_SIZE;
    offset += bytes_to_read;
    iovecs[current_block].iov_len = bytes_to_read;
    void *buf;
    if (posix_memalign(&buf, TARZAN_BLOCK_SIZE, TARZAN_BLOCK_SIZE)) {
      return Error::AllocationError;
    }
    iovecs[current_block].iov_base = buf;
    current_block++;
    bytes_remaining -= bytes_to_read;
  }

  struct io_uring_sqe *sqe = io_uring_get_sqe(ring);
  io_uring_prep_readv(sqe, fd, iovecs, blocks, 0);
  const char* uniq = "SOME";
  io_uring_sqe_set_data(sqe, (void*) uniq);
  io_uring_submit(ring);
  return Error::NoError;
}
auto sendMessage(struct Handle* handle, const all_msg& msg) -> Error {
  if (not handle) return Error::InvalidHandle;
  uint8_t buffer[TARZAN_MAX_MSG_LEN];
  if (auto result = cobs_encode(
          reinterpret_cast<void *>(buffer), TARZAN_MAX_MSG_LEN,
          reinterpret_cast<const void *>(&msg), sizeof(struct all_msg));
      result.status != COBS_ENCODE_OK) {
        return CobsEncodeError;
  }
  buffer[TARZAN_MAX_MSG_LEN-1] = 0x00;
  return submitWrite(handle, buffer, TARZAN_MAX_MSG_LEN, 1);
}
auto setTargetVelocity(struct Handle *handle, float linear_x,
                       float angular_z, const void* rec = nullptr) -> Error {
  if (not handle) return Error::InvalidHandle;
  struct DiffDriveTwist twist = {.linear_x = linear_x, .angular_z = angular_z};
  if (rec != nullptr)
    ((rerun::RecordingStream *)rec)
        ->log("log/vel", rerun::BarChart::f32({linear_x, angular_z}));
  struct all_msg msg = {.auto_cmd = twist};
  msg.crc = crc32_ieee(reinterpret_cast<const uint8_t *>(&msg),
                       sizeof(struct all_msg) - sizeof(msg.crc));
  for (int i = 0; i < 5; i++) {
    auto res = sendMessage(handle, msg);
    if (res != Error::NoError) return res;
  }
  return Error::NoError;
}
}

#ifdef TEST_TARZAN_CPP
#include <cstdio>
int main() {
  mem::GeneralPurposeAllocator gpa;
  auto handle = std::get<1>(tarzan::initHandleAsio(gpa, "/dev/ttyACM0"));
  if (not handle) {
    printf("Tarzan initHandle failure\n");
    return 1;
  }
  auto err = tarzan::setTargetVelocity(handle, 1, 0, 0x00);
  if (err) {
    printf("Error setting target velocity: %s\n", tarzan::getError(err));
    return 1;
  }
  err = tarzan::update(handle);
  if (err) {
    printf("Error updating: %s\n", tarzan::getError(err));
    return 1;
  }
  err = tarzan::destroyHandle(handle);
  if (err) {
    printf("Error destroying handle: %s\n", tarzan::getError(err));
  }
  return 0;
}
#endif
