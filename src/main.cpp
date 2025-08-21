#include "motionplanner.h"
#include <rerun.hpp>
#include <iostream>

int main() {
    try {
        auto rec = rerun::RecordingStream("rover_simulation");
        rec.spawn().exit_on_failure();
        MotionPlanner controller(rec);
        controller.setup();
        //Main loop
        bool needs_planning=false;
        while (true) {
            if (needs_planning) {
                controller.runPathPlanning();
                needs_planning=false; 
            } else {
                if (controller.runMapping()) {
                    needs_planning=true;
                }
            }
        }

    } catch (const rs2::error& e) {
        std::cerr << "RealSense error caught in main: " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "Standard exception caught in main: " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Unknown exception caught in main." << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

