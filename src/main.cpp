#include "rovercontrol.h"
#include <rerun.hpp>
#include <iostream>

int main() {
    try {
        // 1. Initialize Rerun and the Controller
        auto rec = rerun::RecordingStream("rover_simulation");
        rec.spawn().exit_on_failure();
        RoverControl controller(rec);

        // 2. Perform initial setup (e.g., get user input for goal)
        controller.setup();

        // 3. Main application loop
        bool needs_planning = false;
        while (true) { // Replace with your actual exit condition
            if (needs_planning) {
                // If planning is needed, run the planning step.
                // This step will run until it's done or aborted, then switch the flag.
                controller.runPathPlanning();
                needs_planning = false; // Assume planning is done, switch back to mapping
            } else {
                // Run a mapping step. The method returns true if it decides
                // that path planning is now required.
                if (controller.runMapping()) {
                    needs_planning = true;
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

