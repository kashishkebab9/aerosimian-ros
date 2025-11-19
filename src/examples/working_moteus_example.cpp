
#include <unistd.h>
#include <iostream>
#include <cstdio>

#include "moteus.h"

int main(int argc, char** argv) {
  using namespace mjbots;

  // Let moteus handle standard command-line transport options, etc.
  moteus::Controller::DefaultArgProcess(argc, argv);

  // Set up controller options (ID 1 by default).
  moteus::Controller::Options options;
  options.id = 1;

  moteus::Controller controller(options);

  // Clear any existing faults.
  controller.SetStop();

  // Prepare a simple position command.
  moteus::PositionMode::Command cmd;
  cmd.position = 0.36;   // target position in revolutions
  // All other fields (velocity, torque, gains) are left at their default NaN,
  // so the servo uses its configured internal values.

  while (true) {
    const auto maybe_result = controller.SetPosition(cmd);

    if (maybe_result) {
      const auto& r = maybe_result->values;
      // r.position is the fused position estimate (in revolutions),
      // derived from the encoder.
      std::printf("mode=%d  position=%.6f rev  velocity=%.6f rev/s  torque=%.6f Nm\n",
                  static_cast<int>(r.mode),
                  r.position,
                  r.velocity,
                  r.torque);
    } else {
      std::printf("no response from controller\n");
    }

    ::usleep(20000);  // 20 ms
  }

  return 0;
}

