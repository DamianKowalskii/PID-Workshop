#ifndef SIMULATION_CONFIG_H
#define SIMULATION_CONFIG_H
#include <cstdint>
namespace Model {
namespace Simulation {
const uint32_t pid_loop_time_ms = 10;
const static double pid_tuning_scale_factor = 10e4;
}  // namespace Simulation
}  // namespace Model
#endif
