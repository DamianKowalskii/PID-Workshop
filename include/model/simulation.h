#ifndef SIMULATION_H
#define SIMULATION_H
#include <QPair>
namespace Model {
namespace Simulation {
typedef struct simulation_vars {
  double gas_pedal = 0;
  double velocity = 0;
  double setpoint = 0;
  double accumulate_time = 0;
} simulation_vars_t;
typedef struct simulation_sample {
  QPair<double, double> car_output_now;
  QPair<double, double> setpoint_now;
  QPair<double, double> pid_output_now;
} simulation_sample_t;
}  // namespace Simulation
}  // namespace Model
#endif
