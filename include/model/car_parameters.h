#ifndef CAR_PARAMETERS_H
#define CAR_PARAMETERS_H
#include <stdint.h>
namespace Model {
namespace Car_parameters {
const static double load = 100;
const static double drag_cefficient = 0.24;
const static double cross_section_area = 5.0;
const static double thrust_parameter = 20;
const static double mass = 1000;
const static double min_gas_pedal_position = -50;
const static double max_gas_pedal_position = 100;
}  // namespace Car_parameters
}  // namespace Model
#endif
