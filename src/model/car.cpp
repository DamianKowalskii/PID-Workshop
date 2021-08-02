#include "car.h"

#include <algorithm>
#include <cmath>

#include "car_parameters.h"

//Car model source: https://apmonitor.com/pdc/index.php/Main/SpeedControl
namespace Model {
Car::Car() {
  load = 0;
  drag_coefficient = 0;
  cross_section_area = 0;
  thrust_parameter = 0;
  mass = 0;
  velocity = 0;
  gas_pedal_position = 0;
}
Car::Car(double _load, double _drag_cefficient, double _cross_section_area,
         double _thrust_parameter, double _mass) {
  load = _load;
  drag_coefficient = _drag_cefficient;
  cross_section_area = _cross_section_area;
  thrust_parameter = _thrust_parameter;
  mass = _mass;
  velocity = 0;
  gas_pedal_position = 0;
}
Car::~Car() {}
double Car::get_velocity() { return velocity; }
void Car::set_gas_pedal_position(double _gas_pedal_position) {
  gas_pedal_position = std::clamp(
      _gas_pedal_position, Model::Car_parameters::min_gas_pedal_position,
      Model::Car_parameters::max_gas_pedal_position);
}
void Car::set_load(double _load) { load = _load; }
void Car::drive(double dt) {
  double d_velocity =
      (1.0 / (mass + load)) * (thrust_parameter * gas_pedal_position -
                               (0.5 * AIR_DENSINITY * drag_coefficient *
                                cross_section_area * std::pow(velocity, 2)));
  d_velocity *= dt;
  velocity += d_velocity;
  if (velocity < 0) {
    velocity = 0;
  }
}

void Car::restart() { velocity = 0; }
}  // namespace Model
