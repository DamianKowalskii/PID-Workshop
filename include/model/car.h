#ifndef CAR_H
#define CAR_H
#include <stdint.h>

#include <algorithm>
namespace Model {
const double AIR_DENSINITY = 1.225;
class Car {
 public:
  Car();
  Car(double _load, double _drag_cefficient, double _cross_section_area,
      double _thrust_parameter, double _mass);
  ~Car();
  double get_velocity();
  void set_gas_pedal_position(double _gas_pedal_position);
  void set_load(double _load);
  void drive(double dt);
  void restart();

 private:
  double velocity;
  double load;
  int32_t gas_pedal_position;
  double drag_coefficient;
  double cross_section_area;
  double thrust_parameter;
  double mass;
};
}  // namespace Model
#endif
