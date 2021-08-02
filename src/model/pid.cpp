
#include "pid.h"
namespace Model {
PID::PID() {
  kp = 0;
  ki = 0;
  kd = 0;
  out_min = 0;
  out_max = 0;
  last_input = 0;
  error_sum = 0;
  prevent_windup = false;
}
PID::PID(double _kp, double _ki, double _kd, double _out_min, double _out_max) {
  kp = _kp;
  ki = _ki;
  kd = _kd;
  out_min = _out_min;
  out_max = _out_max;
  last_input = 0;
  error_sum = 0;
  prevent_windup = false;
}
PID::~PID() {}
double PID::get_kp() { return kp; }
double PID::get_ki() { return ki; }
double PID::get_kd() { return kd; }
double PID::get_out_max() { return out_max; }
double PID::get_out_min() { return out_min; }
double PID::calculate_output(double input, double setpoint, double dt) {
/* Improvements to basic formula of PID controller
   source: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
*/
double error = setpoint - input;
  double d_input = (input - last_input) / dt;

  last_input = input;
  error_sum += ki * error * dt;
  double p_term = kp * error;
  double i_term = 0;

  if (prevent_windup) {
    i_term = std::clamp(error_sum, out_min, out_max);
  } else {
    i_term = error_sum;
  }
  double d_term = kd * d_input;
  double output = p_term + i_term + d_term;

  if (prevent_windup) {
    output = std::clamp(output, out_min, out_max);
  }
  return output;
}
void PID::set_kp(double _kp) { kp = _kp; }
void PID::set_ki(double _ki) { ki = _ki; }
void PID::set_kd(double _kd) { kd = _kd; }
void PID::set_output_min(double _out_min) { out_min = _out_min; }
void PID::set_output_max(double _out_max) { out_max = _out_max; }
void PID::set_prevent_windup(bool prevent) { prevent_windup = prevent; }

void PID::restart() {
  last_input = 0;
  error_sum = 0;
}
}  // namespace Model
