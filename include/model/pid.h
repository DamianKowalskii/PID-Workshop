#ifndef PID_H
#define PID_H
#include <algorithm>
namespace Model {
class PID {
 public:
  PID();
  PID(double _kp, double _ki, double _kd, double _out_min, double _out_max);
  ~PID();
  double calculate_output(double input, double setpoint, double dt);
  double get_kp();
  double get_ki();
  double get_kd();
  double get_out_max();
  double get_out_min();
  void set_kp(double _kp);
  void set_ki(double _ki);
  void set_kd(double _kd);
  void set_output_min(double _out_min);
  void set_output_max(double _out_max);
  void set_prevent_windup(bool prevent);
  void restart();

 private:
  double kp;
  double ki;
  double kd;
  double last_input;
  double error_sum;
  double out_min;
  double out_max;
  bool prevent_windup;
};
}  // namespace Model
#endif
