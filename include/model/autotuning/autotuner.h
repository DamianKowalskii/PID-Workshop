#ifndef AUTOTUNER_H
#define AUTOTUNER_H
#include <stdint.h>

#include <vector>

#include "car.h"
#include "pid.h"
#include "population.h"
namespace Model {
namespace Autotuning {
typedef struct sample_gains_t {
  double kp;
  double ki;
  double kd;
} sample_gains_t;
class Autotuner {
 public:
  Autotuner();
  ~Autotuner();
  double get_the_best_kp();
  double get_the_best_ki();
  double get_the_best_kd();
  void run_autotuning();

 private:
  void simulate_pid(Model::PID pid, std::vector<double>& time,
                    std::vector<double>& values, int32_t simulation_loops);

 private:
  population_t population;
  Model::Car* car;
  sample_gains_t the_best_tunings;
};
}  // namespace Autotuning
}  // namespace Model
#endif
