#include "autotuner.h"

#include <QBitArray>
#include <QProgressDialog>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <tuple>
#include <functional>

#include "autotuning_config.h"
#include "car_parameters.h"
#include "pid.h"
#include "population.h"
namespace Model {
namespace Autotuning {
static const int32_t initial_durbin_watson_statistic = -1;
typedef struct pid_rating {
  double overshoot_percentage;
  double rise_time_s;
  double peak_time_s;
  bool check_durpin_watson_statistic;
  double durpin_watson_statistic_distance_to_ideal;
} pid_rating_t;
typedef struct sample {
  uint8_t *bits;
  uint32_t fitness;
} sample_t;
static double predict(double row, std::tuple<double, double> coefficients) {
  double intercept = std::get<0>(coefficients);
  double result = intercept;
  result += (row * std::get<1>(coefficients));
  return result;
}
static double convert_genome_bits_to_double(uint8_t *genome,
                                            int32_t index_start,
                                            int32_t index_stop,
                                            uint32_t chunk_size) {
  QBitArray qba(chunk_size, false);
  uint32_t qba_iterator = 0;
  for (int i = index_start; i <= index_stop; ++i, ++qba_iterator) {
    if (genome[i] == 1) {
      qba[qba_iterator] = 1;
    }
  }
  // Each of the PID gain is coded by 10 bits in the genome and treated as
  // double value after conversion.

  int16_t value = static_cast<int16_t>(qba.toUInt32(QSysInfo::BigEndian));
  double final_double = static_cast<double>(value) / 1000;
  return final_double;
}
static void rate_pid(pid_rating_t *pid_rating, std::vector<double> x,
                     std::vector<double> y, double setpoint, double dt) {
    double overshoot_percentage =
        (*std::max_element(y.begin(), y.end()) / setpoint) * 100;
    bool count_time = false;
    double counter = 0;
    for (size_t i = 0; i < x.size(); ++i) {
        if (y[i] >= (0.05 * setpoint)) {
            count_time = true;
        }
        if (y[i] >= (0.9 * setpoint)) {
            break;
        }
        if (count_time == true) {
            counter += dt;
        }
    }
    double rise_time_s = counter;
    auto it = std::max_element(y.begin(), y.end());
    uint32_t peak_time_index = std::distance(y.begin(), it);
    double peak_time_s = peak_time_index * dt;

    uint32_t reaching_setpoint_index = 0;
    /* Check for reaching setpoint, because the generated sample can have other
     good metrics,but can oscillate. Reaching setpoint is necessary for looking
     at oscillations around settle point.
  */
    bool is_setpoint_reached = false;
    for (auto &element : y) {
        ++reaching_setpoint_index;
        if (element >= setpoint) {
            is_setpoint_reached = true;
            break;
        }
    }
    double durpin_watson = initial_durbin_watson_statistic;
    pid_rating->check_durpin_watson_statistic = false;
    if (is_setpoint_reached) {
        /* If the signal is +/-20% of setpoint for 50% of simulation time,
       we can say that there should be no oscillations.
       It can be just big stead state error!
    */
        auto signal_after_reaching_setpoint =
            std::vector<double>(y.begin() + reaching_setpoint_index, y.end());
        double samples_out_of_safe_value = 0;
        for (auto &element : signal_after_reaching_setpoint) {
            bool is_sample_out_of_safe_value =
                (element > 1.2 * setpoint) || (element < 0.8 * setpoint);
            if (is_sample_out_of_safe_value) {
                ++samples_out_of_safe_value;
            }
        }
        bool is_signal_unstable_after_setpoint =
            (samples_out_of_safe_value /
             static_cast<double>(signal_after_reaching_setpoint.size())) >= 0.5;

        if (is_signal_unstable_after_setpoint) {
            // Source:
            // https://www.investopedia.com/terms/d/durbin-watson-statistic.asp
            /* Durbin–Watson statistic is used to detect any significant
        autocorrelation in signal after reaching setpoint. To get Durbin-Watson
        statistics we need to train simple model to perform Least Squares
        Regression on data points.
      */

            double times_mean = std::accumulate(x.begin(), x.end(), 0.0) / x.size();
            double signal_mean = std::accumulate(y.begin(), y.end(), 0.0) / y.size();

            std::vector<double> times_difference(x.size());
            std::vector<double> signal_difference(y.size());
            std::transform(x.begin(), x.end(), times_difference.begin(),
                           std::bind(std::minus<>(), std::placeholders::_1, times_mean));
            std::transform(y.begin(), y.end(), signal_difference.begin(),
                           std::bind(std::minus<>(), std::placeholders::_1, signal_mean));

            std::vector<double> product(x.size());
            std::transform(times_difference.begin(), times_difference.end(), signal_difference.begin(),
                           product.begin(), std::multiplies<>());

            double numerator = std::accumulate(product.begin(), product.end(), 0.0);
            double denominator = std::inner_product(times_difference.begin(), times_difference.end(),
                                                    times_difference.begin(), 0.0);

            double b1 = numerator / denominator;
            double b0 = signal_mean - (b1 * times_mean);
            std::tuple<double, double> coefficients(b0, b1);

            std::vector<double> predicted_signal(x.size());
            std::transform(x.begin(), x.end(), predicted_signal.begin(),
                           [&coefficients](double t) { return predict(t, coefficients); });

            std::vector<double> errors(y.size());
            std::transform(y.begin(), y.end(), predicted_signal.begin(), errors.begin(), std::minus<>());

            double squared_summed_errors = std::inner_product(errors.begin(), errors.end(), errors.begin(), 0.0);

            std::vector<double> reverse_errors = errors;
            std::reverse(reverse_errors.begin(), reverse_errors.end());
            double sum_of_difference = std::inner_product(
                reverse_errors.begin(), reverse_errors.end() - 1,
                reverse_errors.begin() + 1,
                0.0,
                std::plus<>(),
                std::minus<>()
                );

            /* The rule of thumb is that a value between 1.5-2.5 indicates,
         that there is no autocorrelation in the signal.
      */
            durpin_watson = sum_of_difference / squared_summed_errors;
            pid_rating->check_durpin_watson_statistic = true;
        }
    }
    pid_rating->durpin_watson_statistic_distance_to_ideal =
        abs(2 - durpin_watson);
    pid_rating->overshoot_percentage = overshoot_percentage;
    pid_rating->peak_time_s = peak_time_s;
    pid_rating->rise_time_s = rise_time_s;
}

static void get_sample_gains_from_genome(
    Model::Autotuning::sample_gains_t *gains, uint8_t *sample) {
  gains->kp = convert_genome_bits_to_double(
      sample, 0, genome_chunk_size_bits - 1, genome_chunk_size_bits);
  gains->ki = convert_genome_bits_to_double(sample, genome_chunk_size_bits,
                                            (2 * genome_chunk_size_bits) - 1,
                                            genome_chunk_size_bits);
  gains->kd = convert_genome_bits_to_double(sample, 2 * genome_chunk_size_bits,
                                            (3 * genome_chunk_size_bits) - 1,
                                            genome_chunk_size_bits);
}
void fitness(sample_t *sample_a, sample_t *sample_b, pid_rating_t pid_a_rating,
             pid_rating_t pid_b_rating) {
  // Deciding which sample perform better is based on ordinary scoring points.
  // The more points the better.
  if (pid_a_rating.overshoot_percentage < pid_b_rating.overshoot_percentage) {
    sample_a->fitness++;
  } else {
    sample_b->fitness++;
  }
  if (pid_a_rating.rise_time_s < pid_b_rating.rise_time_s) {
    sample_a->fitness++;
  } else {
    sample_b->fitness++;
    ;
  }
  if (pid_a_rating.peak_time_s < pid_b_rating.peak_time_s) {
    sample_a->fitness++;
  } else {
    sample_b->fitness++;
  }
  if ((pid_a_rating.check_durpin_watson_statistic) &&
      (pid_b_rating.check_durpin_watson_statistic)) {
    if (pid_a_rating.durpin_watson_statistic_distance_to_ideal <
        pid_b_rating.durpin_watson_statistic_distance_to_ideal) {
      sample_a->fitness++;
    } else {
      sample_b->fitness++;
    }
  }

  if (sample_a->fitness == sample_b->fitness) {
    sample_a->fitness++;
  }
}
Autotuner::Autotuner() {
  car = new Model::Car(
      Model::Car_parameters::load, Model::Car_parameters::drag_cefficient,
      Model::Car_parameters::cross_section_area,
      Model::Car_parameters::thrust_parameter, Model::Car_parameters::mass);
  population_ctor(&population, Model::Autotuning::population_size,
                  Model::Autotuning::genome_length_bits);
}

Autotuner::~Autotuner() {
  delete car;
  population_dtor(&population);
}

double Model::Autotuning::Autotuner::get_the_best_kp() {
  return the_best_tunings.kp;
}
double Model::Autotuning::Autotuner::get_the_best_ki() {
  return the_best_tunings.ki;
}
double Model::Autotuning::Autotuner::get_the_best_kd() {
  return the_best_tunings.kd;
}
void Model::Autotuning::Autotuner::simulate_pid(Model::PID pid,
                                                std::vector<double> &time,
                                                std::vector<double> &values,
                                                int32_t simulation_loops) {
  car->restart();
  double velocity = 0;
  double gas_pedal = 0;
  double acuumulated_time = 0;
  for (int i = 0; i < simulation_loops; ++i) {
    velocity = car->get_velocity();
    gas_pedal = pid.calculate_output(velocity, Model::Autotuning::test_setpoint,
                                     Model::Autotuning::pid_loop_time_s);
    car->set_gas_pedal_position(gas_pedal);
    car->drive(Model::Autotuning::pid_loop_time_s);
    acuumulated_time += Model::Autotuning::pid_loop_time_s;
    time.push_back(acuumulated_time);
    values.push_back(velocity);
  }
}
void Model::Autotuning::Autotuner::run_autotuning() {
  population_restart(&population);
  int32_t simulation_loops = Model::Autotuning::simulation_time_s * 100;
  std::vector<double> x;
  std::vector<double> y;
  sample_t sample_a;
  sample_a.bits = (uint8_t *)malloc(sizeof(uint8_t) *
                                    Model::Autotuning::genome_length_bits);
  sample_a.fitness = 0;
  sample_t sample_b;
  sample_b.bits = (uint8_t *)malloc(sizeof(uint8_t) *
                                    Model::Autotuning::genome_length_bits);
  sample_b.fitness = 0;

  for (int j = 0; j < Model::Autotuning::generations; ++j) {
    population_generate_sample(&population, sample_a.bits);
    population_generate_sample(&population, sample_b.bits);
    Model::Autotuning::sample_gains_t gains_a;
    Model::Autotuning::sample_gains_t gains_b;
    get_sample_gains_from_genome(&gains_a, sample_a.bits);
    get_sample_gains_from_genome(&gains_b, sample_b.bits);

    Model::PID pid_a(gains_a.kp, gains_a.ki, gains_a.kd,
                     Model::Car_parameters::min_gas_pedal_position,
                     Model::Car_parameters::max_gas_pedal_position);
    Model::PID pid_b(gains_b.kp, gains_b.ki, gains_b.kd,
                     Model::Car_parameters::min_gas_pedal_position,
                     Model::Car_parameters::max_gas_pedal_position);

    pid_a.set_prevent_windup(true);
    pid_b.set_prevent_windup(true);

    simulate_pid(pid_a, x, y, simulation_loops);
    Model::Autotuning::pid_rating_t pid_a_rating;
    rate_pid(&pid_a_rating, x, y, Model::Autotuning::test_setpoint,
             Model::Autotuning::pid_loop_time_s);
    x.clear();
    y.clear();
    simulate_pid(pid_b, x, y, simulation_loops);
    Model::Autotuning::pid_rating_t pid_b_rating;
    rate_pid(&pid_b_rating, x, y, Model::Autotuning::test_setpoint,
             Model::Autotuning::pid_loop_time_s);

    fitness(&sample_a, &sample_b, pid_a_rating, pid_b_rating);
    Model::Autotuning::pid_rating_t the_best_rating;
    uint8_t winner[Model::Autotuning::genome_length_bits];
    uint8_t loser[Model::Autotuning::genome_length_bits];
    if (sample_a.fitness > sample_b.fitness) {
      memcpy(winner, sample_a.bits, Model::Autotuning::genome_length_bits);
      memcpy(loser, sample_b.bits, Model::Autotuning::genome_length_bits);
      the_best_rating = pid_a_rating;
      the_best_tunings = gains_a;
    } else {
      memcpy(winner, sample_b.bits, Model::Autotuning::genome_length_bits);
      memcpy(loser, sample_a.bits, Model::Autotuning::genome_length_bits);
      the_best_rating = pid_b_rating;
      the_best_tunings = gains_b;
    }
    population_update(&population, winner, loser);
    if (population_is_converge(&population)) {
      break;
    }
  }
  free(sample_a.bits);
  free(sample_b.bits);
}

}  // namespace Autotuning
}  // namespace Model
