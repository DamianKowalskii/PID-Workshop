#ifndef AUTOTUNING_CONFIG_H
#define AUTOTUNING_CONFIG_H
#include <stdint.h>
namespace Model {
namespace Autotuning {
static const double test_setpoint = 1;
static const int32_t genome_length_bits = 30;
static const int32_t genome_chunk_size_bits = 10;
static double pid_loop_time_s = 0.01;
static uint32_t simulation_time_s = 30;
const static int32_t generations = 8000;
const static uint32_t population_size = 25;
}  // namespace Autotuning
}  // namespace Model
#endif
