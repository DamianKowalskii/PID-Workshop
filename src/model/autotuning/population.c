
#include "population.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
static const float MIN_PROBABILITY = 0.000001f;
static const  float MAX_PROBABILITY = 0.999999f;
static void init_population(float *population, uint32_t genome_size);
static void init_rng();
static float random_float(float min, float max);
void population_ctor(population_t *me, uint32_t population_size,
                     uint32_t genome_lenght) {
  me->population_size = population_size;
  me->genome_length = genome_lenght;
  me->vector =(float *) malloc(sizeof(float) * me->genome_length);
  init_population(me->vector, me->genome_length);
  init_rng();
}
void population_dtor(population_t *me){
free(me->vector);
}
static void init_population(float *population, uint32_t genome_size) {
  for (size_t i = 0; i < genome_size; i++) {
    population[i] = 0.5f;
  }
}
void population_update(population_t *me, uint8_t *winner, uint8_t *loser) {
  for (size_t i = 0; i < me->genome_length; i++) {
    if (winner[i] != loser[i]) {
      if (winner[i] == 1) {
        me->vector[i] = me->vector[i] + (1.0f / (float)me->population_size);
      } else {
        me->vector[i] = me->vector[i] - (1.0f / (float)me->population_size);
      }
    }
    if (me->vector[i] < MIN_PROBABILITY) {
      me->vector[i] = MIN_PROBABILITY;
    }
    if (me->vector[i] > MAX_PROBABILITY) {
      me->vector[i] = MAX_PROBABILITY;
    }
  }
}
void population_generate_sample(population_t *me, uint8_t *sample) {
  for (size_t i = 0; i < me->genome_length; i++) {
    float probability = me->vector[i];
    if (probability > random_float(0.0f, 1.0f)) {
      sample[i] = 1;
    } else {
      sample[i] = 0;
    }
  }
}
void population_restart(population_t *me)
{
     init_population(me->vector, me->genome_length);
}
bool population_is_converge(population_t *me) {
  bool converge = true;
  for (size_t l = 0; l < me->genome_length; l++) {
    if (me->vector[l] > MIN_PROBABILITY && me->vector[l] < MAX_PROBABILITY) {
      converge = false;
    }
  }
  return converge;
}
static float random_float(float min, float max) {
  float scale = rand() / (float)RAND_MAX;
  return min + scale * (max - min);
}
static void init_rng() {
  time_t tt;
  int32_t seed = time(&tt);
  srand(seed);
}


