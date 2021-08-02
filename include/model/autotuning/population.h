#ifndef CGA_POPULATION_H
#define CGA_POPULATION_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>
typedef struct population {
  uint32_t population_size;
  uint32_t genome_length;
  float *vector;
} population_t;
void population_ctor(population_t *me, uint32_t population_size,
                     uint32_t genome_lenght);
void population_dtor(population_t *me);
void population_generate_sample(population_t *me, uint8_t *sample);
void population_update(population_t *me, uint8_t *winner, uint8_t *loser);
void population_restart(population_t *me);
bool population_is_converge(population_t *me);
#ifdef __cplusplus
}
#endif
#endif  // CGA_POPULATION_H
