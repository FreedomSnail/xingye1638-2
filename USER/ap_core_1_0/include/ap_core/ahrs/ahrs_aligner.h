/**
 * @file subsystems/ahrs/ahrs_aligner.h
 *
 * Interface to align the AHRS via low-passed measurements at startup.
 *
 */

#ifndef AHRS_ALIGNER_H
#define AHRS_ALIGNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "algebra.h"

#define AHRS_ALIGNER_UNINIT  0
#define AHRS_ALIGNER_RUNNING 1
#define AHRS_ALIGNER_LOCKED  2

#define AHRS_ALIGNER_ACCEL_RUNNING 0
#define AHRS_ALIGNER_ACCEL_FINISH  1

struct AhrsAligner {
  struct FloatRates lp_gyro;
  struct Vector3 lp_accel;
  struct Vector3 lp_mag;
  int32_t           noise;
  int32_t           low_noise_cnt;
  uint8_t           status;
  uint8_t           accel_status;
};

extern struct AhrsAligner ahrs_aligner;

extern void ahrs_aligner_init(void);
extern void ahrs_aligner_begin(void);
extern void ahrs_aligner_run(struct FloatRates gyro);
extern void ahrs_aligner_get_init_accel_mag(struct Vector3 accel, struct Vector3 mag);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* AHRS_ALIGNER_H */
