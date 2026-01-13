#ifndef PID_WRAP_HPP
#define PID_WRAP_HPP

#include <stdint.h> /* For uint64_t */

#ifdef __cplusplus
extern "C" {
#endif

/* * Returns the address of the new PID object as a 64-bit integer.
 * Simulink sees this as just a number.
 */
unsigned long long create_pid(float Ts, float max, float min, float K, float Kp, float Ki, float Kd, float clamp, float alpha);

/* * Takes the address as an integer.
 */
float run_pid(unsigned long long ptr_as_int, float setpoint, float measurement);

void delete_pid(unsigned long long ptr_as_int);

#ifdef __cplusplus
}
#endif
#endif