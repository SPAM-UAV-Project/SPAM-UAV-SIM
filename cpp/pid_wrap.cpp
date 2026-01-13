#include "pid_wrap.hpp"
#include "pid_dom.hpp"

extern "C" {
    unsigned long long create_pid(float Ts, float max, float min, float K, float Kp, float Ki, float Kd, float clamp, float alpha) {
        PID_DOM* ptr = new PID_DOM(Ts, max, min, K, Kp, Ki, Kd, clamp, alpha);
        // Cast pointer to integer to hide it from Simulink
        return (unsigned long long)ptr;
    }

    float run_pid(unsigned long long ptr_as_int, float setpoint, float measurement) {
        if (ptr_as_int == 0) return 0.0f;
        
        // Cast integer back to pointer
        PID_DOM* obj = (PID_DOM*)ptr_as_int;
        return obj->run(setpoint, measurement);
    }

    void delete_pid(unsigned long long ptr_as_int) {
        if (ptr_as_int != 0) {
            PID_DOM* obj = (PID_DOM*)ptr_as_int;
            delete obj;
        }
    }
}