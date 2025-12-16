// Motor control

#pragma once
#include "fraise.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/mutex.h"
#include "math.h"

class Motor {
private:
    const int PWM_MAX = 5555; // 22.5kHz
    const int period_ms = 10;
    absolute_time_t next_time = get_absolute_time();
    absolute_time_t watchdog_time = at_the_end_of_time;
    int pinA, pinB, pinPwm;
    int pwm = 0; // pwm [-32768, 32767]
    int pwm_target = 0;
    int full_scale_ms = 20; // 16 second for pwm fullscale
    int accel = 0;
    int watchdog_ms = 5000;
    mutex_t mutex;

    void set_pwm(int p){
        pwm = p;
        int pwm_real = (pwm * PWM_MAX) / 32768;
        pwm_set_gpio_level(pinPwm, abs(pwm_real));
        if(pwm > 0) {
            gpio_put(pinA, 1);
            gpio_put(pinB, 0);
        } else {
            gpio_put(pinA, 0);
            gpio_put(pinB, 1);
        }
    }

public:
    Motor(int a, int b, int p): pinA(a), pinB(b), pinPwm(p) {
        mutex_init(&mutex);
    }

    void init() {
        gpio_init(pinA);
        gpio_put(pinA, 0);
        gpio_set_dir(pinA, GPIO_OUT);

        gpio_init(pinB);
        gpio_put(pinB, 0);
        gpio_set_dir(pinB, GPIO_OUT);

        gpio_set_function(pinPwm, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(pinPwm);
        pwm_set_wrap(slice_num, PWM_MAX);
        pwm_set_enabled(slice_num, true);
        pwm_set_gpio_level(pinPwm, 0);

        pwm_target = pwm = 0;
    }

    void reset_watchdog() {
        watchdog_time = make_timeout_time_ms(watchdog_ms);
    }

    void goto_pwm_ms(int p, int ms) {
        pwm_target = p;
        accel = ((pwm_target - pwm) * period_ms) / abs(ms);
        int accel_max = (32768 * period_ms) / full_scale_ms;
        if(accel > accel_max) accel = accel_max;
        else if(accel < -accel_max) accel = -accel_max;
        //fraise_printf("l accel %d\n", accel);
    }

    void update() {
        if(! mutex_try_enter(&mutex, NULL)) {
            if(time_reached(next_time)) {
                next_time = make_timeout_time_ms(period_ms);
                if(accel > 0) pwm = MIN(pwm_target, pwm + accel);
                else pwm = MAX(pwm_target, pwm + accel);
                set_pwm(pwm);
            }
            if(watchdog_ms && time_reached(watchdog_time)) {
                goto_pwm_ms(0, full_scale_ms);
                watchdog_time = at_the_end_of_time;
            }
            mutex_exit(&mutex);
        }
    }

    int get_pwm() {
        return pwm;
    }
};
