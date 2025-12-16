// Ramp generator

#pragma once
#include "fraise.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/mutex.h"
#include "math.h"

class Ramp {
private:
    float accel = 1.0f;
    float position = 0.0f;
    float speed = 0.0f;
    float maxspeed = 100.0;
    float destination = 0.0f;
    absolute_time_t last_time = get_absolute_time();

public:
    Ramp(float _accel, float _maxspeed): accel(_accel), maxspeed(_maxspeed) {}
    
    void set_destination(float d){
        destination = d;
    }

    float get_position(){
        return position;
    }

    void set(float d){
        destination = position = d;
        speed = 0.0f;
    }

    void set_maxspeed(float _maxspeed){
        maxspeed = _maxspeed;
    }

    void set_accel(float _accel){
        accel = _accel;
    }

    void compute() {
        float error = destination - position;
        float stop_distance = abs(speed) * speed / (2.0 * accel);
        float dt = absolute_time_diff_us(last_time, get_absolute_time()) / (float)1e6;
        if(dt == 0.0) return;
        float dv = accel * dt;
        last_time = get_absolute_time();
        if(abs(error) < 1.0f && (abs(speed) < accel * 0.1)) {
            speed = 0;
            position = destination;
            return;
        }
        if(error > 0) {
            if(error > stop_distance) {
                if(speed < maxspeed) {
                    speed = MIN(speed + dv, maxspeed);
                }
            } else {
                if(speed > 0) speed -= dv;
            }
        } else if (error < 0) {
            if(error < stop_distance) {
                if(speed > -maxspeed) {
                    speed = MAX(speed - dv, -maxspeed);
                }
            } else {
                if(speed < 0) speed += dv;
            }
        }
        if(speed > maxspeed) speed -= dv;
        else if(speed < -maxspeed) speed += dv;
        
        position += speed * dt;
    }
};
