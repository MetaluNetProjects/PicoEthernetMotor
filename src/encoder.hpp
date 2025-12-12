// Motor control

#pragma once
#include "fraise.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "math.h"

class Encoder {
private:
    uint pinA, pinB;
    volatile int count = 0;
    int lastcount = 0;
    float speed_lp1 = 0, speed_lp2 = 0;
    float lp_f = 0.66;
    absolute_time_t lasttime, last_irq_time;

public:
    Encoder(uint a, uint b): pinA(a), pinB(b) {}

    int get_count() {
        return count;
    }

    void init() {
        gpio_init(pinA);
        gpio_set_dir(pinA, GPIO_IN);
        gpio_pull_up(pinA);

        gpio_init(pinB);
        gpio_set_dir(pinB, GPIO_IN);
        gpio_pull_up(pinB);

        gpio_set_irq_enabled(pinA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
        gpio_set_irq_enabled(pinB, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }

    float speed_process()
    {
        uint32_t status = save_and_disable_interrupts();
        int dx = count - lastcount;
        lastcount = count;
        int dt = absolute_time_diff_us(lasttime, last_irq_time);
        lasttime = last_irq_time;
        restore_interrupts(status);
        float speed;
        if(!dt) speed = 0;
        else speed = (200000.0 * dx) / dt;
        //return speed;
        speed_lp1 += (speed - speed_lp1) * lp_f;
        speed_lp2 += (speed_lp1 - speed_lp2) * lp_f;
        return speed_lp2;
    }

    bool gpio_handler(uint gpio, uint32_t events)
    {
        bool on = gpio_get(gpio);
        if(gpio == pinA) {
            if(on) {
                if(gpio_get(pinB)) count++; else count--;
            } else {
                if(!gpio_get(pinB)) count++; else count--;
            }
        } else if(gpio == pinB) {
            if(on) {
                if(!gpio_get(pinA)) count++; else count--;
            } else {
                if(gpio_get(pinA)) count++; else count--;
            }
        } else return false;
        last_irq_time = get_absolute_time();
        return true;
    }
};
