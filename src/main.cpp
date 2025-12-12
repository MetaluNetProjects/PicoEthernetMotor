// Palantir main

#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "udp_server.hpp"
#include "wiznet.h"
#include "fraise.h"
#include "encoder.hpp"
#include "motor.hpp"

// motor on J1: 0=A 1=B 2=PWM 3=
Motor motor{0, 1, 2};
const int motor_current_pin = 26;

// encoder on J2: 4=A 5=B 6= 7=
Encoder encoder(4, 5);

// end switches on J3: 8=LOW 9=HIGH
const int endswitch_low = 8;
const int endswitch_high = 9;

UDPServer udp;

void gpio_callback(uint gpio, uint32_t events) {
	if(encoder.gpio_handler(gpio, events)) return;
}

void setup_end_switches() {
	gpio_init(endswitch_low);
	gpio_set_dir(endswitch_low, GPIO_IN);
	gpio_pull_up(endswitch_low);
	gpio_init(endswitch_high);
	gpio_set_dir(endswitch_high, GPIO_IN);
	gpio_pull_up(endswitch_high);

	gpio_set_irq_enabled_with_callback(endswitch_low, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
	//gpio_set_irq_enabled(pinA, GPIO_IRQ_EDGE_FALL, true);
	//gpio_set_irq_enabled(pinB, GPIO_IRQ_EDGE_FALL, true);
}

bool is_endswitch_low() {
	return gpio_get(endswitch_low) == 1;
}

bool is_endswitch_high() {
	return gpio_get(endswitch_high) == 1;
}

void print_end_switches() {
	static bool low = false, high = false;
	if(low != is_endswitch_low()) {
		low = is_endswitch_low();
		fraise_printf("end_low %d\n", low);
	}
	if(high != is_endswitch_high()) {
		high = is_endswitch_high();
		fraise_printf("end_high %d\n", high);
	}
}
#ifdef PICO_DEFAULT_LED_PIN
#define LED_PIN PICO_DEFAULT_LED_PIN
#endif
//#define LED_PIN 25

void set_led(bool l) {
	//cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, l ? 1 : 0);
	gpio_put(LED_PIN, l ? 1 : 0);
}

void blink(int ledPeriod) {
	static absolute_time_t nextLed;
	static bool led = false;

	if(time_reached(nextLed)) {
		set_led(led = !led);
		nextLed = make_timeout_time_ms(ledPeriod);
		//if(do_print_led) fraise_printf("led %d\n", led ? 1 : 0);
	}
}

void setup() {
	udp.setup(4343);
	gpio_set_irq_callback(gpio_callback);
	setup_end_switches();
	encoder.init();
	motor.init();
}

void loop() {
	blink(250);
	motor.update();
	motor.reset_watchdog();
	wiznet_update();
	print_end_switches();
}

void fraise_receivebytes(const char* data, uint8_t len) {
	char command = fraise_get_uint8();
	motor.reset_watchdog();
	switch(command) {
		case 1: {
				int pwm = fraise_get_int16();
				int ms = fraise_get_int16();
				motor.goto_pwm_ms(pwm, ms);
			}
			break;
		case 2:
			fraise_printf("l pwm %d\n", motor.get_pwm());
			break;
		case 10:
			fraise_printf("l m %d %f %d %d\n", encoder.get_count(), encoder.speed_process(), gpio_get(8), gpio_get(9));
			break;
	}
}

extern void print_version(); // from version.cpp

void fraise_receivechars(const char *data, uint8_t len){
	if(data[0] == 'E') { // Echo
		fraise_printf("E%s\n", data + 1);
	} else if(data[0] == 'V') { // Version
		print_version();
	} else fraise_printf("unknown %d\n", data[0]);
}


// ------------------ SYSTEM --------------

void fraise_putchar(char c) {
    static char line[64];
    static int count = 0;
    if(c == '\n') {
        line[count] = 0;
        udp.send_text(line, count);
        count = 0;
        return;
    }
    if(count < 64) line[count++] = c;
}

bool fraise_putbytes(const char* data, uint8_t len) { // returns true on success
    udp.send_bytes(data, len);
    return true;
}


int main() {
    //stdio_init_all();
    wiznet_init();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    setup();
    while (true) {
        loop();
    }
    return 0;
}

