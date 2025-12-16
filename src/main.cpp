// PicoEthernetMotor main

#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "udp_server.hpp"
#include "wiznet.h"
#include "fraise.h"
#include "encoder.hpp"
#include "motor.hpp"
#include "ramp.hpp"
#include "PID_v1.h"

// PID settings:
const int PID_KP = 370;
const int PID_KI = 650;
const int PID_KD = 0.1;

// RAMP settings:
const int RAMP_ACCEL = 2000;    // steps/s²
const int RAMP_MAXSPEED = 5400; // steps/s
const int HOMING_PWM = 6000;    // max 32767 (but don't do that ;-)

// CURRENT SECURITY
const int OVERCURRENT_MA = 800;
const int OVERCURRENT_MS = 2000;

// LOWSWITCH MARGIN
const int LOWSWITCH_MARGIN = 2000;

// motor on J1: 0=A 1=B 2=PWM 3=
Motor motor{0, 1, 2};
const int motor_current_pin = 26;

// encoder on J2: 4=A 5=B 6= 7=
Encoder encoder(4, 5);

// end switches on J3: 8=LOW 9=HIGH
const int endswitch_low = 8;
const int endswitch_high = 9;

UDPServer udp;

float position;
float pwm;
float setPoint;

PID pid(&position, &pwm, &setPoint, PID_KP, PID_KI, PID_KD, P_ON_M, DIRECT);
Ramp ramp(RAMP_ACCEL, RAMP_MAXSPEED);
bool ramp_to_pid = true;

bool unlock_settings = false;
bool unlock_debug = false;

enum class State{init, homing, finishhoming, run, error} state = State::init;
const char *state_name(State s) {
	const char* name;
	switch(s) {
	case(State::init): name = "init"; break;
	case(State::homing): name = "homing"; break;
	case(State::finishhoming): name = "finishhoming"; break;
	case(State::run): name = "run"; break;
	case(State::error): name = "error"; break;
	}
	return name;
}

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

	// endswitch_low interrupt is not processed, but this enables interrupts for encoder pins:
	gpio_set_irq_enabled_with_callback(endswitch_low, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
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

void setup_motor_current() {
	adc_init();
	adc_gpio_init(motor_current_pin);
}

int get_motor_current_mA(bool update = false) {
	static float current_raw = 0;
	static float current_filtered = 0;
	if(update) {
		adc_select_input(motor_current_pin - 26);
		current_raw = adc_read();
		current_filtered += (current_raw - current_filtered) * 0.05;
	}
	// 140 mV per amp
	return current_filtered * (1000.0 * 3.3 / (4096.0 * 0.140));
}

#ifdef PICO_DEFAULT_LED_PIN
#define LED_PIN PICO_DEFAULT_LED_PIN
#endif

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
	}
}

void enableMotorControl(bool enable) {
	if(enable) {
		position = setPoint = encoder.get_count();
		ramp.set(setPoint);
	}
	pid.SetMode(enable ? 1 : 0);
	ramp_to_pid = enable;
}

void setup() {
	udp.setup(4343);
	gpio_set_irq_callback(gpio_callback);
	setup_end_switches();
	setup_motor_current();
	encoder.init();
	motor.init();
	pid.SetOutputLimits(-32767.0, 32767.0);
	pid.SetSampleTime(5);
	enableMotorControl(false);
}

void motor_updatepwm() {
	if(is_endswitch_low() && pwm < 0) pwm = 0;
	if(is_endswitch_high() && pwm > 0) pwm = 0;
	motor.goto_pwm_ms(pwm, 10);
}

// stop motor and set mode to error
void state_set_error() {
	state = State::error;
	enableMotorControl(false);
	pwm = 0;
}

void motor_check_current() {
	static absolute_time_t alert_time = at_the_end_of_time;
	if(get_motor_current_mA() > OVERCURRENT_MA) {
		if(alert_time == at_the_end_of_time) {
			alert_time = make_timeout_time_ms(OVERCURRENT_MS);
		} else if(time_reached(alert_time)) {
			state_set_error();
			fraise_printf("e overcurrent error!\n");
			alert_time = at_the_end_of_time;
		}
	} else alert_time = at_the_end_of_time;
}

void motorcontrol_update() {
	position = encoder.get_count();
	if(pid.Compute()) {
		ramp.compute();
		if(ramp_to_pid) setPoint = ramp.get_position();
		motor_updatepwm();
	}
	motor.update();
	motor.reset_watchdog();
	get_motor_current_mA(true);
	motor_check_current();
}

// if endswitch low and/or high, set mode to error and return false.
// else, return true ('good');
bool check_endswitches(bool low, bool high) {
	if((is_endswitch_low() && low) || (is_endswitch_high() && high)) {
		state_set_error();
		fraise_printf("e endswitch error!\n");
		return false;
	}
	return true;
}

void state_update() {
	switch(state) {
	case State::init:
		break;
	case State::homing:
		if(is_endswitch_low()) {
			encoder.set_count(-LOWSWITCH_MARGIN);
			enableMotorControl(true);
			ramp.set_destination(0);
			state = State::finishhoming;
		}
		check_endswitches(false, true);
		break;
	case State::finishhoming:
		if(encoder.get_count() > -100) {
			if(check_endswitches(true, true)) {
				state = State::run;
			}
		}
		check_endswitches(false, true);
		break;
	case State::run:
		check_endswitches(true, true);
		break;
	case State::error:
		break;
	}

	static absolute_time_t nextSendTime = 0;
	if(time_reached(nextSendTime)) {
		nextSendTime = make_timeout_time_ms(500);
		fraise_printf("state %s\n", state_name(state));
	}
}

void loop() {
	blink(250);
	state_update();
	motorcontrol_update();
	wiznet_update();
	print_end_switches();
}

void fraise_receivebytes(const char* data, uint8_t len) {
	char command = fraise_get_uint8();
	motor.reset_watchdog();
	switch(command) {
		case 1:
			fraise_printf("l pwm %d\n", motor.get_pwm());
			break;
		case 2:
			fraise_printf("l current %d\n", get_motor_current_mA());
			break;
		case 3:
			fraise_printf("l m %d %f\n", encoder.get_count(), encoder.speed_process());
			break;
		case 4:
			fraise_printf("l ramp %f\n", ramp.get_position());
			break;
		case 10: // HOMING
			enableMotorControl(false);
			state = State::homing;
			pwm = -1 * (HOMING_PWM);
			break;
		case 11: // GOTO
			ramp.set_destination(fraise_get_int32());
			break;
		case 12: // SPEED
			ramp.set_maxspeed(fraise_get_int32());
			break;
		case 20:
			unlock_settings = (fraise_get_uint8() > 0);
			if(!unlock_settings) unlock_debug = false;
			break;
	}

	if(unlock_settings) switch(command) {
		case 100: {
				pwm = fraise_get_int16();
				motor_updatepwm();
				enableMotorControl(false);
			}
			break;
		case 110:
			pid.SetTunings(fraise_get_int32() / 1000.0, fraise_get_int32() / 1000.0, fraise_get_int32() / 1000.0);
			break;
		case 120:
			ramp.set_accel(fraise_get_int32());
			break;
		case 200:
			unlock_debug = (fraise_get_uint8() > 0);
			break;
	}

	if(unlock_debug) switch(command) {
		case 210:
			encoder.set_count(fraise_get_int32());
			break;
		case 220:
			enableMotorControl(fraise_get_uint8() > 0);
			break;
		case 230:
			setPoint = fraise_get_int32();
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
    //stdio_init_all(); // done by wiznet_init() after setting CPU clock
    wiznet_init();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    setup();
    while (true) {
        loop();
    }
    return 0;
}

