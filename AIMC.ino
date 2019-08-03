#include "protocol.hpp"
#include <PID_v1.h>
#include <Encoder.h>
#include <Wire.h>

/// Compile-time settings
// Must be PWM pins
#define PWM_PIN_LEFT 5 
#define PWM_PIN_RIGHT 6 
// Must be interrupt pins
#define ENC_PIN_A 0
#define ENC_PIN_B 1
// Regular pins
#define LIMIT_SWC_PIN 10
// Slave address
#define SLAVE_ADDRESS 0x19
// Number of encoder counts to debounce at
#define ENCODER_ERROR_THRESHOLD 3

// Runtime variables
double current_encoder, pid_out, target, kp, ki, kd; 
int homing_pwm = 0;
int max_pwm = 255;
bool is_enabled = false;
bool is_homing = false;

// Runtime objects
PID pid(&current_encoder, &pid_out, &target, kp, ki, kd, DIRECT);
Encoder encoder(ENC_PIN_A, ENC_PIN_B);

void setup() {
    // Set up I2C bus
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(on_receive);
    Wire.onRequest(on_request);

    // Set up input and output pins
	pinMode(PWM_PIN_LEFT, OUTPUT);
	pinMode(PWM_PIN_RIGHT, OUTPUT);
	pinMode(ENC_PIN_A, INPUT);
	pinMode(ENC_PIN_B, INPUT);
	pinMode(LIMIT_SWC_PIN, INPUT_PULLUP);
	pinMode(LED_BUILTIN, OUTPUT);

    // Set PID coeffs
	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(-max_pwm, max_pwm);
	pid.SetSampleTime(5);

    update_enabled_led();
}

// Handle a new I2C request
void on_request() {
    // Send encoder position, target, and pid output in float format
    float encoder_float = current_encoder;
    Wire.write((unsigned char*)&encoder_float, 4);
    float target_float = target;
    Wire.write((unsigned char*)&target_float, 4);
    float pid_out_float = pid_out;
    Wire.write((unsigned char*)&pid_out_float, 4);
}

// Handle a new I2C message
void on_receive(int n_bytes) {
    if (n_bytes != 5) {
        while(Wire.available()) Wire.read();
        return;
    }
    char buffer[5] = {0};
    char* buffer_ptr = buffer;
    while(Wire.available()) { 
        *buffer_ptr++ = Wire.read();
    }
    Message msg = parse_message(buffer);
    handle_message(msg);
}

void handle_message(Message msg) {
    switch (msg.opcode) {
        case Enable:
            is_enabled = msg.content.u32 == 1;
            update_enabled_led();
            break;
        case Home:
            is_homing = msg.content.i32 != 0;
            homing_pwm = msg.content.i32;
            break;
        case SetTarget:
            target = msg.content.f32;
            break;
        case SetKp: 
            kp = msg.content.f32; 
            pid.SetTunings(kp, ki, kd);
            break;
        case SetKi: 
            ki = msg.content.f32; 
            pid.SetTunings(kp, ki, kd);
            break;
        case SetKd: 
            kd = msg.content.f32; 
            pid.SetTunings(kp, ki, kd);
            break;
        case LimitPwm:
            max_pwm = msg.content.u32;
	        pid.SetOutputLimits(-max_pwm, max_pwm);
            break;
        case EncoderPolarity:
            pid.SetControllerDirection(msg.content.u32 == 1 ? REVERSE : DIRECT);
            break;
        default:
            break;
    }
}

// Notify enabled state via status LED
void update_enabled_led() {
    digitalWrite(LED_BUILTIN, is_enabled ? HIGH : LOW);
}

// Set motor PWM, with polarity
void motor_set_pwm(int speed) {
	if (abs(speed) <= max_pwm) {
		if (speed > 0) {
			analogWrite(PWM_PIN_LEFT, speed);
			//analogWrite(PWM_PIN_RIGHT, 0);
            digitalWrite(PWM_PIN_RIGHT, LOW);
		} else {
			analogWrite(PWM_PIN_RIGHT, -speed);
			//analogWrite(PWM_PIN_LEFT, 0);
            digitalWrite(PWM_PIN_LEFT, LOW);
		}
	}
}

// Main program loop
void loop() {
    current_encoder = encoder.read();
	if (is_enabled) {
        enabled_loop();
	} else {
		motor_set_pwm(0);
	}
}

// Enabled state loop
inline void enabled_loop() {
    if (is_homing) {
        homing_loop();
    } else {
        control_loop();
    }
}

// Homing loop
inline void homing_loop() {
    if (digitalRead(LIMIT_SWC_PIN) == LOW) {
        is_homing = false;
        current_encoder = 0.0;
        target = current_encoder;
        encoder.write(current_encoder);
    } else {
        motor_set_pwm(homing_pwm);	
    }
}

inline void control_loop() {
    pid_control();
}

inline void pid_control() {
    if (abs(current_encoder - target) > ENCODER_ERROR_THRESHOLD) {
        if (pid.Compute()) {
            motor_set_pwm((int)pid_out);
        }
    } else {
        motor_set_pwm(0);
    }
}
