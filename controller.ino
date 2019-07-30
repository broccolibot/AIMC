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
#define ERROR_THRESHOLD 0.01

// Runtime variables
double current_encoder, pid_out, target, kp, ki, kd; 
int homing_pwm = 0;
int max_pwm = 0;

// Runtime objects
PID pid(&current_encoder, &pid_out, &target, kp, ki, kd, DIRECT);
Encoder encoder(ENC_PIN_A, ENC_PIN_B);
bool is_enabled = false;
bool is_homing = false;

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

    // Set PID coeffs
	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(0, 0);
	pid.SetSampleTime(5);

    Serial.begin(9600);
}

void on_request() {
    float encoder_float = current_encoder;
    Wire.write((unsigned char*)&encoder_float, 4);
}

void on_receive(int n_bytes) {
    if (n_bytes != 5) {
        Serial.println("Wrong number of bytes!");
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
        Enable:
            is_enabled = msg.content.u32 == 1;
            break;
        Home:
            is_homing = msg.content.u32 == 1;
            break;
        SetTarget:
            target = msg.content.f32;
            break;
        SetKp: kp = msg.content.f32; break;
        SetKi: ki = msg.content.f32; break;
        SetKd: kd = msg.content.f32; break;
        default:
            break;
    }
}

void motor_set_pwm(int speed) {
	if (abs(speed) <= max_pwm) {
		if (speed > 0) {
			analogWrite(PWM_PIN_LEFT, speed);
			analogWrite(PWM_PIN_RIGHT, 0);
		} else {
			analogWrite(PWM_PIN_RIGHT, -speed);
			analogWrite(PWM_PIN_LEFT, 0);
		}
	}
}

void loop() {
	if (is_enabled) {
        enabled_loop();
	} else {
		motor_set_pwm(0);
	}
}

inline void enabled_loop() {
    if (is_homing) {
        homing_loop();
    } else {
        control_loop();
    }
}

inline void homing_loop() {
    if (digitalRead(LIMIT_SWC_PIN) == LOW) {
        is_homing = false;
        current_encoder = 0.0;
        target = current_encoder;
        encoder.write(target);
    } else {
        motor_set_pwm(homing_pwm);	
    }
}

inline void control_loop() {
    current_encoder = encoder.read();
    pid_control();
}

inline void pid_control() {
    if (abs(current_encoder - target) > ERROR_THRESHOLD) {
        if (pid.Compute()) {
            motor_set_pwm((int)pid_out);
        }
    } else {
        motor_set_pwm(0);
    }
}
