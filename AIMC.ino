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
// Limit switch input pin (Pullup)
#define LIMIT_SWC_PIN 12
#define LIMIT_SWC_CLICKED_STATE LOW
// Number of encoder counts to debounce at
#define ENCODER_ERROR_THRESHOLD 3
// How often PID should update
#define PID_INTERVAL 5
// Pin range for I2C ID (Note: The pins are in low-to-high order and are INPUT_PULLUP)
#define ID_PIN_MIN 7
#define ID_PIN_MAX 11

// Runtime variables
double current_encoder = 0.0;
double pid_out = 0.0;
double target = 0.0;
double kp = 0.0;
double ki = 0.0;
double kd = 0.0;
double target_min = -1000.0;
double target_max = 1000.0; 
int homing_pwm = 0;
int max_pwm = 255;
bool is_enabled = false;
bool is_homing = false;

// Control mode setting
enum ControlMode {
    ControlModePneumatic,
    ControlModePID,
    ControlModePWM,
} control_mode = ControlModePID;

// Runtime objects
PID pid(&current_encoder, &pid_out, &target, kp, ki, kd, DIRECT);
Encoder encoder(ENC_PIN_A, ENC_PIN_B);

int i2c_address_from_pins(int min_pin, int max_pin);

// Setup code
void setup() {
    // Set up I2C bus and ISRs
    Wire.begin(i2c_address_from_pins(ID_PIN_MIN, ID_PIN_MAX));
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
	pid.SetSampleTime(PID_INTERVAL);

    pinMode(LED_BUILTIN, OUTPUT);
    update_enabled_led();
}

// Digital read (with pullup) each pin in the range, and determine 
// the number they represent in binary 
int i2c_address_from_pins(int min_pin, int max_pin) {
    int i2c_id = 0;
    for (int pin = min_pin; pin <= max_pin; pin++) {
        pinMode(pin, INPUT_PULLUP);
        int pin_out = digitalRead(pin);
        i2c_id <<= 1;
        i2c_id |= pin_out;
    }
    return i2c_id;
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
    float limit_switch_float = digitalRead(LIMIT_SWC_PIN) == LIMIT_SWC_CLICKED_STATE;
    Wire.write((unsigned char*)&limit_switch_float, 4);
}

// Handle receiving a new I2C message
void on_receive(int n_bytes) {
    if (n_bytes != 5) {
        // Clear buffer before returning,
        // will prevent the bus from blocking
        while(Wire.available()) Wire.read();
        return;
    }
    char buffer[5] = {0};
    char* buffer_ptr = buffer;

    while(Wire.available()) 
        *buffer_ptr++ = Wire.read();

    Message msg = parse_message(buffer);
    handle_message(msg);
}

// Incoming message handler
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
            //This code MUST be in a block, or the arduino compiler will output erroneous code and
            //will NOT warn you! Any sane compiler _would_ warn you, but that's a fantasy. 
            { 
                float target_proposal = msg.content.f32;
                if (target_proposal >= target_min && target_proposal <= target_max) {
                    target = target_proposal;
                }
            }
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
        case LimitTargetMin:
            target_min = msg.content.f32;
            break;
        case LimitTargetMax:
            target_max = msg.content.f32;
            break;
        case ModePneumatic :
            control_mode = ControlModePneumatic;
            break;
        case ModePID:
            control_mode = ControlModePID;
            break;
        case ModePWM:
            control_mode = ControlModePWM;
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
            digitalWrite(PWM_PIN_RIGHT, LOW);
		} else {
            digitalWrite(PWM_PIN_LEFT, LOW);
			analogWrite(PWM_PIN_RIGHT, -speed);
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
void enabled_loop() {
    if (is_homing) {
        homing_loop();
    } else {
        control_loop();
    }
}

// Homing loop
void homing_loop() {
    if (digitalRead(LIMIT_SWC_PIN) == LIMIT_SWC_CLICKED_STATE) {
        is_homing = false;
        current_encoder = 0.0;
        target = current_encoder;
        encoder.write(current_encoder);
    } else {
        motor_set_pwm(homing_pwm);	
    }
}

// Motor control main loop, used for dispatching other loops 
void control_loop() {
    switch (control_mode) {
        case ControlModePID:
            pid_control();
            break;
        case ControlModePneumatic:
            pneumatic_control();
            break;
        case ControlModePWM:
            pwm_control();
            break;
        default:
            break;
    }
}

// Pneumatic control main loop
void pneumatic_control() {
    if (target > 0) {
        digitalWrite(PWM_PIN_LEFT, HIGH);
        digitalWrite(PWM_PIN_RIGHT, LOW);
    } else {
        digitalWrite(PWM_PIN_LEFT, LOW);
        digitalWrite(PWM_PIN_RIGHT, HIGH);
    }
}

// PWM control main loop
void pwm_control() {
    motor_set_pwm((int)target);
}

// PID control main loop
void pid_control() {
    if (abs(current_encoder - target) > ENCODER_ERROR_THRESHOLD) {
        if (pid.Compute()) {
            motor_set_pwm((int)pid_out);
        }
    } else {
        motor_set_pwm(0);
    }
}
