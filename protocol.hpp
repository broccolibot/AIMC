#pragma once

enum Operation {
    Enable = 0,
    SetTarget = 1,
    Reset = 2,
    ModePWM = 3,
    ModePositionPID = 4,
    ModeVelocityPID = 5,
    SetKP = 6,
    SetKI = 7,
    SetKD = 8,
    Home = 9,
    LimitPWM = 10,
    LimitTargetMin = 11,
    LimitTargetMax = 12,
    SetPWM = 13,
    EncoderPolarity = 14,
    GetPosition = 15,
};

struct message {
    enum Operation operation;
    union {
        unsigned char bytes[4];
        unsigned int u32;
        int i32;
        float f32;
    } content;
};

struct message parse_message(unsigned char* bytes);

//void print_message(struct message input);
