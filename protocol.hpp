#pragma once

enum Operation {
    Enable = 1,
    SetTarget = 2,
    Reset = 3,
    ModePWM = 4,
    ModePID = 5,
    ModePneumatic = 6,
    SetKp = 7,
    SetKi = 8,
    SetKd = 9,
    Home = 10,
    LimitPwm = 11,
    LimitTargetMin = 12,
    LimitTargetMax = 13,
    EncoderPolarity = 14,
};

struct Message {
    enum Operation opcode;
    union Content {
        unsigned char bytes[4];
        unsigned int u32;
        int i32;
        float f32;
    } content;
};

struct Message parse_message(unsigned char* bytes);
