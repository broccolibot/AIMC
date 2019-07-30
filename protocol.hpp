#pragma once

#define MEMBERS() \
    MEMBER(Enable, 1) \
    MEMBER(SetTarget, 2) \
    MEMBER(Reset, 3) \
    MEMBER(ModePWM, 4) \
    MEMBER(ModePID, 5) \
    MEMBER(ModePneumatic, 6) \
    MEMBER(SetKp, 7) \
    MEMBER(SetKi, 8) \
    MEMBER(SetKd, 9) \
    MEMBER(Home, 10) \
    MEMBER(LimitPwm, 11) \
    MEMBER(LimitTargetMin, 12) \
    MEMBER(LimitTargetMax, 13) \
    MEMBER(EncoderPolarity, 14)

#define MEMBER(NAME, NUMBER) NAME = NUMBER,
enum Operation {
    MEMBERS()
};
#undef MEMBER

#define MEMBER(NAME, NUMBER) (char*)#NAME,
char* opcode_names[] = {
    (char*)"ZERO",
    MEMBERS()
    (char*)"MAX"
};
#undef MEMBER

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

void print_message(struct Message input);
