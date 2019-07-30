#include "protocol.hpp"

Message parse_message(unsigned char* bytes) {
    Message result;
    result.opcode = (enum Operation)bytes[0];
    memcpy(result.content.bytes, &bytes[1], sizeof result.content);
    return result;
}

void print_message(Message message) {
    Serial.print("Opcode: ");
    Serial.print(opcode_names[message.opcode]);
    Serial.print(" (");
    Serial.print(message.opcode);
    Serial.print(") f");
    Serial.print(message.content.f32);
    Serial.print(" i");
    Serial.print(message.content.i32);
    Serial.print(" u");
    Serial.println(message.content.u32);
}
