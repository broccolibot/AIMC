#include "protocol.hpp"

Message parse_message(unsigned char* bytes) {
    Message result;
    result.opcode = (enum Operation)bytes[0];
    memcpy(result.content.bytes, &bytes[1], sizeof result.content);
    return result;
}
