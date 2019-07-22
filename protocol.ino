#include "protocol.hpp"

struct message parse_message(unsigned char* bytes) {
    struct message result;
    result.operation = (enum Operation)bytes[0];
    memcpy(result.content.bytes, &bytes[1], sizeof result.content);
    return result;
}
