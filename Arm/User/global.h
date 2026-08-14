#ifndef GLOBAL_H_
#define GLOBAL_H_

#define PACKET_HEADER_COUNT 2
#define PACKET_HEADER 0x55

typedef enum {
    ERR = -1,
    OK,
} Res;

typedef enum {
    NULL_POINTER = -1,
    FAILED_TO_INITIATE = -2,
} Err;

#endif