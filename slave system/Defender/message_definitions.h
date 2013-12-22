#ifndef MESSAGE_DEFINITIONS_H
#define MESSAGE_DEFINITIONS_H


// An example of a message definition, this one only has a parameter to set the value of an led
/*const uint8_t MsgTypeDebug = 0x01;
typedef struct __attribute__ ((__packed__)) {
    uint8_t led_on;
} MsgDebug;
*/
/* Old Not Used!!!
const uint8_t MsgTypeDebug = 0x01;
typedef struct __attribute__ ((__packed__)) {
    uint8_t led_on; 
} MsgDebug;
*/

const uint8_t MsgTypeMode = 0x09; 
typedef struct __attribute__ ((__packed__)){
    uint8_t mode_recv;
} MsgMode;
    

// Debugger Messages

#define MAX_DEBUG_LEN 100
const uint8_t MsgTypeDebug = 99;
// A description of what the message looks like, the actual implementation doesn't use this. 
typedef struct __attribute__ ((__packed__)) {
    char debug_string;
} MsgDebug;  

// A message for the debug enable 
const uint8_t MsgTypeDebugEnable = 100;
typedef struct __attribute__ ((__packed__)) {
    uint8_t enable_debugging;
} MsgDebugEnable;

#endif