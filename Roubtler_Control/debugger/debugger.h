#ifndef DEBUGGER_H
#define DEBUGGER_H

/*
    Debugger: a class for enabling non-blocking debug printf capabilities
              as well as packet_util compliant debug printing
*/

#include "msg_interface.h"
#include "message_definitions.h"
#include "Serial.h"

using namespace mbed;

class Debugger {
public:
    Debugger( MessageIface* iface );      
    void setEnable( bool enable );
    int8_t printf( char* fmt, ... );
    int8_t sendPacket( uint8_t msg_type, uint8_t *data, uint8_t length );
    
private:
    bool _enabled;
    MessageIface* _iface;
    char _debug_str[MAX_DEBUG_LEN];
};

#endif
    
