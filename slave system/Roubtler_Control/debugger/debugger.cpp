#include "debugger.h"
#include <stdarg.h>

Debugger::Debugger( MessageIface* iface ) {
    _iface = iface;
    _enabled = false; 
}

void Debugger::setEnable( bool enable ) {
    _enabled = enable;
}

int8_t Debugger::printf( char* fmt, ... ) {
    // If debugging is not enabled, then don't send print messages
    if ( !_enabled ) {
        return 0;
    }

    uint8_t len;
    va_list args;
    va_start( args, fmt );
    vsprintf( _debug_str, fmt, args );
    va_end( args );
    
    len = strlen(_debug_str);
    if ( len > MAX_DEBUG_LEN ) {
        return 0;
    }
    
    _iface->sendPacket( MsgTypeDebug, (uint8_t*)_debug_str, len );        
    return 1; 
}

int8_t Debugger::sendPacket( uint8_t msg_type, uint8_t *data, uint8_t length ) {
    // If debugging is not enabled, then don't send these packets
    if ( !_enabled ) {
        return 0;
    }
    _iface->sendPacket( msg_type, data, length);
    return 1;
}
    