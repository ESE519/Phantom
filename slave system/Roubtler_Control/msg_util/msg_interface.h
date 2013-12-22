
#ifndef MSG_IFACE_H
#define MSG_IFACE_H

#include <stdint.h>
#include "Serial.h"
#include "crc_helper.h"
#include "packet_finder.h"
#include "byte_queue.h"

#define PF_INDEX_DATA_SIZE 20   // size of index buffer in packet_finder
#define OUT_QUEUE_DATA_SIZE 252 // size of out buffer in usb_interface

using namespace mbed;

class MessageIface { 

public:
    
    MessageIface( Serial *serial );
    int8_t getBytes( void );
    int8_t peekPacket( uint8_t **packet, uint8_t *length );
    int8_t dropPacket( void );
    int8_t sendPacket(uint8_t msg_type, uint8_t *data, uint8_t length);
    int8_t sendBytes(uint8_t *bytes, uint16_t length);
    void sendNow( void );

private:
    Serial* _serial;
    struct ByteQueue pf_index_queue;
    struct PacketFinder pf;      // packet_finder instance
    struct ByteQueue out_queue;  // frame_queue instance for outgoing bytes   
    uint8_t pf_index_data[PF_INDEX_DATA_SIZE];    // data for pf
    uint8_t out_queue_data[OUT_QUEUE_DATA_SIZE];  // data for out_queue 
    
}; 
    
#endif 