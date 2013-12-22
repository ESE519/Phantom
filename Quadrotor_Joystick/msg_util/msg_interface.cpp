#include "msg_interface.h"

/*
#include "crc_helper.h"
#include "packet_finder.h"
#include "byte_queue.h"
*/

//Serial pc1(USBTX,USBRX);

MessageIface::MessageIface( Serial *serial ) {
    _serial = serial;
    //InitPacketFinder(&pf, pf_index_data, PF_INDEX_DATA_SIZE);
    InitBQ(&pf_index_queue, pf_index_data, PF_INDEX_DATA_SIZE);
    InitPacketFinder(&pf, &pf_index_queue);
    InitBQ(&out_queue, out_queue_data, OUT_QUEUE_DATA_SIZE);
}

int8_t MessageIface::getBytes( void ) {
    uint8_t byte;
    int8_t status;
    while ( _serial->readable() ) {
        byte = _serial->getc();
        //pc1.putc(byte);
        status = PutBytes(&pf, &byte, 1);
    }
    return( status );    
}

int8_t MessageIface::peekPacket( uint8_t **packet, uint8_t *length ) {
    return(PeekPacket(&pf, packet, length));
}

int8_t MessageIface::dropPacket( void ) {
    return(DropPacket(&pf));
}

int8_t MessageIface::sendPacket( uint8_t msg_type, uint8_t *data, uint8_t length ) {
    uint8_t header[3];
    header[0] = kStartByte;                   // const defined by packet_finder.c
    header[1] = length;
    header[2] = msg_type;
    sendBytes(header, 3);
    
    sendBytes(data, length);
    
    uint8_t footer[2];
    uint16_t crc;
    crc = MakeCrc(&(header[1]), 2);
    crc = ArrayUpdateCrc(crc, data, length);
    footer[0] = crc & 0x00FF;
    footer[1] = crc >> 8;
    sendBytes(footer, 2);
    
    return(1);
}

int8_t MessageIface::sendBytes( uint8_t *bytes, uint16_t length) {
    uint16_t i;
    for (i=0; i<length; i++) {
        PutByteBQ(&out_queue, bytes[i]);
    }
    return(1);
}

void MessageIface::sendNow( void ) {
    uint8_t byte;
    while( _serial->writeable() ) {
        if( IsEmptyBQ(&out_queue) ) {
            break;
        }
        byte = GetByteBQ( &out_queue );
        _serial->putc( byte );
    }   
}

  
   
 