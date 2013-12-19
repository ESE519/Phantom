#include "packet_finder.h"
#include "crc_helper.h"

//#define DEBUG
#ifdef DEBUG
  #include <stdio.h>
  void PrintFromBuffer(struct PacketFinder *pf, uint16_t index, uint16_t length);
  #define DEBUG_PRINT_TXT(arg) printf(arg);
#else
  #define DEBUG_PRINT_TXT(arg)
#endif



const uint8_t kMaxPacketSize = MAX_PACKET_SIZE;          /// max packet size
const uint8_t kMaxPacketDataSize = MAX_PACKET_DATA_SIZE; /// max size data
const uint8_t kStartByte = 0x55;                         /// special start byte


#define SUCCESS         (0)
#define BUFFER_OVERFLOW (-1)
#define FATAL_ERROR     (-2)


void InitPacketFinder(struct PacketFinder *pf, struct ByteQueue *bq) {

  // parser state machine
  pf->state = kStart;
  pf->parse_index = 0;
  pf->packet_start_index = 0; // index of length byte in the buffer, type and data follow
  pf->received_length = 0;
  pf->packet_indices = bq;
  
  // circular buffer in storage array 'buffer', size 'PACKET_BUFFER_SIZE'
  pf->start_data = pf->buffer;   // pointer to first data byte
  pf->end_data = pf->buffer;     // pointer to byte following last data byte
}


int8_t PutBytes(struct PacketFinder *pf, uint8_t *bytes, uint8_t bytes_length) {
   
  ///////////////////////////////////////////////////
  // copy new data to buffer
  
  int8_t buffer_status = BUFFER_OVERFLOW; // put failure, data loss
  
  // if end_data comes before start_data, the buffer is wrapped around
  //   and the available storage is guarenteed continuous
  if(pf->end_data < pf->start_data) {
    uint16_t space = pf->start_data - pf->end_data - 1; // space available
    
    uint16_t copy_size = space;                         // calculate copy_size
    if(copy_size > bytes_length)
      copy_size = bytes_length;
    
    __builtin_memcpy(pf->end_data, bytes, copy_size);   // do copy
    pf->end_data += copy_size;
    
    if(copy_size == bytes_length)                       // check for data loss
      buffer_status = SUCCESS; // success
  }
  
  // otherwise available storage might be broken into two pieces
  else {
    // if first data byte is occupied, last data byte must not be filled
    uint8_t* current_end = pf->buffer + PACKET_BUFFER_SIZE;
    if(pf->start_data == pf->buffer)
      current_end--;

    // calculate amount of space near end and near beginning
    uint16_t start_space, end_space;
    end_space = current_end - pf->end_data;
    if(pf->start_data == pf->buffer)
      start_space = 0;
    else
      start_space = pf->start_data - pf->buffer - 1;

    // calculate first_half and second_half copy sizes (near end and at beginning)
    uint16_t first_half = end_space;
    if(first_half > bytes_length)
      first_half = bytes_length;
    uint16_t second_half = start_space;
    if(second_half > bytes_length - first_half)
      second_half = bytes_length - first_half;

    // copy first half, then second half
    __builtin_memcpy(pf->end_data, bytes, first_half);
    if(second_half == 0) {
      pf->end_data += first_half;
      if(pf->end_data > &(pf->buffer[PACKET_BUFFER_SIZE-1]))
        pf->end_data = pf->buffer;
    }
    else {
      __builtin_memcpy(pf->buffer, &(bytes[first_half]), second_half);
      pf->end_data = pf->buffer + second_half;
    }

    // check for data loss
    if(first_half + second_half == bytes_length)
      buffer_status = SUCCESS; // success
  }
   
   
  ///////////////////////////////////////////////////
  // run parser to exhaustion
  
  uint16_t end_data_index = pf->end_data - pf->buffer;
  while( pf->parse_index != end_data_index) {      // parser incrementing
  
    switch(pf->state) {

      case kStart:
        if(pf->buffer[pf->parse_index] == kStartByte)
          pf->state = kLen;
        pf->parse_index++;
        if(pf->parse_index >= PACKET_BUFFER_SIZE)
          pf->parse_index = pf->parse_index - PACKET_BUFFER_SIZE;
        break;

      case kLen:
        if(pf->buffer[pf->parse_index] <= kMaxPacketDataSize) {
          pf->received_length = pf->buffer[pf->parse_index];
          pf->expected_crc = MakeCrc(&pf->buffer[pf->parse_index], 1);
          pf->packet_start_index = pf->parse_index;
          pf->state = kType;
          pf->parse_index++;
          if(pf->parse_index >= PACKET_BUFFER_SIZE)
            pf->parse_index = pf->parse_index - PACKET_BUFFER_SIZE;
        }
        else {
          pf->state = kStart;
        }
        break;

      case kType:
        pf->expected_crc = ByteUpdateCrc(pf->expected_crc, pf->buffer[pf->parse_index]);
        if(pf->received_length > 0)
          pf->state = kData;
        else
          pf->state = kCRCL;
        pf->data_bytes = 1;
        pf->parse_index++;
        if(pf->parse_index >= PACKET_BUFFER_SIZE)
          pf->parse_index = pf->parse_index - PACKET_BUFFER_SIZE;
        break;

      case kData:
        pf->expected_crc = ByteUpdateCrc(pf->expected_crc, pf->buffer[pf->parse_index]);
        pf->data_bytes++;
        if(pf->data_bytes >= pf->received_length+1)
          pf->state = kCRCL;
        pf->parse_index++;
        if(pf->parse_index >= PACKET_BUFFER_SIZE)
          pf->parse_index = pf->parse_index - PACKET_BUFFER_SIZE;
        break;

      case kCRCL:
        pf->received_crc = pf->buffer[pf->parse_index];
        pf->state = kCRCH;
        pf->parse_index++;
        if(pf->parse_index >= PACKET_BUFFER_SIZE)
          pf->parse_index = pf->parse_index - PACKET_BUFFER_SIZE;
        break;

      case kCRCH:
        pf->received_crc = pf->received_crc + 256*pf->buffer[pf->parse_index];
        if(pf->expected_crc == pf->received_crc) {            // pass CRC
          PutByteBQ(pf->packet_indices, pf->packet_start_index);

          #ifdef DEBUG
          printf("Found: ");
          PrintFromBuffer(pf, pf->packet_start_index+1, pf->received_length+1);
          printf("\n");
          #endif

          pf->parse_index++;
          if(pf->parse_index >= PACKET_BUFFER_SIZE)
            pf->parse_index = pf->parse_index - PACKET_BUFFER_SIZE;
        }
        else {                                                // fail CRC
          pf->parse_index = pf->packet_start_index;
        }
        pf->state = kStart;
        break;

      default:        // a serious error occured, the algorithm must have a bug
        return(FATAL_ERROR);
    }
  }

  return(buffer_status);
}


int8_t PeekPacket(struct PacketFinder *pf, uint8_t **packet, uint8_t *length) {
    
  if(!IsEmptyBQ(pf->packet_indices)) {

    // get return index and length
    uint16_t start_index = PeekByteBQ(pf->packet_indices);
    uint16_t return_index = start_index + 1;
    if(return_index >= PACKET_BUFFER_SIZE)
      return_index = 0;
    *length = pf->buffer[start_index] + 1;
    
    #ifdef DEBUG
    printf("Peek: ");
    PrintFromBuffer(pf, return_index, *length);
    printf("\n");
    #endif

    // if packet is contiguous, return direct pointer into buffer
    if(return_index + *length < PACKET_BUFFER_SIZE) {
      *packet = &(pf->buffer[return_index]);
    }
    
    // else copy to out_buffer and return pointer to that
    else {
      uint8_t length_at_end = PACKET_BUFFER_SIZE - return_index;

      __builtin_memcpy( &(pf->out_buffer[0]),
                        &(pf->buffer[return_index]),
                        length_at_end);                                         // overflow out would cause a segfault
                        
      
      __builtin_memcpy( &(pf->out_buffer[length_at_end]),
                        pf->buffer,
                        *length - length_at_end);
                        
      *packet = pf->out_buffer;
    }
    return(1);
  }
  return(0);
}


int8_t DropPacket(struct PacketFinder *pf) {

  if(!IsEmptyBQ(pf->packet_indices)) {

    // get previous return index and length
    uint16_t prev_start_index = GetByteBQ(pf->packet_indices);
    uint16_t prev_return_index = prev_start_index + 1;
    if(prev_return_index >= PACKET_BUFFER_SIZE)
      prev_return_index = 0;
    uint8_t prev_length = pf->buffer[prev_start_index] + 1;

    // if still more packets, dump up to next packet start
    if(!IsEmptyBQ(pf->packet_indices)) {
      pf->start_data = &(pf->buffer[PeekByteBQ(pf->packet_indices)]);
    }
    // otherwise dump only used up bytes
    else {

      // if packet was contiguous
      if(prev_return_index + prev_length <= PACKET_BUFFER_SIZE) {

        uint16_t new_start_index = prev_return_index + prev_length;
        if(new_start_index >= PACKET_BUFFER_SIZE)
          new_start_index = 0;
        pf->start_data = &(pf->buffer[new_start_index]);
      }
      
      // not contiguous
      else {
        uint8_t length_at_end = PACKET_BUFFER_SIZE - prev_return_index;
        pf->start_data = &(pf->buffer[prev_length - length_at_end]);
      }
    }

    return(1);
  }
  return(0);
}


int8_t GetPacketCopy(struct PacketFinder *pf, uint8_t *packet_copy, uint8_t *length) {

  uint8_t *packet_original;
  int8_t status = PeekPacket(pf, &packet_original, length);
  if(1 == status) {
    __builtin_memcpy( packet_copy, packet_original, *length);
    DropPacket(pf);
    return(1);
  }
  return(0);
}


int8_t FormPacket(uint8_t type,
                  uint8_t *in_data, uint8_t in_len,
                  uint8_t *out_data, uint8_t *out_len) {

  out_data[0] = kStartByte;
  out_data[1] = in_len;
  out_data[2] = type;

  //__builtin_memcpy( &(out_data[i+3]), in_data, in_len); // untested, faster alternative
  uint8_t i = 0;
  for(i=0; i<in_len; i++)
    out_data[i+3] = in_data[i];

  uint16_t crc = MakeCrc(&(out_data[1]), in_len+2);

  out_data[in_len+3] = crc & 0x00FF;
  out_data[in_len+4] = crc >> 8;

  *out_len = in_len+5;
  return(1);
}


#ifdef DEBUG
void PrintFromBuffer(struct PacketFinder *pf, uint16_t index, uint16_t length) {
  printf("[ ");
  uint8_t i = 0;
  for(i=0; i<length; i++) {
    printf("%i ", pf->buffer[index]);
    index++;
    if(index >= PACKET_BUFFER_SIZE)
      index = 0;
  }
  printf("]");
}
#endif

