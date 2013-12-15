/// Modlab, University of Pennsylvania, June 2013
///
/// See header file for use notes.

#include "frame_queue.h"


void InitFQ(struct FrameQueue *fq, uint8_t* data, uint16_t data_size, uint16_t frame_size) {
    fq->data = data;
    fq->data_size = data_size;
    __builtin_memset(fq->data, 255, fq->data_size);             // unneccessary
    
    fq->start_frame = fq->data;
    fq->end_data = fq->data;
    
    fq->frame_size = frame_size;
    
    // make pointer to one past end of reserved space (the first invalid put)
    fq->end = fq->data + fq->frame_size * (fq->data_size / fq->frame_size);
}




int8_t IsEmptyFQ(struct FrameQueue *fq) {
    if(fq->start_frame == fq->end_data)
        return 1;
    else
        return 0;
}




void PeekFrameFQ(struct FrameQueue *fq, uint8_t** bytes, uint16_t* length) {

  *bytes = fq->start_frame;
  
  // if empty
  if(fq->start_frame == fq->end_data) {
    *length = 0;
  }
  // else not empty
  else {
    // if start_frame comes before end_data, buffer is not wrapped around
    if(fq->start_frame < fq->end_data) {
      *length = fq->end_data - fq->start_frame;
      if(*length > fq->frame_size)
        *length = fq->frame_size;
    }
    else {
      // else buffer wraps and therefore frame must be full
      *length = fq->frame_size;
    }
  }
}



void DropFrameFQ(struct FrameQueue *fq) {

  // if start_frame contains end_data, the active frame is being dropped
  if(fq->start_frame <= fq->end_data 
                        && fq->end_data < fq->start_frame + fq->frame_size) {
    fq->start_frame = fq->data;
    fq->end_data = fq->data;                        // buffer is now empty
  }
  // otherwise a full frame is being dropped
  else {
    fq->start_frame += fq->frame_size;              // advance pointer
    if(fq->start_frame + fq->frame_size > fq->end)  // pointer must wrap
      fq->start_frame = fq->data;
  }
}




int8_t PutBytesFQ(struct FrameQueue *fq, uint8_t* bytes, uint16_t length) {
  int8_t retval = -1; // failure, data loss
  
  // if end_data comes before start_frame, the buffer is wrapped around
  //   and the available storage is guarenteed continuous
  if(fq->end_data < fq->start_frame) {
    
    uint16_t space = fq->start_frame - fq->end_data - 1; // space available
    
    uint16_t copy_size = space;                          // calculate copy_size
    if(copy_size > length)
      copy_size = length;
    
    __builtin_memcpy(fq->end_data, bytes, copy_size);   // do copy
    fq->end_data += copy_size;
    
    if(copy_size == length)                             // check for data loss
      retval = 1; // success
  }
  
  // otherwise available storage might be broken into two pieces
  else {
    
    // if first data byte is occupied, last data byte must not be filled
    uint8_t* current_end = fq->end;
    if(fq->start_frame == fq->data)
      current_end--;

    // calculate amount of space near end and near beginning
    uint16_t end_space = current_end - fq->end_data;
    uint16_t start_space = fq->start_frame - fq->data - 1;

    // calculate first_half and second_half copy sizes (near end and at beginning)
    uint16_t first_half = end_space;
    if(first_half > length)
      first_half = length;
    uint16_t second_half = start_space;
    if(second_half > length - first_half)
      second_half = length - first_half;

    // copy first half, then second half
    __builtin_memcpy(fq->end_data, bytes, first_half);
    if(second_half == 0) {
      fq->end_data += first_half;
      if(fq->end_data >= fq->end)
        fq->end_data = fq->data;
    }
    else {
      __builtin_memcpy(fq->data, &(bytes[first_half]), second_half);
      fq->end_data = fq->data + second_half;
    }

    // check for data loss
    if(first_half + second_half == length)
      retval = 1; // success
  }
    
  return(retval);
}

