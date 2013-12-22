/// Modlab, University of Pennsylvania, June 2013
///
/// A FIFO N-buffer (double-buffer, quintiple-buffer, ect) of N frames each 
/// of M bytes. A FrameQueue struct maintains the data and state for one 
/// instance, and many simultaneous instances are supported. An operation on 
/// one must not be interrupted by another operation on that same instance 
/// (possibly via an interrupt). This restriction is not enforced and no error 
/// code is generated.

#ifndef FRAME_QUEUE_H
#define    FRAME_QUEUE_H

#include <stdint.h>

/// FrameQueue instance state struct, voluntarily opaque.
struct FrameQueue {
    uint8_t* start_frame; // points to start of first frame
    uint8_t* end_data;    // points to byte past last data byte
    uint16_t data_size;   // total allocated bytes
    uint16_t frame_size;  // number bytes in a frame
    uint8_t* data;        // pointer to array allocated for data
    
    uint8_t* end;         // pointer to byte past last allowed start_frame
                          // calculated from data_size and frame_size during init
};


/// initialize buffer as empty
void InitFQ(struct FrameQueue *fq, uint8_t* data, uint16_t data_size, uint16_t frame_size);

/// return 1 if buffer empty, 0 else
int8_t IsEmptyFQ(struct FrameQueue *fq);

/// peek next frame from buffer (frame is not removed)
/// returns point to up to num_bytes bytes of data
void PeekFrameFQ(struct FrameQueue *fq, uint8_t** bytes, uint16_t* length);

/// drop frame from buffer
void DropFrameFQ(struct FrameQueue *fq);

/// add bytes to buffer
/// return 1 for success, 0 for failure
int8_t PutBytesFQ(struct FrameQueue *fq, uint8_t* bytes, uint16_t length);

#endif // FRAME_QUEUE_H

