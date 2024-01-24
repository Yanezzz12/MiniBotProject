#ifndef Encoders
#define Encoders

void SetEncodersInput();
void LeftEncoderInterrupt();
void RightEncoderInterrupt();
volatile long LeftCount();
volatile long RightCount();

#endif
