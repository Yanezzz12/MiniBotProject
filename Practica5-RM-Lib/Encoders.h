#ifndef Encoders
#define Encoders

void EncoderSetup();
void LeftEncoderInterrupt();
void RightEncoderInterrupt();
volatile long LeftCount();
volatile long RightCount();

#endif
