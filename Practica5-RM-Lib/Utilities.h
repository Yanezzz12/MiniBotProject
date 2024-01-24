#ifndef Utilities
#define Utilities

float ErrorToPWM(long maxError, float currentError);
float AngleCorrection(short angle);
long AngleToTicks(float angle);
long DistanceToTicks(float distance);
short Map(short value);

#endif
