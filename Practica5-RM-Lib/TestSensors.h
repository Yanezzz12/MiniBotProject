#ifndef TestSensors
#define TestSensors

bool InfraredSensor(char sensor);
bool ContactSensor(char sensor);

void SensorSetup();
void TestEncoders();
void TestContact();
void TestInfrared();
void TestArrayLDR();

#endif
