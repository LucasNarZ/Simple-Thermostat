#ifndef SENSORS_H
#define SENSORS_H

#define NTC_PIN 34
// #define LDR_PIN 36
#define FLOOR_SENSOR_PIN 39

void initSensors();
bool hasFloorSensor();
// int getLDRvalue();
float getSensorTemperature(bool sensorPiso);

#endif