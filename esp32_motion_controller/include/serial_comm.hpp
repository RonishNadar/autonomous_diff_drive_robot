// serial_comm.hpp
#ifndef SERIAL_COMM_HPP
#define SERIAL_COMM_HPP

void initSerial();
float getTargetLinearVelocity();
float getTargetAngularVelocity();
void sendOdometrySerial();
void processSerialInput();

#endif // SERIAL_COMM_HPP
