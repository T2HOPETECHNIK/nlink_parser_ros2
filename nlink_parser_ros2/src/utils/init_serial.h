#ifndef INITSERIAL_H
#define INITSERIAL_H
#include <serial/serial.h>

bool initSerial(serial::Serial *serial, char *param_file_path);

#endif // INITSERIAL_H
