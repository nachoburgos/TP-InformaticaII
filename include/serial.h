#ifndef serial_h
#define serial_h

#include <stdint.h>

void serialInit(void);
void serialWrite(uint8_t);
uint8_t serialRead(uint8_t *);


#endif // SERIAL_H



