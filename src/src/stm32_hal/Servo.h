#ifndef _SERVO_H__
#define _SERVO_H__

#include <stdint.h>

class Servo
{
public:
    Servo() {};

    void attach(int pin, uint16_t min=1000, uint16_t max=2000, uint32_t us=UINT32_MAX);
    void writeMicroseconds(uint32_t us);

private:
    uint32_t _channel;
    uint16_t _min;
    uint16_t _max;
};

#endif // _SERVO_H__
