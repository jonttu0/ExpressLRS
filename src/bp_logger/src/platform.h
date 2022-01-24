#ifndef _PALTFORM_H_
#define _PALTFORM_H_

#include "platform_internal.h"

class CtrlSerial
{
public:
    virtual size_t available(void) = 0;
    virtual uint8_t read(void) = 0;

    void write(uint8_t data) {
        write(&data, 1);
    }
    virtual void write(uint8_t * buffer, size_t size) = 0;
};

extern CtrlSerial &ctrl_serial;

#endif /* _PALTFORM_H_ */
