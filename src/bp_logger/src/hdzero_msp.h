#pragma once

#include "platform.h"
#include "msp.h"
#include <stdint.h>


class HDZeroMsp
{
public:
    HDZeroMsp(CtrlSerial *serial) : _serial(serial) {}
    ~HDZeroMsp() {}

    void init(void);

    void syncSettings(int num);

    int parse_data(uint8_t const chr);
    int parse_command(char * cmd, size_t len, int num);

    int handle_received_msp(mspPacket_t &msp_in);

    void loop(void);

private:
    CtrlSerial * _serial;

    MSP _handler;
    mspPacket_t msp_out;

    void handleUserText(const char * input, size_t len);
    void handleVtxFrequency(const char * input, int num = -1);
};
