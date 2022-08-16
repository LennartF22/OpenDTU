/* frameHandler.h
 *
 * Arduino library to read from Victron devices using VE.Direct protocol.
 * Derived from Victron framehandler reference implementation.
 * 
 * 2020.05.05 - 0.2 - initial release
 * 2021.02.23 - 0.3 - change frameLen to 22 per VE.Direct Protocol version 3.30
 * 
 */
#pragma once

#include <Arduino.h>

const byte frameLen = 22;                       // VE.Direct Protocol: max frame size is 18
const byte nameLen = 9;                         // VE.Direct Protocol: max name size is 9 including /0
const byte valueLen = 33;                       // VE.Direct Protocol: max value size is 33 including /0
const byte buffLen = 40;                        // Maximum number of lines possible from the device. Current protocol shows this to be the BMV700 at 33 lines.

#ifndef VICTRON_PIN_TX
#define VICTRON_PIN_TX 21
#endif

#ifndef VICTRON_PIN_RX
#define VICTRON_PIN_RX 22
#endif

class VeDirectFrameHandler {

public:

    VeDirectFrameHandler();
    void init();
    void setPollInterval(uint32_t interval);
    void loop();
    uint32_t getLastUpdate();
    void setLastUpdate();
    String getPidAsString(const char* pid);
    String getCsAsString(const char* pid);
    String getErrAsString(const char* err);
    String getOrAsString(const char* offReason);
    String getMpptAsString(const char* mppt);

    char veName[buffLen][nameLen] = { };        // public buffer for received names
    char veValue[buffLen][valueLen] = { };      // public buffer for received values

    int frameIndex;                             // which line of the frame are we on
    int veEnd;                                  // current size (end) of the public buffer

private:
    //bool mStop;                               // not sure what Victron uses this for, not using

    enum States {                               // state machine
        IDLE,
        RECORD_BEGIN,
        RECORD_NAME,
        RECORD_VALUE,
        CHECKSUM,
        RECORD_HEX
    };

    int mState;                                 // current state

    uint8_t	mChecksum;                          // checksum value

    char * mTextPointer;                        // pointer to the private buffer we're writing to, name or value

    char mName[9];                              // buffer for the field name
    char mValue[33];                            // buffer for the field value
    char tempName[frameLen][nameLen];           // private buffer for received names
    char tempValue[frameLen][valueLen];         // private buffer for received values

    void rxData(uint8_t inbyte);                // byte of serial data to be passed by the application
    void textRxEvent(char *, char *);
    void frameEndEvent(bool);
    void logE(const char *, const char *);
    bool hexRxEvent(uint8_t);
    uint32_t _lastUpdate = 0;
    uint32_t _pollInterval;
    uint32_t _lastPoll = 0;
};

extern VeDirectFrameHandler VeDirect;
