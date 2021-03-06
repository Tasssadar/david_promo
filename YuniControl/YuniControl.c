#define DEVICE_ADDRESS 0x01
#define MASTER_ADDRESS 0xEF

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

bool checkForStart = true;
void StartMatch();
void sendPowerReq(uint8_t, uint8_t);

inline uint16_t fabs(int16_t num)
{
    if(num < 0)
        return -num;
    return num;
}
#include "yunimin3.h"

#define PING 1

enum states
{
    STATE_BUTTON      = 0x01,
    STATE_RECORD      = 0x02,
    STATE_PLAY        = 0x04,
    STATE_ERASING     = 0x08,
    STATE_CORRECTION2 = 0x10,
    STATE_COLLISION   = 0x20,
    STATE_PAUSED      = 0x40,
    STATE_FORCE_MOVE  = 0x80,
};

#define RANGE_THRESHOLD  45

volatile bool checkRange = true;
volatile uint8_t state = 0;
bool sendEmergency = false;
bool emergencySent = false;
#ifdef PING
 uint32_t lastTime = getTickCount();
 uint32_t thisTime = 0;
 const uint32_t PING_TIME ((1000000) * JUNIOR_WAIT_MUL / JUNIOR_WAIT_DIV);
 volatile uint32_t pingTimer = PING_TIME;
 uint32_t diff = 0;

 const uint32_t RANGE_TIME ((70000) * JUNIOR_WAIT_MUL / JUNIOR_WAIT_DIV);
 volatile uint32_t rangeTimer = RANGE_TIME;
 volatile bool checkRangeNow = false;
 uint8_t rangeTry = 0;

 bool doReel = false;
 uint32_t reelStopTimer = ((6000000) * JUNIOR_WAIT_MUL / JUNIOR_WAIT_DIV);

 const uint32_t MOVE_CHECK_TIME ((2000000) * JUNIOR_WAIT_MUL / JUNIOR_WAIT_DIV);
 volatile uint32_t moveCheckTimer = MOVE_CHECK_TIME;
#endif


#include "func.h"
#include "movement.h"
#include "packets.h"

void run()
{
    state = 0;
    setLeftServo(-422);
    setRightServo(690);
    clearLed();

    i2c.max_slave_rx(10);

    uint8_t poc = 0;
    while(true)
    {   
        if(!(state & STATE_COLLISION) && (moveflags == MOVE_FORWARD || moveflags == MOVE_BACKWARD))
            MovementCorrection();

        if(readPacket())
            handlePacket(&pkt);
#ifdef PING
        if(state & STATE_PAUSED)
            continue;

        //discButton((PINB & (1<<0)) == 0);

        thisTime = getTickCount();
        diff = (thisTime - lastTime);

        if(checkRangeNow && checkRange)
        {
            if(rangeTimer <= diff)
            {
                rangeTimer = RANGE_TIME;
                checkRangeNow = false;
                bool collision = checkRangeFunc();
                if(collision && !(state & STATE_COLLISION))
                {
                    state |= STATE_COLLISION;
                    setMotorPower(0, 0);
                    //clean_dc_motor();
                    Packet col(CMSG_RANGE_BLOCK);
                    sendPacket(&col);
                }
                else if(!collision && (state & STATE_COLLISION))
                {
                    if(rangeTry >= 50)
                    { 
                        state &= ~(STATE_COLLISION);
                        moveCheckTimer = MOVE_CHECK_TIME;
                        Packet col(CMSG_RANGE_BLOCK_GONE);
                        sendPacket(&col);
                        //init_dc_motor();
                        SetMovementByFlags();
                    }
                }
                if(!collision && (state & STATE_COLLISION));
                    ++rangeTry;
                if(collision)
                    rangeTry = 0;
            }
            else rangeTimer -= diff;
        }

        if(doReel)
        {
            if(reelStopTimer <= diff)
            {
               doReel = false;
               clearLed();
               reelStopTimer = ((1500000) * JUNIOR_WAIT_MUL / JUNIOR_WAIT_DIV);
            }
            else reelStopTimer -= diff;
        }

        if(moveflags != MOVE_NONE && !(state & STATE_COLLISION))
        {
            if(moveCheckTimer <= diff)
            {
                Packet dead(CMSG_DEADEND_DETECTED, 0);
                sendPacket(&dead);
                for(uint8_t y = 0; y < 5; ++y)
                {
                    if(enc_events[y].id == 0)
                        continue;
                    Packet encoder(CMSG_ENCODER_EVENT_DONE, 1);
                    encoder.m_data[0] = enc_events[y].id;
                    enc_events[y].id = 0;
                    sendPacket(&encoder);
                    moveflags = MOVE_NONE;
                    SetMovementByFlags();
                }
                moveCheckTimer = MOVE_CHECK_TIME;
            }else moveCheckTimer -= diff;
        }
   
        lastTime = thisTime;
#endif
    }
}
