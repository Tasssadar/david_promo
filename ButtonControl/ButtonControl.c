#define DEVICE_ADDRESS 0x02
#define MASTER_ADDRESS 0xEF

#include "base.h"
#include "usart.h"

volatile uint8_t i2c_idx = 0;
#include "i2c.h"
#include "packets.h"

volatile uint8_t sendColor[3] = { 0 };
volatile uint8_t pressedButtons = 0;
volatile bool read = false;
bool led = false;
volatile int16_t ledCounter = 1000;

volatile int blink_time[10] = { 0 };

#include "buttons.h"

inline void init()
{
    init_rs232();
    init_buttons();
    init_i2c();
    DDRD |= (1<<PD6)|(1<<PD7);
    PORTD |= (1<<PD6);
}

inline void clean()
{
    clean_i2c();
    clean_rs232();
    clean_buttons();
}

static uint8_t i2c_slave_tx(void *)
{
    while(!read) nop();

    switch(i2c_idx++)
    {
        case 0:
            return pressedButtons;
        case 1:
            return sendColor[0];
        case 2:
            return sendColor[1];
        case 3:
            read = false;
            return sendColor[2];
        default:
            return 0xFF;
    }
}

uint8_t toInt(char hexChar)
{
    static const char hex[] = "0123456789ABCDEF";
    for(uint8_t i = 0; i < 16; ++i)
        if(hex[i] == hexChar)
            return i;
    return 0;
}

uint8_t result[3];
uint16_t cur_color = 0;
//uint16_t calib[3] = { 108, 98, 144 };

uint8_t color = 0; // R = 0, G = 1, B = 2
int8_t character = 2;

bool readColor()
{
    char ch;
    if(!rs232.peek(ch))
        return false;

    cur_color |= (toInt(ch) << (character*4));
    --character;

    if(character < 0)
    {
        character = 2;

        cur_color >>= 1;
        if(cur_color > 0xFF)
            result[color] = 0xFF;
        else
            result[color] = cur_color;
        cur_color = 0;

        if(++color == 3)
        {
            color = 0;
            if(!read)
                for(uint8_t i = 0; i < 3; ++i)
                    sendColor[i] = result[i];
            read = true;
        }
    }
    return true;
}
int main()
{
    sei();
    init();


    i2c.address(0x02);
    i2c.on_slave_tx(&i2c_slave_tx, 0);

    rs232.setMode(false);
    rs232.send("=  (00 m ) !");
    rs232.wait();
    rs232.setMode(true);

    while(true)
    {
        while(readColor());
/*
        if(led)
        {
            if(ledCounter <= 0)
            {
                for(uint8_t i = 0; i < 10; ++i)
                {
                    int cur_time = blink_time[i];
                    while(cur_time > 0)
                    {
                        PORTD |= (1<<PD7);
                        _delay_ms(500);
                        PORTD &= ~(1<<PD7);
                        _delay_ms(500);
                        --cur_time;
                    }
                    PORTD &= ~(1<<PD6);
                    _delay_ms(500);
                    PORTD |= (1<<PD6);
                    _delay_ms(1000);
                }
                led = false;
            }
            else
                --ledCounter;
        }
        _delay_ms(1);*/
    }
    
    cli();
    clean();
}




