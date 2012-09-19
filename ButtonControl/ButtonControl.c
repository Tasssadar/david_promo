#define DEVICE_ADDRESS 0x02
#define MASTER_ADDRESS 0xEF

#include "base.h"
#include "usart.h"
#include "i2c.h"
#include "packets.h"
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

int main()
{
    sei();
    init();
    
    i2c.on_slave_tx(&i2c_slave_tx, 0);
    
    while(true)
    {
        
        if(led)
        {
            if(ledCounter >= 10)
                PORTD &= ~(1<<PD7);
            else
                ++ledCounter;
        }
        _delay_ms(10);
    }
    
    cli();
    clean();
}


