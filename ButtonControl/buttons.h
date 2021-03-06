enum Buttons
{
    BUTTON_START     = 0x01,
    BUTTON_PAWN      = 0x02,
    BUTTON_FRONT     = 0x04,
    BUTTON_PAWS      = 0x08,
    BUTTON_BACK_LEFT = 0x10,
    BUTTON_BACK_RIGHT= 0x20,
};

inline void init_buttons()
{
    PORTC |= (1<<PC0);
    PORTB |= (1<<PB1);
    PORTD |= (1<<PD3);
    PCICR |= (1<<PCIE1)|(1<<PCIE0)|(1<<PCIE2);
    PCIFR |= (1<<PCIE1)|(1<<PCIE0)|(1<<PCIE2);
    PCMSK2 |= (1<<PCINT19);
    PCMSK1 |= (1<<PCINT8);
    PCMSK0 |= (1<<PCINT1);
}

inline void clean_buttons()
{
    PORTC = 0;
    PORTB = 0;
}

inline void ButtonPressed(uint8_t address, bool pressed)
{
    if((pressed && (pressedButtons & address)) || (!pressed && !(pressedButtons & address)))
        return;

    PORTD |= (1<<PD7);
    led = true;
    ledCounter = 100;

    if(pressed)
    {
        pressedButtons |= address;
        sendButtons |= address;
    }
    else
        pressedButtons &= ~(address);
}

ISR(PCINT0_vect)
{
    uint8_t status = (PINB & (1<<PB1));
    //_delay_ms(20);
    if(status != (PINB & (1<<PB1)))
        return;
    ButtonPressed(BUTTON_PAWS, !(PINB & (1<<PB1)));
}

ISR(PCINT1_vect)
{
    uint8_t status = (PINC & (1<<PC0));
    //_delay_ms(20);
    if(status != (PINC & (1<<PC0)))
        return;
    ButtonPressed(BUTTON_BACK_RIGHT, !(PINC & (1<<PC0)));
}

ISR(PCINT2_vect)
{
    uint8_t status = (PIND & (1<<PD3));
    //_delay_ms(20);
    if(status != (PIND & (1<<PD3)))
        return;
    ButtonPressed(BUTTON_FRONT, !(PIND & (1<<PD3)));
}
