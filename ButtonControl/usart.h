#ifndef JUNIOR_RS232_BPS
#define JUNIOR_RS232_BPS 4800
#endif

#ifndef JUNIOR_RS232_SIGNAL
#define JUNIOR_RS232_SIGNAL USART
#endif

#ifndef JUNIOR_RS232_TXBUF
# define JUNIOR_RS232_TXBUF 96
#endif

#ifndef JUNIOR_RS232_RXBUF
# define JUNIOR_RS232_RXBUF 255
#endif

#ifndef JUNIOR_RS232_BPS
# error Nastavte symbol JUNIOR_RS232_BPS na rychlost, s jakou chcete komunikovat (napr. 115200).
#endif

inline void force_wd_reset()
{
    cli();
#if defined(WDTCR)
    WDTCR = (1<<WDCE)|(1<<WDE);
    WDTCR = (1<<WDE);
#elif defined(WDTCSR)
    WDTCSR = (1<<WDCE)|(1<<WDE);
    WDTCSR = (1<<WDE);
#else
# error Unsupported Watchdog timer interface.
#endif
    for (;;)
    {
    }
}

#ifndef JUNIOR_H_QUEUE_H
#define JUNIOR_H_QUEUE_H

namespace detail {

template <typename T, uint8_t size>
class queue
{
public:
    queue()
        : m_rd_ptr(0), m_wr_ptr(0)
    {
    }

    bool push(T t)
    {
        uint8_t wr = m_wr_ptr;

        uint8_t new_ptr = inc(wr);

        if (new_ptr == m_rd_ptr)
            return false;

        m_elems[wr] = t;
        m_wr_ptr = new_ptr;
        return true;
    }

    bool empty() const
    {
        return m_rd_ptr == m_wr_ptr;
    }

    bool full() const
    {
        return inc(m_wr_ptr) == m_rd_ptr;
    }

    T top() const
    {
        return m_elems[m_rd_ptr];
    }

    void pop()
    {
        m_rd_ptr = inc(m_rd_ptr);
    }

private:
    T m_elems[size];
    volatile uint8_t m_rd_ptr;
    volatile uint8_t m_wr_ptr;

    uint8_t inc(uint8_t v) const
    {
        ++v;
        return v == size? 0: v;
    }
};

}

#endif


namespace detail {

template <int rxmax, int txmax>
class rs232_t
{
public:
    void data_in(char ch)
    {
        m_rxbuf.push(ch);
    }

    bool data_out(char & ch)
    {
        if (m_txbuf.empty())
            return false;

        ch = m_txbuf.top();
        m_txbuf.pop();
        return true;
    }

    bool peek(char & ch)
    {
        if (m_rxbuf.empty())
            return false;

        ch = m_rxbuf.top();
        m_rxbuf.pop();
        return true;
    }

    void sendCharacter(char ch)
    {
        m_txbuf.push(ch);
        UCSR0B |= (1<<UDRIE0);
    }

    void send(const char * str)
    {
        for (; *str != 0; ++str)
            m_txbuf.push(*str);
        UCSR0B |= (1<<UDRIE0);
    }

    void sendHexByte(uint8_t byte)
    {
        static const char hexdigits[] = "0123456789ABCDEF";
        this->sendCharacter(hexdigits[byte >> 4]);
        this->sendCharacter(hexdigits[byte & 0x0f]);
    }

    void sendBytes(uint8_t data[], uint8_t lenght)
    {
        for(uint8_t i = 0; i < lenght; ++i)
            sendCharacter(char(data[i]));
    }

    template <typename T>
    void sendNumberHex(T n, uint8_t width = 0)
    {
        char buf[32];
        uint8_t len = 0;

        if (n != 0)
        {
            T a = (n < 0)? -n: n;

            while (a > 0)
            {
                buf[len++] = '0' + (a & 0x0f);
                a = a >> 4;
            }

            if (n < 0)
                buf[len++] = '-';
        }
        else
            buf[len++] = '0';

        for (; width > len; --width)
            m_txbuf.push(' ');
        for (; len > 0; --len)
            m_txbuf.push(buf[len-1]);
        UCSR0B |= (1<<UDRIE0);
    }

    template <typename T>
    void sendNumber(T n, uint8_t width = 0)
    {
        char buf[32];
        uint8_t len = 0;

        if (n != 0)
        {
            T a = (n < 0)? -n: n;

            while (a > 0)
            {
                T b = a / 10;
                buf[len++] = '0' + (a - b * 10);
                a = b;
            }

            if (n < 0)
                buf[len++] = '-';
        }
        else
            buf[len++] = '0';

        for (; width > len; --width)
            m_txbuf.push(' ');
        for (; len > 0; --len)
            m_txbuf.push(buf[len-1]);
        UCSR0B |= (1<<UDRIE0);
    }

    template <typename T>
    void dumpNumber(T n)
    {
        sendNumber(n);
        sendCharacter(' ');
    }

    void wait()
    {
        while (!m_txbuf.empty())
        {
            JUNIOR_DO_IDLE();
        }
    }

    bool txempty() const
    {
        return m_txbuf.empty();
    }

    void setMode(bool receiver)
    {
        if(receiver)
        {
            UCSR0B &= ~(1<<TXEN0);
            UCSR0B |= (1<<RXEN0);
        }
        else
        {
            UCSR0B &= ~(1<<RXEN0);
            UCSR0B |= (1<<TXEN0);
        }
    }

private:
    detail::queue<char, rxmax> m_rxbuf;
    detail::queue<char, txmax> m_txbuf;
};

}

detail::rs232_t<JUNIOR_RS232_RXBUF, JUNIOR_RS232_TXBUF> rs232;

inline void init_rs232()
{
    // Initialize the RS232 line
    UCSR0A = (1<<U2X0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);

#define JUNIOR_UBRR ((F_CPU + 4 * JUNIOR_RS232_BPS) / (8 * JUNIOR_RS232_BPS) - 1)
    
    UBRR0H = 0;//JUNIOR_UBRR >> 8;
    UBRR0L = 207; //JUNIOR_UBRR;
    UCSR0B = (1<<RXCIE0);
}

inline void clean_rs232()
{
    UCSR0B = 0;
    UCSR0A = 0;
    UCSR0C = 0;
}

ISR(JUNIOR_CONCAT(JUNIOR_RS232_SIGNAL, _RX_vect))
{
    unsigned char ch = UDR0;

    {
        static const unsigned char bootSeq[] = { 0x74, 0x7E, 0x7A, 0x33 };
        static uint8_t state = 0;

        if (ch == bootSeq[state])
        {
            if (++state == 4)
                force_wd_reset();
        }
        else
            state = 0;
    }
/*
    {
        static const unsigned char bootSeq[] = { 27, 91, 75 };
        static uint8_t state = 0;

        if (ch == bootSeq[state])
        {
            if (++state == 3)
            {
                clean();
                JUNIOR_RESET_SEQ();
            }
        }
        else
            state = 0;
    }*/

    rs232.data_in(ch);
}

ISR(JUNIOR_CONCAT(JUNIOR_RS232_SIGNAL, _UDRE_vect))
{
    char ch;
    if (rs232.data_out(ch))
        UDR0 = ch;
    else
        UCSR0B &= ~(1<<UDRIE0);
}