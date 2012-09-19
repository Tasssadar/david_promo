

#ifndef JUNIOR_I2C_BPS
#define JUNIOR_I2C_BPS 400000
#endif

#ifndef JUNIOR_I2C_TXBUF
# define JUNIOR_I2C_TXBUF 32
#endif

#ifndef JUNIOR_I2C_RXBUF
# define JUNIOR_I2C_RXBUF 32
#endif

#ifndef JUNIOR_I2C_BPS
# error Nastavte rychlost i2c linky!
#endif

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


struct i2c_transaction
{
    uint8_t address;
    uint8_t length;
    uint8_t result;
};

namespace detail {

class i2c_t
{
public:
    i2c_t()
        : m_on_slave_tx(0), m_max_slave_rx_len(0)
    {
        m_current_transaction.length = 0;
    }

    bool handle_slave_states(uint8_t state)
    {
        switch (state)
        {
        case 0x60:    // Own SLA+W has been received
//      case 0x68:    // Arbitration lost in SLA as Master, own SLA+W has been received
        case 0x70:    // GCA has been received
//      case 0x78:    // Arbitration lost in SLA as Master, GCA has been received
            m_current_transaction.address = 0x01;
            m_current_transaction.length = m_max_slave_rx_len;
            m_current_transaction.result = 0;

            if (m_current_transaction.result == m_current_transaction.length)
                TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
            else
                TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE);
            return true;

        case 0x80:    // Data has been received, SLA+W
        case 0x90:    // Data has been received, GCA
            if (!m_rx_queue.push(TWDR))
                m_current_transaction.length = m_current_transaction.result;
            else
                ++m_current_transaction.result;

            if (m_current_transaction.result == m_current_transaction.length)
                TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
            else
                TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE);
            return true;

        case 0x88:    // Addressed with SLA+W, data received, NACK returned
        case 0x98:    // Addressed with GCA, data received, NACK returned
        case 0xA0:    // Stop condition received
            m_results.push(m_current_transaction);
            TWCR = construct_twcr();
            return true;

        case 0xC0:    // Data byte has been transmitted, NACK was received
            TWCR = construct_twcr();
            return true;

        case 0xA8:    // Own SLA+R received, ACK returned
        case 0xB8:    // Data byte has been transmitted, ACK was received
            if (m_on_slave_tx == 0)
                TWDR = 0xff;
            else
                TWDR = m_on_slave_tx(m_on_slave_tx_data);
            TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE);
            return true;
        }
        return false;
    }

    bool handle_master_states(uint8_t state)
    {
        switch (state)
        {
        case 0x08:    // A START condition has been transmitted
        case 0x10:    // A repeated START condition has been transmitted
            if (m_master_actions.empty())
                return false;

            m_current_transaction = m_master_actions.top();
            m_master_actions.pop();
            TWDR = m_current_transaction.address;
            TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE);
            return true;

        case 0x18:    // SLA+W has been transmitted, ACK
        case 0x20:    // SLA+W has been transmitted, NACK
        case 0x28:    // Data byte has been transmitted, ACK
        case 0x30:    // Data byte has been transmitted, NACK
            if (m_current_transaction.result == m_current_transaction.length)
            {
                release_transmission();
                TWCR = construct_twcr(true);
            }
            else
            {
                TWDR = m_tx_queue.top();
                TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE);
                m_tx_queue.pop();
                ++m_current_transaction.result;
            }

            return true;

//      case 0x38:    // Arbitration lost
//          TWCR = construct_twcr();
//          return true;

        case 0x50:    // Data byte received, returned ACK
            if (!m_rx_queue.push(TWDR))
                return false;

            ++m_current_transaction.result;
            // no break
            
        case 0x40:    // SLA+R has been transmitted, ACK was received
            if (m_current_transaction.result + 1 < m_current_transaction.length)
                TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE);
            else
                TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);

            return true;

        case 0x58:    // Data byte received, returned NACK
            if (!m_rx_queue.push(TWDR))
                return false;

            ++m_current_transaction.result;
            // no break

        case 0x48:    // SLA+R has been transmitted, NACK was received
            m_results.push(m_current_transaction);
            TWCR = construct_twcr(true);
            return true;
        }
        return false;
    }

    void write(uint8_t address, uint8_t data)
    {
        this->write(address, &data, 1);
    }

    void write(uint8_t address, uint8_t * data, uint8_t length)
    {
        i2c_transaction tr = { address & 0xfe, 0, 0 };

        for (; tr.length < length; ++tr.length)
            if (!m_tx_queue.push(*data++))
                break;

        m_master_actions.push(tr);

        // FIXME: Force a START condition only if we're not already transmitting.
        TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE);
    }

    void read(uint8_t address, uint8_t length)
    {
        i2c_transaction tr = { address | 1, length, 0 };
        m_master_actions.push(tr);

        // FIXME: Force a START condition only if we're not already transmitting.
        TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE);
    }

    void max_slave_rx(uint8_t v)
    {
        m_max_slave_rx_len = v;
    }

    void on_slave_tx(uint8_t (*handler)(void * data), void * data)
    {
        m_on_slave_tx = handler;
        m_on_slave_tx_data = data;
    }

    uint8_t rx_get()
    {
        if(m_rx_queue.empty())
            return 255;
        uint8_t res = m_rx_queue.top();
        m_rx_queue.pop();
        return res;
    }

    void wait_for_result()
    {
        while (m_results.empty())
        {
        }
    }

    bool has_results() const
    {
        return !m_results.empty();
    }

    i2c_transaction get_result()
    {
        wait_for_result();
        i2c_transaction res = m_results.top();
        m_results.pop();
        return res;
    }

    void address(uint8_t address)
    {
        TWAR = address & 0xfe;
    }

    void clear()
    {
        while(!m_tx_queue.empty())
            m_tx_queue.pop();
        while(! m_rx_queue.empty())
              m_rx_queue.pop();
        while(!m_results.empty())
            m_results.pop();
        while(!m_master_actions.empty())
            m_master_actions.pop();
    }

private:
    uint8_t construct_twcr(bool active = false)
    {
        uint8_t res = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);

        if (!m_master_actions.empty())
            res |= (1<<TWSTA);
        else
            if (active)
                res |= (1<<TWSTO);

        return res;
    }

    void release_transmission()
    {
        for (uint8_t i = m_current_transaction.result; i < m_current_transaction.length; ++i)
            m_tx_queue.pop();

        m_results.push(m_current_transaction);
    }

    detail::queue<uint8_t, 64> m_tx_queue;
    detail::queue<uint8_t, 64> m_rx_queue;

    detail::queue<i2c_transaction, 16> m_master_actions;
    detail::queue<i2c_transaction, 16> m_results;

    uint8_t (* volatile m_on_slave_tx)(void * data);
    void * volatile m_on_slave_tx_data;

    uint8_t m_max_slave_rx_len;

    i2c_transaction m_current_transaction;
};

}

detail::i2c_t i2c;

inline void init_i2c()
{
#define JUNIOR_TWBR ((JUNIOR_F_CPU + JUNIOR_I2C_BPS) / (2 * JUNIOR_I2C_BPS) - 8)
    TWBR = JUNIOR_TWBR;
    TWSR = 0;
    TWAMR = 0;
    TWAR = 0;
    TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE);
}

inline void stop_i2c()
{
}

inline void clean_i2c()
{
    TWCR = 0;
}

ISR(TWI_vect)
{
    uint8_t state = TWSR & 0xf8;

    if (!i2c.handle_master_states(state)
        && !i2c.handle_slave_states(state))
    {
        // Bus error or an unexpected state,
        // reset the TWI module and indicate an error.
        TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE);
    }
}