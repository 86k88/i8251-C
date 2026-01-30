#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "i8251.h"

static i8251_command_t decode_command(uint8_t command)
{
    i8251_command_t mode = {0};
    mode.texn = (command >> 0) & 1;
    mode.dtr  = (command >> 1) & 1;
    mode.rxe  = (command >> 2) & 1;
    mode.sbrk = (command >> 3) & 1;
    mode.er   = (command >> 4) & 1;
    mode.rts  = (command >> 5) & 1;
    mode.ir   = (command >> 6) & 1;
    mode.eh   = (command >> 7) & 1;
    return mode;
}

static bool parity_bit(uint8_t data, uint8_t data_len, bool even)
{
    data &= (data_len >= 8) ? 0xFFu : (uint8_t)((1u << data_len) - 1u);
    uint8_t x = data;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    bool odd_parity = x & 1u;
    return even ? odd_parity : !odd_parity;
}

static void error_clear(i8251_usart_t *u)
{
    u->status.pe = false;
    u->status.oe = false;
    u->status.fe = false;
}

static bitstream_t create_bitstream_sync(uint8_t data, const i8251_mode_sync_t *m)
{
    bitstream_t bitstream = {0};
    uint8_t data_len = m->char_len;
    if (data_len > 8) data_len = 8;

    data &= (data_len >= 8) ? 0xFFu : (uint8_t)((1u << data_len) - 1u);

    for (uint8_t i = 0; i < data_len; i++)
    {
        bitstream.bits |= ((data >> i) & 1u) << bitstream.bits_len;
        bitstream.bits_len++;
    }

    if (m->pen)
    {
        bool pb = parity_bit(data, data_len, m->ep);
        bitstream.bits |= (uint16_t)(pb ? 1u : 0u) << bitstream.bits_len;
        bitstream.bits_len++;
    }

    return bitstream;
}

static bitstream_t create_bitstream_async(uint8_t data, const i8251_mode_async_t *m)
{
    bitstream_t bitstream = {0};
    uint8_t data_len = m->char_len;
    if (data_len > 8) data_len = 8;

    data &= (data_len >= 8) ? 0xFFu : (uint8_t)((1u << data_len) - 1u);

    // make sure the first bit is always 0 since it's a start bit
    bitstream.bits_len++;

    for (uint8_t i = 0; i < data_len; i++) {
        bitstream.bits |= ((data >> i) & 1u) << bitstream.bits_len;
        bitstream.bits_len++;
    }

    if (m->pen) {
        bool pb = parity_bit(data, data_len, m->ep);
        bitstream.bits |= (uint16_t)(pb ? 1u : 0u) << bitstream.bits_len;
        bitstream.bits_len++;
    }

    uint8_t stop_len = m->stop_len ? m->stop_len : 1;
    for (uint8_t i = 0; i < stop_len; i++) {
        bitstream.bits |= 1u << bitstream.bits_len;
        bitstream.bits_len++;
    }

    return bitstream;
}

static i8251_mode_async_t decode_async_mode(uint8_t mode)
{
    i8251_mode_async_t m = {0};

    switch (mode & 0x03) {
        case 0b01: m.baud_sel = 1;  break;
        case 0b10: m.baud_sel = 16; break;
        case 0b11: m.baud_sel = 64; break;
        default:
            // 00 = sync mode selector, invalid for async mode
            m.baud_sel = 0;
            break;
    }


    switch ((mode >> 2) & 0x03) {
        case 0b00: m.char_len = 5; break;
        case 0b01: m.char_len = 6; break;
        case 0b10: m.char_len = 7; break;
        case 0b11: m.char_len = 8; break;
    }

    m.pen = (mode >> 4) & 1;
    m.ep  = (mode >> 5) & 1;

    switch ((mode >> 6) & 0x03) {
        case 0b01: m.stop_len = 1; break;
        case 0b10: m.stop_len = 2; break; // 1.5 bits. treat as 2 for now
        case 0b11: m.stop_len = 2; break;
        default:
            // 00, invalid per datasheet
            m.stop_len = 0;
            break;
    }

    return m;
}

static i8251_mode_sync_t decode_sync_mode(uint8_t mode)
{
    i8251_mode_sync_t m = {0};

    // D7: Single / double sync
    m.scs = (mode >> 7) & 1;

    // D6: External sync detect
    m.esd = (mode >> 6) & 1;

    // D5: Even parity
    m.ep  = (mode >> 5) & 1;

    // D4: Parity enable
    m.pen = (mode >> 4) & 1;

    // D3–D2: Character length
    switch ((mode >> 2) & 0x3) {
        case 0b00: m.char_len = 5; break;
        case 0b01: m.char_len = 6; break;
        case 0b10: m.char_len = 7; break;
        case 0b11: m.char_len = 8; break;
    }

    return m;
}

static uint8_t i8251_div_async(const i8251_usart_t *usart)
{
    uint8_t d = usart->mode.asynchronous_mode.baud_sel;
    if (d == 0) d = 1;
    return d;
}

static uint8_t i8251_async_total_bits(const i8251_usart_t *usart)
{
    uint8_t data_len = usart->mode.asynchronous_mode.char_len;
    if (data_len > 8) data_len = 8;

    uint8_t stop_len = usart->mode.asynchronous_mode.stop_len;
    if (stop_len == 0) stop_len = 1;

    uint8_t pen = usart->mode.asynchronous_mode.pen ? 1u : 0u;
    return (uint8_t)(1u + data_len + pen + stop_len);
}

static void i8251_tick_tx_clock(i8251_usart_t *usart)
{
    if (usart->format != I8251_ASYNC)
    {
        i8251_tick_tx(usart);
        return;
    }

    uint8_t div = i8251_div_async(usart);

    usart->tx_clk_cnt++;
    if (usart->tx_clk_cnt < div)
        return;

    usart->tx_clk_cnt = 0;
    i8251_tick_tx(usart);
}
static void i8251_rx_complete_async(i8251_usart_t *usart, uint8_t data, bool parity_rx, bool stop_ok)
{
    if (!usart->command.rxe)
        return;

    if (usart->status.rxrdy)
    {
        usart->status.oe = true;
        return;
    }

    uint8_t data_len = usart->mode.asynchronous_mode.char_len;
    if (data_len > 8) data_len = 8;

    data &= (data_len >= 8) ? 0xFFu : (uint8_t)((1u << data_len) - 1u);

    usart->rx_data = data;
    usart->status.rxrdy = true;

    if (usart->mode.asynchronous_mode.pen)
    {
        bool expected = parity_bit(data, data_len, usart->mode.asynchronous_mode.ep);
        if (expected != parity_rx)
            usart->status.pe = true;
    }

    if (!stop_ok)
        usart->status.fe = true;
}
static void i8251_tick_rx_clock(i8251_usart_t *usart, uint8_t rxd)
{
    if (!usart->command.rxe)
        return;

    if (usart->format != I8251_ASYNC)
        return;

    if (!usart->rx_busy)
    {
        if (rxd == 0)
        {
            usart->rx_busy = true;
            usart->rx_clk_cnt = 0;
            usart->rx_shift = (bitstream_t){0};
        }
        return;
    }

    uint8_t div = i8251_div_async(usart);

    usart->rx_clk_cnt++;
    if (usart->rx_clk_cnt < div)
        return;

    usart->rx_clk_cnt = 0;

    if (usart->rx_shift.bits_len < 16)
    {
        usart->rx_shift.bits |= (uint16_t)((rxd ? 1u : 0u) << usart->rx_shift.bits_len);
        usart->rx_shift.bits_len++;
    }

    uint8_t need = i8251_async_total_bits(usart);
    if (usart->rx_shift.bits_len < need)
        return;

    uint8_t data_len = usart->mode.asynchronous_mode.char_len;
    if (data_len > 8) data_len = 8;

    uint8_t stop_len = usart->mode.asynchronous_mode.stop_len;
    if (stop_len == 0) stop_len = 1;

    uint8_t mask = (data_len >= 8) ? 0xFFu : (uint8_t)((1u << data_len) - 1u);

    uint16_t sh = usart->rx_shift.bits;

    bool start_ok = ((sh >> 0) & 1u) ? false : true;

    uint8_t data = (uint8_t)((sh >> 1) & mask);

    bool parity_rx = false;
    if (usart->mode.asynchronous_mode.pen)
        parity_rx = ((sh >> (1 + data_len)) & 1u) ? true : false;

    bool stop_ok = true;
    uint8_t stop_base = (uint8_t)(1 + data_len + (usart->mode.asynchronous_mode.pen ? 1u : 0u));
    for (uint8_t i = 0; i < stop_len; i++)
    {
        if (((sh >> (stop_base + i)) & 1u) == 0)
        {
            stop_ok = false;
            break;
        }
    }

    if (start_ok)
        i8251_rx_complete_async(usart, data, parity_rx, stop_ok);
    else
        usart->status.fe = true;

    usart->rx_busy = false;
    usart->rx_shift = (bitstream_t){0};
}

uint8_t i8251_read(i8251_usart_t *usart, bool control_data)
{
    uint8_t val = 0;
    if (control_data)
    {
        usart->status.dsr = usart->dsr; // make sure other end is reflected onto the chip
        val |= (usart->status.txrdy ? 1u : 0u) << 0;
        val |= (usart->status.rxrdy ? 1u : 0u) << 1;
        val |= (usart->status.txe   ? 1u : 0u) << 2;
        val |= (usart->status.pe    ? 1u : 0u) << 3;
        val |= (usart->status.oe    ? 1u : 0u) << 4;
        val |= (usart->status.fe    ? 1u : 0u) << 5;
        val |= (usart->status.syndet? 1u : 0u) << 6;
        val |= (usart->status.dsr   ? 1u : 0u) << 7;

    } else {

        val = usart->rx_data;
        usart->status.rxrdy = false;
    }
    return val;
}

uint8_t i8251_tx_line(i8251_usart_t *usart)
{
    return usart->tx_pin ? 1u : 0u;
}

void i8251_tick_tx(i8251_usart_t *usart)
{
    if (!usart->command.texn)
    {
        usart->tx_pin = 1;
        usart->tx_clk_cnt = 0;
        return;
    }

    if (usart->command.sbrk)
    {
        usart->tx_pin = 0;
        usart->status.txrdy = false;
        usart->status.txe = false;
        usart->tx_clk_cnt = 0;
        return;
    }

    if (usart->format == I8251_ASYNC)
    {
        if (!usart->cts)
        {
            usart->tx_pin = 1;
            usart->status.txrdy = false; // not sure if i need to do this, but since it's a "clear to send", maybe it's possible that the flags also get cleared.
            usart->status.txe = false;
            return;
        }
        uint8_t div = usart->mode.asynchronous_mode.baud_sel;
        if (div == 0) div = 1;

        usart->tx_clk_cnt++;
        if (usart->tx_clk_cnt < div)
            return;

        usart->tx_clk_cnt = 0;
    }
    else
    {
        usart->tx_clk_cnt = 0;
    }

    if (usart->tx_shift.bits_len == 0)
    {
        usart->tx_pin = 1;
        usart->status.txrdy = true;
        usart->status.txe = true;
        return;
    }

    usart->tx_pin = (usart->tx_shift.bits & 1u) ? 1u : 0u;

    usart->tx_shift.bits >>= 1;
    usart->tx_shift.bits_len--;

    if (usart->tx_shift.bits_len == 0)
    {
        usart->status.txrdy = true;
        usart->status.txe = true;
        usart->tx_pin = 1;
    }
}

void i8251_tick_rx(i8251_usart_t *usart, uint8_t rxd)
{
    if (!usart->command.rxe)
    {
        usart->rx_clk_cnt = 0;
        usart->rx_busy = false;
        usart->rx_shift = (bitstream_t){0};
        return;
    }

    if (usart->format != I8251_ASYNC)
    {
        usart->rx_clk_cnt = 0;
        usart->rx_busy = false;
        usart->rx_shift = (bitstream_t){0};
        return;
    }

    if (!usart->rx_busy)
    {
        usart->rx_clk_cnt = 0;
        usart->rx_shift = (bitstream_t){0};

        if (rxd == 0)
        {
            usart->rx_busy = true;
            usart->rx_shift.bits |= 0u << usart->rx_shift.bits_len;
            usart->rx_shift.bits_len++;
        }
        return;
    }

    uint8_t div = usart->mode.asynchronous_mode.baud_sel;
    if (div == 0) div = 1;

    usart->rx_clk_cnt++;
    if (usart->rx_clk_cnt < div)
        return;

    usart->rx_clk_cnt = 0;

    if (usart->rx_shift.bits_len < 16)
    {
        usart->rx_shift.bits |= (uint16_t)((rxd ? 1u : 0u) << usart->rx_shift.bits_len);
        usart->rx_shift.bits_len++;
    }

    uint8_t data_len = usart->mode.asynchronous_mode.char_len;
    if (data_len > 8) data_len = 8;

    uint8_t stop_len = usart->mode.asynchronous_mode.stop_len;
    if (stop_len == 0) stop_len = 1;

    uint8_t need = (uint8_t)(1u + data_len + (usart->mode.asynchronous_mode.pen ? 1u : 0u) + stop_len);
    if (usart->rx_shift.bits_len < need)
        return;

    uint16_t sh = usart->rx_shift.bits;

    bool start_ok = ((sh >> 0) & 1u) ? false : true;

    uint8_t mask = (data_len >= 8) ? 0xFFu : (uint8_t)((1u << data_len) - 1u);
    uint8_t data = (uint8_t)((sh >> 1) & mask);

    bool parity_rx = false;
    if (usart->mode.asynchronous_mode.pen)
        parity_rx = ((sh >> (1 + data_len)) & 1u) ? true : false;

    bool stop_ok = true;
    uint8_t stop_base = (uint8_t)(1 + data_len + (usart->mode.asynchronous_mode.pen ? 1u : 0u));
    for (uint8_t i = 0; i < stop_len; i++)
    {
        if (((sh >> (stop_base + i)) & 1u) == 0)
        {
            stop_ok = false;
            break;
        }
    }

    if (start_ok)
        i8251_rx_complete_async(usart, data, parity_rx, stop_ok);
    else
        usart->status.fe = true;

    usart->rx_busy = false;
    usart->rx_shift = (bitstream_t){0};
}



void i8251_write(i8251_usart_t *usart, bool control_data, uint8_t data)
{
    uint8_t val = data;

    if (!control_data)
    {
        usart->tx_data = data;

        if (usart->command.texn)
        {
            if (usart->format == I8251_ASYNC)
            {
                if (usart->mode.asynchronous_mode.stop_len == 0)
                    usart->mode.asynchronous_mode.stop_len = 1;

                usart->tx_shift = create_bitstream_async(data, &usart->mode.asynchronous_mode);
                usart->status.txrdy = false;
                usart->status.txe = false;
                return;
            }

            if (usart->format == I8251_SYNC)
            {
                usart->tx_shift = create_bitstream_sync(data, &usart->mode.synchronous_mode);
                usart->status.txrdy = false;
                usart->status.txe = false;
                return;
            }
        }

        return;
    }

    switch (usart->mode_st)
    {
        case I8251_MODE_INSTR:
        {
            if ((val & 0x03) == 0)
            {
                usart->format = I8251_SYNC;
                usart->mode.synchronous_mode = decode_sync_mode(val);
                usart->mode_st = I8251_SYNC_CHAR_1;
                break;
            }
            else
            {
                usart->format = I8251_ASYNC;
                usart->mode.asynchronous_mode = decode_async_mode(val);

                if (usart->mode.asynchronous_mode.stop_len == 0)
                    usart->mode.asynchronous_mode.stop_len = 1;

                usart->mode_st = I8251_COMMAND_INSTR;
                break;
            }
        }

        case I8251_SYNC_CHAR_1:
        {
            usart->sync_char1 = val;

            if (usart->mode.synchronous_mode.scs)
                usart->mode_st = I8251_COMMAND_INSTR;
            else
                usart->mode_st = I8251_SYNC_CHAR_2;

            break;
        }

        case I8251_SYNC_CHAR_2:
        {
            usart->sync_char2 = val;
            usart->mode_st = I8251_COMMAND_INSTR;
            break;
        }

        case I8251_COMMAND_INSTR:
        {
            usart->command = decode_command(val);
            usart->rts = usart->command.rts;
            usart->dtr = usart->command.dtr;
            if (usart->command.er)
                error_clear(usart);

            if (usart->command.ir)
            {
                error_clear(usart);
                usart->mode_st = I8251_MODE_INSTR;
                usart->status.rxrdy = false;
                usart->status.txrdy = true;
                usart->status.txe = true;
                usart->tx_shift = (bitstream_t){0};
                usart->tx_pin = 1;
                usart->tx_clk_cnt = 0;
                usart->rx_clk_cnt = 0;
                usart->rx_busy = false;
                usart->rx_shift = (bitstream_t){0};
                usart->cts = true;  // make sure to reset state on IR.
                usart->dsr = true;
            }

            break;
        }

        default:
            break;
    }
}



void i8251_init(i8251_usart_t *usart)
{
    *usart = (i8251_usart_t){0};
    usart->format = I8251_ASYNC;
    usart->mode_st = I8251_MODE_INSTR;
    usart->status.txrdy = true;
    usart->status.txe = true;
    usart->tx_pin = 1;
    usart->tx_clk_cnt = 0;
    usart->rx_clk_cnt = 0;
    usart->rx_busy = false;
    usart->cts = true;
    usart->dsr = true;
}