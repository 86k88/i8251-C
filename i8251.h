#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

typedef enum {
    I8251_ASYNC = 0,
    I8251_SYNC
} i8251_format_t;

typedef enum {
    I8251_MODE_INSTR = 0,
    I8251_SYNC_CHAR_1,
    I8251_SYNC_CHAR_2,
    I8251_COMMAND_INSTR,
    I8251_MODE_LEN
} i8251_mode_enum_t;

typedef struct {
    uint16_t bits;
    uint8_t  bits_len;
} bitstream_t;

typedef struct {
    bool    txrdy;
    bool    rxrdy;
    bool    txe;
    bool    pe; // parity error
    bool    oe; // overrun error
    bool    fe; // framing error, async only?
    bool    syndet;
    bool    dsr;
} i8251_status_t;

typedef struct {
    bool    texn; // transmit enable
    bool    dtr;  
    bool    rxe;  // receive enable
    bool    sbrk; // send break character
    bool    er;   // error reset, clears all error flags
    bool    rts; 
    bool    ir;   // internal reset, returns the UART into Mode Instruction 
    bool    eh;   // hunt mode (yes), searches for sync characters
} i8251_command_t;

typedef struct {
    uint8_t char_len;
    bool    pen;
    bool    ep;
    bool    esd;
    bool    scs;
} i8251_mode_sync_t;

typedef struct {
    uint8_t baud_sel;
    uint8_t char_len;
    uint8_t stop_len;
    bool    pen;
    bool    ep;
} i8251_mode_async_t;

typedef struct {
    i8251_format_t     format;
    i8251_mode_enum_t  mode_st;
    union {
        i8251_mode_async_t asynchronous_mode;
        i8251_mode_sync_t synchronous_mode;
    } mode;
    i8251_command_t    command;
    i8251_status_t     status;
    uint8_t rx_data;
    uint8_t tx_data;
    uint8_t sync_char1;
    uint8_t sync_char2;
    bitstream_t tx_shift;
    bitstream_t rx_shift;
    uint8_t     tx_pin;
    uint8_t     tx_clk_cnt;
    uint8_t     rx_clk_cnt;
    bool        rx_busy;

} i8251_usart_t;

void i8251_init(i8251_usart_t *usart);
void i8251_write(i8251_usart_t *usart, bool control_data, uint8_t data);
uint8_t i8251_read(i8251_usart_t *usart, bool control_data);
void i8251_tick_tx(i8251_usart_t *usart);
void i8251_tick_rx(i8251_usart_t *usart, uint8_t rxd);
uint8_t i8251_tx_line(i8251_usart_t *usart);
