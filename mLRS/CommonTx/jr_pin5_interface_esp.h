//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// JR Pin5 Interface Header for ESP32
//********************************************************
#ifndef JRPIN5_INTERFACE_ESP_H
#define JRPIN5_INTERFACE_ESP_H

#include "../Common/esp-lib/esp-uart.h"
#include "../Common/protocols/crsf_protocol.h"
#include <hal/uart_ll.h>

volatile bool transmitting;  // only used for half duplex JRPin5


//-------------------------------------------------------
// Clock ISR
//-------------------------------------------------------

IRQHANDLER(
void CLOCK100US_IRQHandler(void)
{
    if (transmitting) { 
        if (!uart_ll_is_tx_idle(UART_LL_GET_HW(1))) { return; }  // still transmitting

        transmitting = false;
        gpio_set_direction((gpio_num_t)UART_USE_TX_IO, GPIO_MODE_INPUT);
        gpio_matrix_in((gpio_num_t)UART_USE_TX_IO, U1RXD_IN_IDX, true);
        gpio_pulldown_dis((gpio_num_t)UART_USE_TX_IO);
        gpio_pullup_dis((gpio_num_t)UART_USE_TX_IO);
    
    }
})


//-------------------------------------------------------
// Pin5BridgeBase class

class tPin5BridgeBase
{
  public:
    void Init(void);

    // telemetry handling
    bool telemetry_start_next_tick;
    uint16_t telemetry_state;

    void TelemetryStart(void) { telemetry_start_next_tick = true; }

    // interface to the uart hardware peripheral used for the bridge, may be called in isr context
    void pin5_init(void) { uart_init(); }
    void pin5_tx_start(void) {}
    void pin5_putbuf(uint8_t* const buf, uint16_t len) { uart_putbuf(buf, len); }
    void pin5_getbuf(char* const buf, uint16_t len) { uart_getbuf(buf, len); }

    // for in-isr processing
    void IRAM_ATTR pin5_tx_enable(void);  // only used for half duplex JRPin5
    void IRAM_ATTR pin5_rx_enable(void);  // only used for half duplex JRPin5
    virtual void parse_nextchar(uint8_t c) = 0;
    virtual bool transmit_start(void) = 0; // returns true if transmission should be started

    // actual isr functions
    void IRAM_ATTR pin5_rx_callback(uint8_t c);
    void pin5_tc_callback(void) {}

    // parser
    typedef enum {
        STATE_IDLE = 0,

        // mBridge receive states
        STATE_RECEIVE_MBRIDGE_STX2,
        STATE_RECEIVE_MBRIDGE_LEN,
        STATE_RECEIVE_MBRIDGE_SERIALPACKET,
        STATE_RECEIVE_MBRIDGE_CHANNELPACKET,
        STATE_RECEIVE_MBRIDGE_COMMANDPACKET,

        // CRSF receive states
        STATE_RECEIVE_CRSF_LEN,
        STATE_RECEIVE_CRSF_PAYLOAD,
        STATE_RECEIVE_CRSF_CRC,

        // transmit states, used by all
        STATE_TRANSMIT_START,
        STATE_TRANSMITING,
    } STATE_ENUM;

    // not used in this class, but required by the children, so just add them here
    // no need for volatile since used only in isr context
    uint8_t state;
    uint8_t len;
    uint8_t cnt;
    uint16_t tlast_us;

    // check and rescue
    void CheckAndRescue(void) {}

  private:
    bool initialized = false;
};


void tPin5BridgeBase::Init(void)
{

#ifndef UART_USE_SERIAL1
  #error Serial1 must be used for JRPin5 on ESP!
#endif

    state = STATE_IDLE;
    len = 0;
    cnt = 0;
    tlast_us = 0;

    telemetry_start_next_tick = false;
    telemetry_state = 0;

    transmitting = false;

    pin5_init();
    Serial1.onReceive((void (*)(void)) uart_rx_callback_ptr, false);

#ifndef JR_PIN5_FULL_DUPLEX

    pin5_rx_enable();

    if (initialized) return;

    xTaskCreatePinnedToCore([](void *parameter) {
        hw_timer_t* timer1_cfg = nullptr;
        timer1_cfg = timerBegin(1, 800, 1);  // Timer 1, APB clock is 80 Mhz | divide by 800 is 100 KHz / 10 us, count up
        timerAttachInterrupt(timer1_cfg, &CLOCK100US_IRQHandler, true);
        timerAlarmWrite(timer1_cfg, 10, true); // 10 * 10 = 100 us
        timerAlarmEnable(timer1_cfg);
        vTaskDelete(NULL);
    }, "TimerSetup", 2048, NULL, 1, NULL, 0);  // last argument here is Core 0, ignored on ESP32C3

    initialized = true;
#endif
}


//-------------------------------------------------------
// Interface to the uart hardware peripheral used for the bridge

void IRAM_ATTR tPin5BridgeBase::pin5_tx_enable(void)
{
#ifndef JR_PIN5_FULL_DUPLEX
    gpio_set_pull_mode((gpio_num_t)UART_USE_TX_IO, GPIO_FLOATING);
    gpio_set_level((gpio_num_t)UART_USE_TX_IO, 0);
    gpio_set_direction((gpio_num_t)UART_USE_TX_IO, GPIO_MODE_OUTPUT);
    constexpr uint8_t MATRIX_DETACH_IN_LOW = 0x30; // routes 0 to matrix slot
    gpio_matrix_in(MATRIX_DETACH_IN_LOW, U1RXD_IN_IDX, false); // Disconnect RX from all pads
    gpio_matrix_out((gpio_num_t)UART_USE_TX_IO, U1TXD_OUT_IDX, true, false);
#endif
}


void IRAM_ATTR tPin5BridgeBase::pin5_rx_enable(void)
{
#ifndef JR_PIN5_FULL_DUPLEX
    gpio_set_direction((gpio_num_t)UART_USE_TX_IO, GPIO_MODE_INPUT);
    gpio_matrix_in((gpio_num_t)UART_USE_TX_IO, U1RXD_IN_IDX, true);
    gpio_pulldown_dis((gpio_num_t)UART_USE_TX_IO);
    gpio_pullup_dis((gpio_num_t)UART_USE_TX_IO);
#endif
}


//-------------------------------------------------------
// Receiver callback

void IRAM_ATTR tPin5BridgeBase::pin5_rx_callback(uint8_t c)
{
    // poll uart
    char buf[CRSF_FRAME_LEN_MAX + 16];
    uint16_t available = uart_rx_bytesavailable();
    available = MIN(available, CRSF_FRAME_LEN_MAX);
    
    pin5_getbuf(buf, available);
    
    for (uint16_t i = 0; i < available; i++) {
        if (state >= STATE_TRANSMIT_START) break; // read at most 1 message
        parse_nextchar(buf[i]);
    }

    // send telemetry after every received message
    if (state == STATE_TRANSMIT_START) {
        pin5_tx_enable();
        
        if(!transmit_start()) {   
            transmitting = false;
            pin5_rx_enable();
        } else {
            transmitting = true;
        }
        
        state = STATE_IDLE;
    }
}


//-------------------------------------------------------
// Pin5 Serial class

class tJrPin5SerialPort : public tSerialBase
{
  public:
    void Init(void) override { uart_init(); }
    void SetBaudRate(uint32_t baud) override { uart_setprotocol(baud, XUART_PARITY_NO, UART_STOPBIT_1); }
    bool full(void) { return !uart_tx_notfull(); }
    void putbuf(uint8_t* const buf, uint16_t len) override { uart_putbuf(buf, len); }
    bool available(void) override { return uart_rx_available(); }
    char getc(void) override { return uart_getc(); }
    void getbuf(char* const buf, uint16_t len) override { uart_getbuf(buf, len); }
    void flush(void) override { uart_rx_flush(); uart_tx_flush(); }
    uint16_t bytes_available(void) override { return uart_rx_bytesavailable(); }
};

tJrPin5SerialPort jrpin5serial;


#endif // JRPIN5_INTERFACE_ESP_H
