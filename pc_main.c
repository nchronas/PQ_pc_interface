/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== uartecho.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

#include <ti/drivers/ADC.h>
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Watchdog.h>
#define __MSP432P401R__
#include <ti/devices/msp432p4xx/driverlib/uart.h>
#include <ti/drivers/uart/UARTMSP432.h>

#include <ti/sysbios/knl/Clock.h>

#include <FS_Board.h>
//#include "RED_Board.h"


/*
 *  ------------ UART functions ------------
 *  Each subsystem has the same uart and rs-485 peripherals, so no need for change.
 */

UART_Handle uart_dbg_bus;
UART_Handle uart_pq9_bus;
UART_Handle uart_pc_bus;
UART_Handle uart_inj_bus;

void temp(UART_Handle handle, void *buf, size_t count) {

}



void uart_test() {

      UART_Params uartParams;

      UART_Params_init(&uartParams);
      uartParams.writeMode = UART_MODE_BLOCKING;
      uartParams.writeDataMode = UART_DATA_BINARY;

      uartParams.readMode = UART_MODE_BLOCKING;
      uartParams.readDataMode = UART_DATA_BINARY;
      uartParams.readTimeout = 1;
      uartParams.readReturnMode = UART_RETURN_FULL;
      uartParams.readEcho = UART_ECHO_OFF;
      uartParams.baudRate = 115200;
      uart_dbg_bus = UART_open(DBG, &uartParams);

      UART_Params_init(&uartParams);
      uartParams.writeMode = UART_MODE_BLOCKING;
      uartParams.writeDataMode = UART_DATA_BINARY;

      uartParams.readMode = UART_MODE_BLOCKING;
      uartParams.readDataMode = UART_DATA_BINARY;
      uartParams.readTimeout = 1;
      uartParams.readReturnMode = UART_RETURN_FULL;
      uartParams.readEcho = UART_ECHO_OFF;
      uartParams.baudRate = 115200;
      uart_inj_bus = UART_open(INJ, &uartParams);

      UART_Params_init(&uartParams);
      uartParams.writeMode = UART_MODE_BLOCKING;
      uartParams.writeDataMode = UART_DATA_BINARY;

      uartParams.readMode = UART_MODE_BLOCKING;
      uartParams.readDataMode = UART_DATA_BINARY;
      uartParams.readTimeout = 1;
      uartParams.readReturnMode = UART_RETURN_FULL;
      uartParams.readEcho = UART_ECHO_OFF;
      uartParams.baudRate = 115200;
      uart_pc_bus = UART_open(PC_, &uartParams);

}

void rs_test() {


  UART_Params uartParams;

  GPIO_write(PQ9_EN, 0);

  /* Create a UART with data processing off. */
  UART_Params_init(&uartParams);
  uartParams.writeMode = UART_MODE_BLOCKING;
  uartParams.writeDataMode = UART_DATA_BINARY;
  uartParams.readMode = UART_MODE_CALLBACK;
  uartParams.readDataMode = UART_DATA_BINARY;
  uartParams.readReturnMode = UART_RETURN_FULL;
  uartParams.readEcho = UART_ECHO_OFF;
  uartParams.baudRate = 115200; //500000;
  uartParams.readCallback = &temp;

  uart_pq9_bus = UART_open(PQ9, &uartParams);

  if(uart_pq9_bus == NULL) {
    usleep(1);
  }

  UARTMSP432_HWAttrsV1 const *hwAttrs = uart_pq9_bus->hwAttrs;
  UART_setDormant(hwAttrs->baseAddr);

}

uint8_t get_subs_addr() {

  return 0x01; //OBC

  //return 0x07; //DBG

}

extern uint8_t pq_rx_buf[300];
extern uint16_t pq_rx_count, pq_size;
extern bool pq_rx_flag;

#define HLDLC_START_FLAG        0x7E
#define HLDLC_CONTROL_FLAG      0x7D
#define HLDLC_STOP_FLAG         0x7C

void HLDLC_deframe(uint8_t *buf_in,
                              uint8_t *buf_out,
                              const uint16_t size_in,
                              uint16_t *size_out) {


    uint16_t cnt = 0;

    for(uint16_t i = 0; i < size_in; i++) {

        uint8_t c = buf_in[i];

        if(c == HLDLC_START_FLAG) {
            cnt = 0;
        } else if(c == HLDLC_STOP_FLAG) {
            *size_out = cnt;
            return ;
        } else if(c == HLDLC_CONTROL_FLAG) {
            i++;
            c = buf_in[i];

            if(c == 0x5E) {
              buf_out[cnt++] = 0x7E;
            } else if(c == 0x5D) {
              buf_out[cnt++] = 0x7D;
            } else if(c== 0x5C) {
              buf_out[cnt++] = 0x7C;
            } else {
              return ;
            }
        } else {
            buf_out[cnt++] = c;
        }

    }
    return ;
}



uint8_t buf_uart43[100];
uint8_t buf_uart_hldlc[100];
uint16_t buf_uart_cnt = 0;
uint16_t buf_uart_hldlc_cnt = 0;
uint8_t resp54[3];
uint8_t res;

unsigned  char buf_rs[100];
uint16_t buf_cnt = 0;

bool ctrl_flag, strt_flag = false;

unsigned  char buf_inj[100];
uint16_t inj_cnt = 0;

void pc_interface() {

  UARTMSP432_Object *object = uart_pq9_bus->object;

  uint8_t tx_count, tx_size, tx_buf[255];

  uint32_t res_t, time_now;
  time_now = Clock_getTicks();

  while(1) {


    res_t = Clock_getTicks();
    if(res_t - time_now > 500) {
       GPIO_toggle(BLINK_LED);
       time_now = res_t;
    }

    // PQ9 -> PC
    while(RingBuf_get(&object->ringBuffer, &buf_rs[buf_cnt]) != -1) {
      buf_cnt++;
      //usleep(1);
    }
    if(buf_cnt > 0) {
      //sprintf(temp,"%02x ", buf_rs[buf_cnt]);
      //UART_write(uart_dbg_bus, temp, strlen(temp));
      UART_write(uart_pc_bus, buf_rs, buf_cnt);
      buf_cnt = 0;
    }

    // PC -> PQ9
    do {
      res = UART_read(uart_pc_bus, resp54, 1);
      if(res > 0) {
        buf_inj[inj_cnt] = resp54[0];
        inj_cnt++;
        if(resp54[0] == HLDLC_START_FLAG) {
          strt_flag = true;
          buf_inj[0] = resp54[0];
          inj_cnt = 1;
        } else if(resp54[0] == HLDLC_CONTROL_FLAG) {
          ctrl_flag = true;
        } else if(strt_flag) {
           strt_flag = false;
           tx_buf[0] = resp54[0];
           tx_count = 1;
        } else if(ctrl_flag) {
           ctrl_flag = false;
           if(resp54[0] == 0x5D) {
             tx_buf[tx_count] = 0x7D;
             tx_count++;
           } else if(resp54[0] == 0x5E) {
             tx_buf[tx_count] = 0x7E;
             tx_count++;
           }
        } else if(tx_count == 1) {
          tx_buf[tx_count] = resp54[0];
          tx_size = resp54[0] + 5;
          tx_count++;
        } else if(tx_count > 0 && tx_count < tx_size - 1) {
          tx_buf[tx_count] = resp54[0];
          tx_count++;
        } else if(tx_count > 0 && tx_count == tx_size - 1) {
          tx_buf[tx_count] = resp54[0];
          tx_count++;
          if(tx_buf[0] < 127) {
            GPIO_write(PQ9_EN, 1);
            UART_writePolling(uart_pq9_bus, tx_buf, tx_count);
            GPIO_write(PQ9_EN, 0);
          } else {
            UART_write(uart_inj_bus, buf_inj, inj_cnt);
          }
        }
      }
    } while(res > 0);

  }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    /* Call driver init functions */
    GPIO_init();
    UART_init();

    uart_test();

    rs_test();

    pc_interface();

    /* Loop forever echoing */
    while (1) {

    }
}

/*  ======== wdgThread ========
 *  This thread runs on a higher priority, since wdg pin
 *  has to be ready for master.
 */
#if (SUBS_TESTS > 0)
void *wdgThread(void *arg0)
{

    /* Loop forever */
    while (1) {
        wdg_reset();
    }

    return (NULL);
}
#endif
