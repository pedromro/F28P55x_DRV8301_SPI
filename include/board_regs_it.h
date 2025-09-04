
#ifndef BOARD_REGS_IT_H
#define BOARD_REGS_IT_H

#include <stdint.h>
#include <stdbool.h>
#include "F28x_Project.h"

#define USE_MANUAL_CS   1
#define DRV8301_CS_GPIO 19
#define USER_LED_GPIO   31

#ifndef LSPCLK_HZ
#define LSPCLK_HZ   50000000UL
#endif

// Public API
void Board_initClocksAndPIE(void);
void Board_initGPIO(void);
void Board_initSPIA_IT(uint32_t bitrate_hz);
void Board_initCPUTimer0_100ms(void);
void Board_delay_us(uint32_t us);

// CS helpers
void CS_assert(void);
void CS_release(void);

// Expose a start of 2-frame read for Status1 (called from Timer ISR)
void SPIA_startReadStatus1(void);

// Expose latest Status1
typedef union {
    uint16_t all;
    struct {
        uint16_t FETLC_OC : 1;
        uint16_t FETHC_OC : 1;
        uint16_t FETLB_OC : 1;
        uint16_t FETHB_OC : 1;
        uint16_t FETLA_OC : 1;
        uint16_t FETHA_OC : 1;
        uint16_t OTW      : 1;
        uint16_t OTSD     : 1;
        uint16_t PVDD_UV  : 1;
        uint16_t GVDD_UV  : 1;
        uint16_t FAULT    : 1;
        uint16_t rsvd     : 5;
    } bits;
} DRV8301_Status1_ISRView_t;

extern volatile DRV8301_Status1_ISRView_t gStatus1;

#endif
