#ifndef BOARD_REGS_IT_H
#define BOARD_REGS_IT_H

#include <stdint.h>
#include <stdbool.h>
#include "f28x_project.h"

// -------- Board configuration --------
#define USE_MANUAL_CS   1     // 1: use GPIO19 as manual nCS; 0: use SPISTE on GPIO19
#define DRV8301_CS_GPIO 19
#define USER_LED_GPIO   31

#ifndef LSPCLK_HZ
#define LSPCLK_HZ   50000000UL // Typical when CPU=200 MHz
#define CPU_HZ 200000000UL // CPU frequency
#endif

// -------- Init API --------
void Board_initClocksAndPIE(void);
void Board_initGPIO(void);
void Board_initSPIA_IT(uint32_t bitrate_hz);
void Board_initCPUTimer0_100ms(void);
void Board_delay_us(uint32_t us);

// -------- Manual CS helpers --------
void CS_assert(void);
void CS_release(void);

#endif
