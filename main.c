#include "f28x_project.h"
#include "board_regs_it.h"
#include "drv8301.h"

// Example callback: copy CONTROL1 data into a shadow variable
static volatile uint16_t latest_ctrl1_data11 = 0;
static void on_ctrl1_read(uint16_t raw, uint16_t data11, void *user)
{
    (void)raw; (void)user;
    latest_ctrl1_data11 = data11;
}

int main(void)
{
    Board_initClocksAndPIE();
    Board_initGPIO();
    Board_initSPIA_IT(1000000);     // 1 MHz
    Board_initCPUTimer0_100ms();    // periodic Status1 reads

    // One-shot CONTROL1 read with callback
    DRV8301_queueRead(DRV8301_REG_CONTROL1, on_ctrl1_read, 0);

    // Example write: set GATE_RESET (bit toggled high)
    DRV8301_Ctrl1_t c1 = { .all = 0 };
    c1.bits.GATE_RESET = 1;
    DRV8301_queueWrite(DRV8301_REG_CONTROL1, c1.all, 0, 0);

    for (;;)
    {
        __asm(" NOP");
    }
}
