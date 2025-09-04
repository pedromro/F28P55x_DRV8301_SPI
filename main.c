
#include "F28x_Project.h"
#include "board_regs_it.h"

int main(void)
{
    Board_initClocksAndPIE();
    Board_initGPIO();
    Board_initSPIA_IT(1000000);     // 1 MHz
    Board_initCPUTimer0_100ms();    // Kick reads every 100 ms

    // Idle loop, all the work happens in ISRs
    for (;;)
    {
        // Could put low-power IDLE here if desired
        __asm(" NOP");
    }
}
