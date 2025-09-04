#include "board_regs_it.h"
#include "drv8301.h"

void Board_initClocksAndPIE(void)
{
    InitSysCtrl();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
}

static void gpio_mux_spia_pins(void)
{
    EALLOW;
#if USE_MANUAL_CS
    // 3-wire SPIA + manual nCS on GPIO19
    GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // SIMO
    GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // SOMI
    GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // CLK
    GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0; // GPIO for nCS
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
    GpioDataRegs.GPASET.bit.GPIO19 = 1; // idle high
#else
    // 4-wire SPIA with SPISTE on GPIO19
    GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // SIMO
    GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // SOMI
    GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // CLK
    GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // SPISTE
#endif

    // LED
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioDataRegs.GPASET.bit.GPIO31 = 1;
    EDIS;
}

void Board_initGPIO(void)
{
    InitGpio();
    gpio_mux_spia_pins();
}

static inline uint16_t spi_brr_from_bitrate(uint32_t lspclk_hz, uint32_t bitrate_hz)
{
    // BitRate = LSPCLK / (SPIBRR+1) / 2  =>  SPIBRR = (LSPCLK/(2*BitRate)) - 1
    uint32_t div = (lspclk_hz / (2U * bitrate_hz));
    if (div == 0) div = 1;
    return (uint16_t)(div - 1U);
}

// Declared in header; implemented in drv8301_it.c
__interrupt void spiaRxISR(void);
__interrupt void cpuTimer0ISR(void);

void Board_initSPIA_IT(uint32_t bitrate_hz)
{
    EALLOW;
    CpuSysRegs.PCLKCR8.bit.SPI_A = 1; // enable SPIA clock
    EDIS;

    // Hold SPI in reset during config
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;

    // 16-bit, Mode 1: CPOL=0, CPHA=1
    SpiaRegs.SPICCR.bit.SPICHAR = 0xF;     // 16-bit
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;   // CPOL=0
    SpiaRegs.SPICTL.bit.CONTROLLER_PERIPHERAL = 1;  // Controller
    SpiaRegs.SPICTL.bit.TALK = 1;          // enable TX
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;     // CPHA=1
    SpiaRegs.SPICTL.bit.SPIINTENA = 1;     // route SPI int to PIE

    SpiaRegs.SPIBRR.all = spi_brr_from_bitrate(LSPCLK_HZ, bitrate_hz);

    // FIFO & RX interrupt
    SpiaRegs.SPIFFTX.bit.SPIRST = 1;       // reset FIFO
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;     // enable FIFO
    SpiaRegs.SPIFFRX.bit.RXFFIL  = 1;      // int when >=1 word
    SpiaRegs.SPIFFRX.bit.RXFFIENA= 1;      // enable RX FIFO int
    SpiaRegs.SPIFFCT.all = 0;
    SpiaRegs.SPIPRI.bit.FREE = 1;          // free run in emulation

    // Map IRQs & enable PIE group 6 for SPIA RX
    EALLOW;
    PieVectTable.SPIA_RX_INT = &spiaRxISR;
    EDIS;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;
    IER |= M_INT6;

    // Release SPI from reset
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;

    EINT;
    ERTM;
}

void Board_initCPUTimer0_100ms(void)
{
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 100000.0); // 100 ms at 200 MHz
    EALLOW;
    PieVectTable.TIMER0_INT = &cpuTimer0ISR;
    EDIS;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    IER |= M_INT1;
    CpuTimer0Regs.TCR.bit.TSS = 0; // start
}

void Board_delay_us(uint32_t us)
{
#ifdef DELAY_US
    DELAY_US(us);
#else
    volatile uint32_t i;
    for (i = 0; i < us * 200; ++i) { __asm(" NOP"); } // rough @200MHz
#endif
}

void CS_assert(void)
{
#if USE_MANUAL_CS
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
#endif
}
void CS_release(void)
{
#if USE_MANUAL_CS
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
#endif
}
