
#include "board_regs_it.h"
#include "drv8301.h"

// ---------------- Globals for ISR communication ----------------
volatile DRV8301_Status1_ISRView_t gStatus1 = { .all = 0 };

static volatile uint16_t rx_count = 0;
static volatile uint16_t last_rx_word = 0;
static volatile enum { SPI_IDLE=0, SPI_READING_STATUS1 } spi_state = SPI_IDLE;

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
    GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // SIMO
    GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // SOMI
    GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // CLK

    GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0; // GPIO for nCS
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
#else
    GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;
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
    uint32_t div = (lspclk_hz / (2U * bitrate_hz));
    if (div == 0) div = 1;
    return (uint16_t)(div - 1U);
}

__interrupt void spiaRxISR(void);
__interrupt void cpuTimer0ISR(void);

void Board_initSPIA_IT(uint32_t bitrate_hz)
{
    EALLOW;
    CpuSysRegs.PCLKCR8.bit.SPI_A = 1;
    EDIS;

    // Hold reset
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;

    SpiaRegs.SPICCR.bit.SPICHAR = 0xF;     // 16-bit
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;   // CPOL=0
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;  // Master
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;     // CPHA=1
    SpiaRegs.SPICTL.bit.SPIINTENA = 1;     // Enable SPI INT to PIE

    SpiaRegs.SPIBRR.all = spi_brr_from_bitrate(LSPCLK_HZ, bitrate_hz);

    // FIFO & RX interrupt
    SpiaRegs.SPIFFTX.bit.SPIRST = 1;
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;
    SpiaRegs.SPIFFRX.bit.RXFFIL = 1;        // Interrupt when >=1 word
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;      // Enable RX FIFO int
    SpiaRegs.SPIFFCT.all = 0;
    SpiaRegs.SPIPRI.bit.FREE = 1;

    // Map ISR and enable PIE group 6 / INTx1 for SPIA RX
    EALLOW;
    PieVectTable.SPIA_RX_INT = &spiaRxISR;
    EDIS;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1; // SPIA RX
    IER |= M_INT6;

    // Release SPI
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;

    EINT;
    ERTM;
}

void Board_initCPUTimer0_100ms(void)
{
    // CPU timer 0 @ system clock; we use macros from headers
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 100000.0); // 200 MHz CPU, 100 ms period
    EALLOW;
    PieVectTable.TIMER0_INT = &cpuTimer0ISR;
    EDIS;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // Group1, INT7 = TINT0
    IER |= M_INT1;
    CpuTimer0Regs.TCR.bit.TSS = 0; // start
}

// ---------- CS helpers ----------
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

// ---------- Transaction kick (called by Timer ISR) ----------
void SPIA_startReadStatus1(void)
{
    if (spi_state != SPI_IDLE) return;

    spi_state = SPI_READING_STATUS1;
    rx_count = 0;

#if USE_MANUAL_CS
    CS_assert();
#endif

    // Frame N: READ command
    uint16_t cmd = ((uint16_t)1U << 15) | ((uint16_t)(DRV8301_REG_STATUS1 & 0xFU) << 11);
    SpiaRegs.SPITXBUF = cmd;

    // Frame N+1: dummy to clock out response
    SpiaRegs.SPITXBUF = 0x0000;
}

// ---------- ISRs ----------
__interrupt void spiaRxISR(void)
{
    while (SpiaRegs.SPIFFRX.bit.RXFFST > 0)
    {
        last_rx_word = SpiaRegs.SPIRXBUF;
        rx_count++;

        if (spi_state == SPI_READING_STATUS1 && rx_count >= 2)
        {
            // Second word is the response: parse D[10:0]
            uint16_t data11 = (last_rx_word & 0x07FFU);
            gStatus1.all = data11;

#if USE_MANUAL_CS
            CS_release();
#endif
            // Side effect: toggle LED when FAULT set
            if (gStatus1.bits.FAULT) {
                GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
            }

            spi_state = SPI_IDLE;
            rx_count = 0;
        }
    }

    // Clear RX FIFO int flag and ack PIE
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

__interrupt void cpuTimer0ISR(void)
{
    SPIA_startReadStatus1();

    CpuTimer0Regs.TCR.bit.TIF = 1;              // Clear timer flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
