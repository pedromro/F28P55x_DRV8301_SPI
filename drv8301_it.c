#include "f28x_project.h"
#include "board_regs_it.h"
#include "drv8301.h"

// -------- Command queue entry --------
typedef struct {
    uint16_t     is_write;  // 0=read, 1=write
    uint16_t     reg;       // DRV8301 register (0..3)
    uint16_t     data11;    // data payload (11 bits)
    drv8301_cb_t cb;        // optional callback (ISR context!)
    void        *user;      // user cookie
} drv8301_cmd_t;

// -------- Ring buffer queue (power of two) --------
#define QSIZE 8
_Static_assert((QSIZE & (QSIZE-1)) == 0, "QSIZE must be power of two");
static volatile uint16_t q_head = 0, q_tail = 0;
static drv8301_cmd_t     qbuf[QSIZE];

static inline uint16_t nexti(uint16_t i)     { return (uint16_t)((i + 1U) & (QSIZE - 1U)); }
static inline bool     q_empty(void)         { return q_head == q_tail; }
static inline bool     q_full(void)          { return nexti(q_head) == q_tail; }

static bool enqueue(drv8301_cmd_t c)
{
    DINT;
    if (q_full()) { EINT; return false; }
    qbuf[q_head] = c;
    q_head = nexti(q_head);
    EINT;
    return true;
}

static bool dequeue(drv8301_cmd_t *out)
{
    DINT;
    if (q_empty()) { EINT; return false; }
    *out = qbuf[q_tail];
    q_tail = nexti(q_tail);
    EINT;
    return true;
}

// -------- Live mirror of STATUS1 for quick watch --------
volatile DRV8301_Status1_t gStatus1 = { .all = 0 };

// -------- Transaction state (owned by ISR) --------
static volatile uint16_t rx_count     = 0;
static volatile uint16_t last_rx_word = 0;
static volatile uint16_t  busy         = 0;
static drv8301_cmd_t     cur;

// Build a DRV8301 SPI command word: W[15] A[14:11] D[10:0]
static inline uint16_t build_cmd(bool read, uint16_t reg, uint16_t data11)
{
    return ((uint16_t)(read ? 1U : 0U) << 15) | ((uint16_t)(reg & 0xFU) << 11) | (data11 & 0x07FFU);
}

// Must be called with ints disabled (DINT) — starts next transaction if any
static void start_next_locked(void)
{
    if (busy) return;
    if (!dequeue(&cur)) return;

    busy     = 1U;
    rx_count = 0;

#if USE_MANUAL_CS
    CS_assert();
#endif

    // Frame N: issue READ (or WRITE) command
    uint16_t cmd = build_cmd(!cur.is_write, cur.reg, cur.data11);
    SpiaRegs.SPITXBUF = cmd;

    // Frame N+1: dummy word to clock out response (also used to flush on write)
    SpiaRegs.SPITXBUF = 0x0000;
}

// Safe to call from thread or ISR
void DRV8301_kick(void)
{
    DINT;
    start_next_locked();
    EINT;
}

// Public enqueue API
bool DRV8301_queueRead(drv8301_reg_t reg, drv8301_cb_t cb, void *user)
{
    drv8301_cmd_t c = { .is_write = 0, .reg = (uint16_t)reg, .data11 = 0, .cb = cb, .user = user };
    if (!enqueue(c)) return false;
    DRV8301_kick();
    return true;
}

bool DRV8301_queueWrite(drv8301_reg_t reg, uint16_t data11, drv8301_cb_t cb, void *user)
{
    drv8301_cmd_t c = { .is_write = 1, .reg = (uint16_t)reg, .data11 = (uint16_t)(data11 & 0x07FFU), .cb = cb, .user = user };
    if (!enqueue(c)) return false;
    DRV8301_kick();
    return true;
}

// -------- SPIA RX FIFO ISR --------
// PIE: Group 6, INTx1 should map here (see board init)
__interrupt void spiaRxISR(void)
{
    while (SpiaRegs.SPIFFRX.bit.RXFFST > 0)
    {
        last_rx_word = SpiaRegs.SPIRXBUF;
        rx_count++;

        if (rx_count >= 2 && busy)
        {
            // Second word carries the response bits (D[10:0])
            uint16_t data11 = (last_rx_word & 0x07FFU);

            // Keep a live mirror of STATUS1 and blink LED on FAULT
            if (!cur.is_write && cur.reg == DRV8301_REG_STATUS1) {
                gStatus1.all = data11;
                if (gStatus1.bits.FAULT) {
                    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
                }
            }

#if USE_MANUAL_CS
            CS_release();
#endif

            // Capture callback data before we clear state
            drv8301_cb_t cb = cur.cb;
            void *user      = cur.user;
            uint16_t raw    = last_rx_word;

            // Ready for the next command
            busy     = 0;
            rx_count = 0;

            // Kick next queued transaction (still in ISR)
            start_next_locked();

            // Optional callback — keep it very short (ISR context!)
            if (cb) cb(raw, data11, user);
        }
    }

    // Clear RX FIFO int flag & ack PIE group
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

// -------- CPU Timer0 ISR (example periodic read) --------
// PIE: Group 1, INTx7 should map here (see board init)
__interrupt void cpuTimer0ISR(void)
{
    (void)DRV8301_queueRead(DRV8301_REG_STATUS1, 0, 0);
    CpuTimer0Regs.TCR.bit.TIF = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
