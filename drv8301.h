#ifndef DRV8301_H
#define DRV8301_H

#include <stdint.h>
#include <stdbool.h>

// ---------- DRV8301 register addresses ----------
typedef enum {
    DRV8301_REG_STATUS1   = 0x0,
    DRV8301_REG_STATUS2   = 0x1,
    DRV8301_REG_CONTROL1  = 0x2,
    DRV8301_REG_CONTROL2  = 0x3
} drv8301_reg_t;

// ---------- Bitfield unions ----------
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
} DRV8301_Status1_t;

typedef union {
    uint16_t all;
    struct {
        uint16_t DeviceID0 : 1;
        uint16_t DeviceID1 : 1;
        uint16_t DeviceID2 : 1;
        uint16_t DeviceID3 : 1;
        uint16_t rsvd0     : 6;
        uint16_t GVDD_OV   : 1;
        uint16_t rsvd1     : 6;
    } bits;
} DRV8301_Status2_t;

typedef union {
    uint16_t all;
    struct {
        uint16_t OC_ADJ_SET  : 5;
        uint16_t OCP_MODE    : 2;
        uint16_t PWM_MODE    : 1;
        uint16_t GATE_RESET  : 1;
        uint16_t GATE_CURRENT: 2;
        uint16_t rsvd        : 5;
    } bits;
} DRV8301_Ctrl1_t;

typedef union {
    uint16_t all;
    struct {
        uint16_t rsvd0      : 4;
        uint16_t OC_TOFF    : 1;
        uint16_t DC_CAL_CH2 : 1;
        uint16_t DC_CAL_CH1 : 1;
        uint16_t GAIN       : 2;
        uint16_t OCTW_MODE  : 2;
        uint16_t rsvd1      : 5;
    } bits;
} DRV8301_Ctrl2_t;

// ---------- Queue/Callback API ----------
typedef void (*drv8301_cb_t)(uint16_t raw_sdo, uint16_t data11, void *user);

bool DRV8301_queueRead(drv8301_reg_t reg, drv8301_cb_t cb, void *user);
bool DRV8301_queueWrite(drv8301_reg_t reg, uint16_t data11, drv8301_cb_t cb, void *user);

// Safe to call anytime; starts next queued transaction if idle
void DRV8301_kick(void);

// ISR entry points to map in PIE (provided by drv8301_it.c)
__interrupt void spiaRxISR(void);
__interrupt void cpuTimer0ISR(void);

// Live mirror of STATUS1 for watch window
extern volatile DRV8301_Status1_t gStatus1;

#endif
