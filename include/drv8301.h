
#ifndef DRV8301_H
#define DRV8301_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    DRV8301_REG_STATUS1   = 0x0,
    DRV8301_REG_STATUS2   = 0x1,
    DRV8301_REG_CONTROL1  = 0x2,
    DRV8301_REG_CONTROL2  = 0x3
} drv8301_reg_t;

#endif
