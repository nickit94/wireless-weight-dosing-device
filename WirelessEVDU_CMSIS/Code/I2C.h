#ifndef I2C_H_
#define I2C_H_

#include "main.h"

#define AT24CXX_ADDR 0x50
#define AT24CXX_ADDR_READ   ((AT24CXX_ADDR << 1) | 0x01)
#define AT24CXX_ADDR_WRITE  ((AT24CXX_ADDR << 1) & 0xFE)

void i2cInit();
void i2cWrite(uint8_t reg_addr, uint8_t data);
uint8_t i2cRead(uint8_t reg_addr);

#endif /* I2C_H_ */
