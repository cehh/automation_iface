/*
 * INA226 Usage in power measurement
 * For more information see http://www.ti.com/product/ina226
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>

#include "ina226_reg_defns.h"
#include "power_data.h"
#include "ina226.h"


static int _ina226_read(I2C_Handle i2c, u8 slave_addr, u8 reg, u16 *data)
{
    I2C_Transaction i2cTransaction;
    bool status = false;
    i2cTransaction.slaveAddress = slave_addr;
    i2cTransaction.writeBuf = &reg;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = data;
    i2cTransaction.readCount = 2;
    status = I2C_transfer(i2c, &i2cTransaction);
    if (status) {
        *data = FLIP_BYTES(*data);
    }
    return !status;
}


static int _ina226_write(I2C_Handle i2c, u8 slave_addr, u8 reg, u16 data)
{
    I2C_Transaction i2cTransaction;
    u8 txdata[3];
    bool status = false;

    txdata[0] = reg;
    txdata[1] = (data & 0xFF00) >> 8;
    txdata[2] = (data & 0xFF);
    i2cTransaction.slaveAddress = slave_addr;
    i2cTransaction.writeBuf = txdata;
    i2cTransaction.writeCount = 3;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    status = I2C_transfer(i2c, &i2cTransaction);
    return !status;
}


int ina226_init(struct ina226_rail *rails, size_t num_rails, I2C_Handle *i2c_bus)
{
    int i;
	u16 data = REG_CONFIG_RESET;
	u16 timeout = 10000;
	u8 op_mode = CONFIG_MODE_CONTINOUS(CONFIG_MODE_V_BUS_VOLT |
	                                   CONFIG_MODE_V_SHUNT_VOLT);
	u16 config = SAFE_SET(NUM_AVERAGES, REG_CONFIG_AVG_MASK);
    config |= SAFE_SET(SHUNT_CONV_TIME, REG_CONFIG_VSH_CT_MASK);
    config |= SAFE_SET(BUS_CONV_TIME, REG_CONFIG_VBUS_CT_MASK);
    config |= SAFE_SET(op_mode, REG_CONFIG_MODE_MASK);

	for (i=0; i < num_rails; i++) {
	    struct ina226_rail *rail = &rails[i];
	    struct reg_ina226 *reg = &rail->reg;
	    int r;
	    float cal_val;
        /* ONLY DO reset here - no other reg access please */
        r = _ina226_write(i2c_bus[rail->i2c_bus_index], rail->i2c_slave_addr, REG_CONFIG, data);
        if (r)
            return r;

        while (data & REG_CONFIG_RESET) {
            /* Usually exits in a single iteration */
            r = _ina226_read(i2c_bus[rail->i2c_bus_index], rail->i2c_slave_addr, REG_CONFIG, &data);
            if (r)
                return r;
            timeout--;
            if (!timeout)
                return -ETIMEDOUT;
        }

        reg->config = config;

        /*
         * Calculate the INA calibration value
         * Formula:
         *     0.00512 (fixed constant)
         * --------------------------------
         * self.current_lsb * self.shunt_r
         */
        cal_val = REG_CAL_VAL_CONST;
        cal_val /= CURRENT_LSB * rail->shunt_resistor_value;

        if (cal_val > (float)REG_CAL_MASK)
            reg->cal = REG_CAL_MASK;
        else
            reg->cal = ((u16) cal_val) & REG_CAL_MASK;

	}

	return 0;
}


int ina226_alloc_data_buffers(struct ina226_rail *rail, int num)
{
    if (rail->data)
        free(rail->data);

    rail->data = malloc(sizeof(*(rail->data)) * num);

    if (!rail->data)
        return -ENOMEM;
    return 0;
}


int ina226_configure(struct ina226_rail *rail, I2C_Handle *i2c_bus)
{
    int r;
    struct reg_ina226 *reg = &rail->reg;

    r = _ina226_write(i2c_bus[rail->i2c_bus_index], rail->i2c_slave_addr, REG_CONFIG, reg->config);
    if (r)
        return r;
    r = _ina226_write(i2c_bus[rail->i2c_bus_index], rail->i2c_slave_addr, REG_CAL, reg->cal);
    if (r)
        return r;

    return 0;
}


int ina226_sample_one(struct ina226_rail *rail, I2C_Handle *i2c_bus)
{
    int r;
    struct reg_ina226 *reg = &rail->reg;

    r = _ina226_read(i2c_bus[rail->i2c_bus_index], rail->i2c_slave_addr, REG_SHUNT, &reg->shunt);
    if (r)
        return r;

    r = _ina226_read(i2c_bus[rail->i2c_bus_index], rail->i2c_slave_addr, REG_BUS, &reg->bus);
    if (r)
        return r;

    r = _ina226_read(i2c_bus[rail->i2c_bus_index], rail->i2c_slave_addr, REG_POWER, &reg->power);
    if (r)
        return r;

    r = _ina226_read(i2c_bus[rail->i2c_bus_index], rail->i2c_slave_addr, REG_CURRENT, &reg->current);

    return r;
}


static s16 convert_to_decimal(u16 x)
{
    u16 m = x >> 16;
    return (~m & x) | (((x & 0x8000) - x) & m);
}


int ina226_process_one(struct ina226_rail *rail, struct power_data_sample *data)
{
    struct reg_ina226 *reg = &rail->reg;

    data->shunt_uV =
        ((float)convert_to_decimal(reg->shunt & REG_SHUNT_MASK)) *
        REG_SHUNT_UV_PER_LSB;

    data->rail_uV = ((float)(reg->bus & REG_BUS_MASK)) * REG_BUS_UV_PER_LSB;

    data->current_mA =
        ((float)convert_to_decimal(reg->current & REG_CURRENT_MASK)) *
        CURRENT_LSB * 1000;

    data->power_mW =
        ((float)(reg->power & REG_POWER_MASK)) * POWER_LSB * 1000;

    return 0;
}
