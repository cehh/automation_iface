/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2cmasterexample1.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>

/* Driver Header files */
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#include "Board.h"

/* Application Header files */
#include "dra71x_evm.h"
#include "ina226_reg_defns.h"

static Display_Handle display;
struct ina226_rail *rails = NULL;
I2C_Handle i2c_bus[2];


void reportError(char *message)
{
    Display_printf(display, 0, 0, "ERROR: %s\n", message);
}

/*
 * Define devices available in DUT's PM1 and PM2 buses and
 * initialize them.
 */
void setDutType(char *dut_name)
{
    int result;
    if (strcmp("dra71x-evm", dut_name) == 0) {
        rails = dra71x_evm_rails;
    }

    result = ina226_init(rails, i2c_bus);
    if (result)
        reportError("initializing ina226s")
}

void configureRails(int num)
{
    char message[256];
    for (int i=0; i < ARRAY_SIZE(rails); i++) {
        struct ina226_rail rail = rails[i];
        /* buffers are freed after final data processing */
        ret = ina226_alloc_data_buffers(rail, num);
        if (ret) {
            snprintf(message, 256, "alloc %s failed:%d\n", rail->name, ret);
            reportError(message)
        }

        ret = ina226_configure(rail);
        if (ret) {
            snprintf(message, 256, "configure %s failed:%d\n", rail->name, ret);
            reportError(message)
        }

    }
}


void measurePower(int num)
{
    configureRails(int num);

}

void writeReg(I2C_Handle i2c, u8 slave_addr, u8 reg, u16 data)
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
    if (status) {
        Display_printf(display, 0, 0, "%d written to device:%d register:%d\n", data, slave_addr, reg);
    }
}

void readReg(I2C_Handle i2c, u8 slave_addr, u8 reg, u16 *data)
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
        Display_printf(display, 0, 0, "%d read from device:%d register:%d\n", *data, slave_addr, reg);
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    u16 data;

    /* Call driver init functions */
    Display_init();
    I2C_init();

    /* Open the HOST display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        while (1);
    }

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.bitRate = I2C_100kHz;

    i2c_bus[0] = I2C_open(Board_I2C0, &i2cParams);
    i2c_bus[1] = I2C_open(Board_I2C0, &i2cParams);

    if (i2c_bus[0] == NULL || i2c_bus[1] == NULL) {
        Display_printf(display, 0, 0, "Error Initializing I2C!\n");
    }
    else {
        Display_printf(display, 0, 0, "I2C Initialized!\n");
    }


/*
    float cal_val;
    float max_current_A;
    struct ina226_rail myrail;
    struct ina226_rail *rail = &myrail;
    struct reg_ina226 *reg = &rail->reg;

    writeReg(i2c, 0x44, 0x0, (0x1 << 15));
    rail->shunt_conv_time = 4;
    rail->bus_conv_time = 4;
    rail->shunt_resistor_accuracy = 5;
    rail->num_avgs = 0;
    rail->shunt_resistor_value = 0.010;
    max_current_A = REG_MAX_EXP_CURRENT;
    rail->current_lsb_A = max_current_A / (float) 32768;
    rail->power_lsb = rail->current_lsb_A * REG_POWER_LSB_RATIO;
    rail->op_mode = CONFIG_MODE_CONTINOUS(CONFIG_MODE_V_BUS_VOLT |
                          CONFIG_MODE_V_SHUNT_VOLT);

    reg->config = SAFE_SET(rail->num_avgs, REG_CONFIG_AVG_MASK);
    reg->config |= SAFE_SET(rail->shunt_conv_time, REG_CONFIG_VSH_CT_MASK);
    reg->config |= SAFE_SET(rail->bus_conv_time, REG_CONFIG_VBUS_CT_MASK);
    reg->config |= SAFE_SET(rail->op_mode, REG_CONFIG_MODE_MASK);
    cal_val = REG_CAL_VAL_CONST;
    cal_val /= rail->current_lsb_A * rail->shunt_resistor_value;

    if (cal_val > (float)REG_CAL_MASK)
        reg->cal = REG_CAL_MASK;
    else
        reg->cal = ((u16) cal_val) & REG_CAL_MASK;

    writeReg(i2c, 0x44, 0x0, reg->config);
    writeReg(i2c, 0x44, 0x5, reg->cal);
    for (i = 0x0; i < 7; i++) {
        readReg(i2c, 0x44, i, &data);
    }
    readReg(i2c, 0x44, 0xfe, &data);
    readReg(i2c, 0x44, 0xff, &data);
    Display_printf(display, 0, 0, "Got I2C data!\n");
*/
    /* Deinitialized I2C */
    I2C_close(i2c);
    Display_printf(display, 0, 0, "I2C closed!\n");

    return (0);
}
