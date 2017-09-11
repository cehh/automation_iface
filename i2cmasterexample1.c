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


#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/sysbios/knl/Task.h>

/* Example/Board Header files */
#include "Board.h"

/* Application Header files */
#include "dra71x_evm.h"
#include "ina226_reg_defns.h"

/* Main application Definitions */
#define MAX_MESSAGE_LEN 256
#define TABLE_MAX_ROW 100
#define TABLE_MAX_COL 10
#define TABLE_MAX_ELT_LEN 50

static Display_Handle display;
struct ina226_rail *rails = NULL;
I2C_Handle i2c_bus[2];
static void algo_average_data(int num, int dur_ms);
int autoadjust_table_init();
void Display_table(char table[][][], u16 num_rows);

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
        reportError("initializing ina226s");
}

void configureRails(int num)
{
    int i, ret;
    char message[MAX_MESSAGE_LEN];
    for (i=0; i < ARRAY_SIZE(rails); i++) {
        struct ina226_rail *rail = &rails[i];
        /* buffers are freed after final data processing */
        ret = ina226_alloc_data_buffers(rail, num);
        if (ret) {
            snprintf(message, MAX_MESSAGE_LEN, "allocating %s failed:%d\n", rail->name, ret);
            reportError(message);
        }

        ret = ina226_configure(rail, i2c_bus);
        if (ret) {
            snprintf(message, MAX_MESSAGE_LEN, "configuring %s failed:%d\n", rail->name, ret);
            reportError(message);
        }

    }
}


void measurePower(int num, int duration_ms)
{
    int sample, rail_idx, ret;
    char message[MAX_MESSAGE_LEN];
    configureRails(num);
    for (sample=0; sample < num; sample++) {
        for (rail_idx=0; rail_idx < ARRAY_SIZE(rails); rail_idx++) {
            ret = ina226_sample_one(&rails[rail_idx], i2c_bus);
            if (ret) {
                snprintf(message, MAX_MESSAGE_LEN, "sampling %s failed:%d\n", rails[rail_idx].name, ret);
                reportError(message);
            }
            ina226_process_one(&rails[rail_idx], &(rails[rail_idx].data[sample]));
        }
        Task_sleep(duration_ms);
    }
    algo_average_data(num, duration_ms);
}

static void algo_average_data(int num, int dur_ms)
{
    char table[TABLE_MAX_ROW][TABLE_MAX_COL][TABLE_MAX_ELT_LEN];
    struct ina226_rail *rail;
    int i, row, rail_idx;
    float current_summary_bus;

    Display_printf(display, 0, 0, "\nAverage Data Start\n");
    current_summary_bus = 0;
    autoadjust_table_init(table);
    Display_printf(display, 0, 0,  "AVG(Samples=%d, Interval=%d ms)", num, dur_ms);
    row = 0;
    strncpy(table[row][0], "Index", TABLE_MAX_ELT_LEN);
    strncpy(table[row][1], "Rail Name", TABLE_MAX_ELT_LEN);
    strncpy(table[row][2], "Shunt voltage(uV)", TABLE_MAX_ELT_LEN);
    strncpy(table[row][3], "Rail voltage(V)", TABLE_MAX_ELT_LEN);
    strncpy(table[row][4], "Current(mA)", TABLE_MAX_ELT_LEN);
    strncpy(table[row][5], "Power(mW)", TABLE_MAX_ELT_LEN);
    row++;
    for (rail_idx=0; rail_idx < ARRAY_SIZE(rails); rail_idx++) {
            rail = &rails[rail_idx];

            struct power_data_sample *data = rail->data;
            struct power_data_sample avg_sample = { 0 };

            for (i = 0; i < num; i++, data++) {
                avg_sample.shunt_uV =
                    ((avg_sample.shunt_uV * i) +
                     data->shunt_uV) / (i + 1);
                avg_sample.rail_uV =
                    ((avg_sample.rail_uV * i) +
                     data->rail_uV) / (i + 1);
                avg_sample.current_mA =
                    ((avg_sample.current_mA * i) +
                     data->current_mA) / (i + 1);
                avg_sample.power_mW =
                    ((avg_sample.power_mW * i) +
                     data->power_mW) / (i + 1);
            }
            snprintf(table[row][0], TABLE_MAX_ELT_LEN, "%d", rail_idx);
            snprintf(table[row][1], TABLE_MAX_ELT_LEN, "%s",
                 rail->name);
            snprintf(table[row][2], TABLE_MAX_ELT_LEN, "%3.2f",
                 avg_sample.shunt_uV);
            snprintf(table[row][3], TABLE_MAX_ELT_LEN, "%3.6f",
                 avg_sample.rail_uV / 1000000);
            snprintf(table[row][4], TABLE_MAX_ELT_LEN, "%3.2f",
                 avg_sample.current_mA);
            snprintf(table[row][5], TABLE_MAX_ELT_LEN, "%3.2f",
                 avg_sample.power_mW);
            row++;

            current_summary_bus += avg_sample.power_mW;
    }

        snprintf(table[row][0], TABLE_MAX_ELT_LEN, "---");
        snprintf(table[row][1], TABLE_MAX_ELT_LEN, "---");
        snprintf(table[row][2], TABLE_MAX_ELT_LEN, "---");
        snprintf(table[row][3], TABLE_MAX_ELT_LEN, "---");
        snprintf(table[row][4], TABLE_MAX_ELT_LEN, "---");
        snprintf(table[row][5], TABLE_MAX_ELT_LEN, "---");
        row++;
        snprintf(table[row][0], TABLE_MAX_ELT_LEN, "Total");
        snprintf(table[row][5], TABLE_MAX_ELT_LEN, "%3.2f",
             current_summary_bus);
        row++;
        Display_table(table, row);
    return;
}

int autoadjust_table_init(
    char table[TABLE_MAX_ROW][TABLE_MAX_COL][TABLE_MAX_ELT_LEN])
{
    unsigned int col, row;

    if (table == NULL) {
        printf("autoadjust_table_init() error: table == NULL!\n");
        return -1;
    }

    for (row = 0; row < TABLE_MAX_ROW; row++)
        for (col = 0; col < TABLE_MAX_COL; col++)
            table[row][col][0] = '\0';

    return 0;
}

void Display_table(
        char table[TABLE_MAX_ROW][TABLE_MAX_COL][TABLE_MAX_ELT_LEN],
        u16 num_rows)
{
    int row;
    for (row=0; row < num_rows; row++) {
        Display_printf(display, 0, 0, "| %s | %s | %s | %s | %s | %s |",
                       table[row][0], table[row][1], table[row][2],
                       table[row][3], table[row][4], table[row][5]);
    }
}
/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    I2C_Params      i2cParams;

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
    i2c_bus[1] = I2C_open(Board_I2C1, &i2cParams);

    if (i2c_bus[0] == NULL || i2c_bus[1] == NULL) {
        Display_printf(display, 0, 0, "Error Initializing I2C!\n");
    }
    else {
        Display_printf(display, 0, 0, "I2C Initialized!\n");
    }

    setDutType("dra71x-evm");
    measurePower(10, 10);


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
    I2C_close(i2c_bus[0]);
    I2C_close(i2c_bus[1]);
    Display_printf(display, 0, 0, "I2C closed!\n");

    return (0);
}
