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
#include <ctype.h>

/* Driver Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/sysbios/knl/Task.h>

/* Example/Board Header files */
#include "Board.h"

/* Application Header files */
#include "dra71x_evm.h"
#include "ina226_reg_defns.h"
#include "automation_interface.h"


void clearBuffer()
{
    int i;
    printMsg(echoPrompt);
    for(i = 0; i < BUFF_LENGTH; i++)rBuff[i] = 0;
    rBytes = 0;
    rbuffp = rBuff;
}

void printHelp()
{
  printMsg(" mmc <l-microsd|r-microsd>\t\t\t:Connect to left/right uSD card");
  printMsg(" auto reset\t\t\t\t\t:Warm reset DUT");
  printMsg(" auto por\t\t\t\t\t:Power on reset DUT");
  printMsg(" auto power <on|off>\t\t\t\t:Power on|off DUT");
  printMsg(" auto sysboot <setting>\t\t\t\t:e.g. 110000");
  printMsg(" auto set dut <DUT type>\t\t\t:Initialize i2c for DUT");
  printMsg(" auto measure power <iter(<=150)> <delay(ms)>   :Measure DUT power");
  printMsg(" version\t\t\t\t\t:Show SW version");
  printMsg(" help\t\t\t\t\t\t:Print this menu");
}

void printError(char *cmd)
{
  char msg[BUFF_LENGTH] = {0};
  strcat(msg, "Command ");
  strcat(msg, cmd);
  strcat(msg, " not valid ...");
  printMsg(msg);
  printHelp();
}

void printMsg(char *msg)
{
    UART_write(uart, msg, strlen(msg));
    UART_write(uart, "\r\n", 2);
}

int getSysbootPin(int pin) {
    switch(pin) {
        case 0:
            return AUTO_SYSBOOT0;
        case 1:
            return AUTO_SYSBOOT1;
        case 2:
            return AUTO_SYSBOOT2;
        case 3:
            return AUTO_SYSBOOT3;
        case 4:
            return AUTO_SYSBOOT4;
        case 5:
        default:
            return AUTO_SYSBOOT5;
    }
}

char *trimSpaces(char *str)
{
  char *end;

  // Trim leading space
  while(isspace((unsigned char)*str)) str++;

  if(*str == 0)  // All spaces?
    return str;

  // Trim trailing space
  end = str + strlen(str) - 1;
  while(end > str && isspace((unsigned char)*end)) end--;

  // Write new null terminator
  *(end+1) = 0;

  return str;
}

int startsWith(char *str, char *substr)
{
    size_t sub_len = strlen(substr);
    return (strncmp(str, substr, sub_len) == 0);
}

void bootMode(char *mode)
{
    int i;
    for (i=0; i < 6; i++) {
        if ( *(mode + i) == '1' ) {
            digitalWrite(getSysbootPin(5-i), HIGH);
        } else {
            digitalWrite(getSysbootPin(5-i), LOW);
        }
    }
}

void reportError(char *message)
{
    Display_printf("ERROR: %s\n", message);
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
        num_rails = ARRAY_SIZE(dra71x_evm_rails);
    }
    else {
        set_dut = 0;
        Display_printf("Error: Unsupported DUT %s", dut_name);
    }

    if (set_dut) {
        result = ina226_init(rails, num_rails, i2c_bus);
        if (result) {
            set_dut = 0;
            reportError("initializing ina226s");
        }
    }
}

void configureRails(int num)
{
    int i, ret;
    char message[MAX_MESSAGE_LEN];
    for (i=0; i < num_rails; i++) {
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
        for (rail_idx=0; rail_idx < num_rails; rail_idx++) {
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

void algo_average_data(int num, int dur_ms)
{
    char table[TABLE_MAX_ROW][TABLE_MAX_COL][TABLE_MAX_ELT_LEN];
    //char table[2][TABLE_MAX_COL][TABLE_MAX_ELT_LEN];
    struct ina226_rail *rail;
    int i, row, rail_idx;
    float current_summary_bus=0;

    Display_printf("\nAverage Data Start\n");
    current_summary_bus = 0;
    autoadjust_table_init(table);
    Display_printf( "AVG(Samples=%d, Interval=%d ms)", num, dur_ms);
    row = 0;
    strncpy(table[row][0], "Index", TABLE_MAX_ELT_LEN);
    strncpy(table[row][1], "Rail Name", TABLE_MAX_ELT_LEN);
    strncpy(table[row][2], "Shunt voltage(uV)", TABLE_MAX_ELT_LEN);
    strncpy(table[row][3], "Rail voltage(V)", TABLE_MAX_ELT_LEN);
    strncpy(table[row][4], "Current(mA)", TABLE_MAX_ELT_LEN);
    strncpy(table[row][5], "Power(mW)", TABLE_MAX_ELT_LEN);
    row++;
    for (rail_idx=0; rail_idx < num_rails; rail_idx++) {
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

void Display_printf(char *fmt, ...)
{
    char msg[BUFF_LENGTH] = {0};
    char *msgp = msg;
    int ret;
    va_list va;
    va_start(va, fmt);
    ret = vsnprintf(msgp, BUFF_LENGTH, fmt, va);
    if (ret > 0)
        printMsg(msgp);
}

void Display_table(
        char table[TABLE_MAX_ROW][TABLE_MAX_COL][TABLE_MAX_ELT_LEN],
        u16 num_rows)
{
    int row;
    for (row=0; row < num_rows; row++) {
        Display_printf("| %s | %s | %s | %s | %s | %s |",
                       table[row][0], table[row][1], table[row][2],
                       table[row][3], table[row][4], table[row][5]);
    }
}

void cleanup()
{
    /* Deinitialized I2C */
    I2C_close(i2c_bus[0]);
    I2C_close(i2c_bus[1]);
    Display_printf("I2C closed!\n");
}

void digitalWrite(uint_least8_t index, unsigned int value)
{
    GPIO_PinConfig pinConfig;
    GPIO_getConfig(index, &pinConfig);
    GPIO_write(index, value);
}

void pinMode(uint_least8_t index, GPIO_PinConfig mode)
{
    GPIO_PinConfig pinConfig;
    GPIO_getConfig(index, &pinConfig);
    pinConfig &= ~GPIO_INOUT_MASK;
    pinConfig |= (mode & GPIO_INOUT_MASK);
    GPIO_setConfig(index, pinConfig);
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    UART_Params uartParams;
    I2C_Params      i2cParams;
    unsigned int i;

    /* Call driver init functions */
    I2C_init();
    GPIO_init();
    UART_init();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    clearBuffer();
    printMsg(version);

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.bitRate = I2C_100kHz;

    i2c_bus[0] = I2C_open(Board_I2C1, &i2cParams);
    i2c_bus[1] = I2C_open(Board_I2C0, &i2cParams);

    if (i2c_bus[0] == NULL || i2c_bus[1] == NULL) {
        Display_printf("Error Initializing I2C!");
    }
    else {
        Display_printf("I2C Initialized!");
    }

    printHelp();

    /* Loop forever echoing */
    while (1) {
        while (1) {
            UART_read(uart, rbuffp + rBytes, 1);
            UART_write(uart, rbuffp + rBytes, 1);

            if (rBytes == BUFF_LENGTH) {
                printMsg(max_len_msg);
                clearBuffer();

            } else if(rBuff[rBytes] == 13 || rBuff[rBytes] == 10) {
                printMsg("");
                break;

            } else if(rBytes > 0 && (rBuff[rBytes] == 8 || rBuff[rBytes] == 127)) {
                rBuff[rBytes] = 0;
                rBytes -= 1;

            } else if(rBuff[rBytes] == 27 || rBuff[rBytes] == 3) {
                printMsg("^C");
                clearBuffer();
            } else {
                rBytes += 1;
                Task_sleep(10);
            }
        }

        if (rBytes >= 0) {
            rBytes = 0;
            rbuffp = trimSpaces(rbuffp);
            for (i=0; *(rbuffp + i); ++i) *(rbuffp + i) = tolower((int) *(rbuffp + i)); // toLowerCase

            if (startsWith(rbuffp, "mmc ")) {
                rbuffp += 4;
                rbuffp = trimSpaces(rbuffp);
                if (startsWith(rbuffp, "l-microsd")) {
                    printMsg("Using Left microSD connection");
                    digitalWrite(MUX_SELECT, HIGH);
                    digitalWrite(MUX_R_MICROSD_LED, LOW);
                    digitalWrite(MUX_L_MICROSD_LED, HIGH);
                }
                else if (startsWith(rbuffp, "r-microsd")) {
                    printMsg("Using Right microSD connection");
                    digitalWrite(MUX_SELECT, LOW);
                    digitalWrite(MUX_R_MICROSD_LED, HIGH);
                    digitalWrite(MUX_L_MICROSD_LED, LOW);
                }
                else printError(rBuff);
            }

            else if (startsWith(rbuffp, "auto ")) {
                rbuffp += 5;
                rbuffp = trimSpaces(rbuffp);
                if (startsWith(rbuffp, "reset")) {
                    printMsg("Resetting DUT");
                    pinMode(AUTO_RESET, OUTPUT);
                    digitalWrite(AUTO_RESET, LOW);
                    Task_sleep(100);
                    pinMode(AUTO_RESET, INPUT_PULLUP);
                }
                else if (startsWith(rbuffp, "por")) {
                    printMsg("Power-On-Reset on DUT");
                    pinMode(AUTO_POR, OUTPUT);
                    digitalWrite(AUTO_POR, LOW);
                    Task_sleep(100);
                    pinMode(AUTO_POR, INPUT_PULLUP);
                }
                else if (startsWith(rbuffp, "power ")) {
                    rbuffp += 6;
                    rbuffp = trimSpaces(rbuffp);
                    if (startsWith(rbuffp, "off")) {
                        printMsg("Powering Off DUT");
                        digitalWrite(AUTO_POWER, HIGH);
                    } else if(startsWith(rbuffp, "on")) {
                        printMsg("Powering On DUT");
                        digitalWrite(AUTO_POWER, LOW);
                    }
                    else {
                        printError(rBuff);
                    }
                }
                else if (startsWith(rbuffp, "sysboot ")) {
                    rbuffp += 8;
                    rbuffp = trimSpaces(rbuffp);
                    bootMode(rbuffp);
                    printMsg("Sysboot set on DUT");
                }
                else if (startsWith(rbuffp, "set dut ")) {
                    rbuffp += 8;
                    rbuffp = trimSpaces(rbuffp);
                    printMsg("Setting DUT");
                    set_dut = 1;
                    setDutType(rbuffp);
                }
                  else if(startsWith(rbuffp, "measure power ")) {
                      int samples, delay;
                      if (set_dut == 0) {
                          printMsg("auto set dut <DUT type> must be called first");
                      } else {
                          printMsg("Measuring power");
                          rbuffp += 14;
                          rbuffp = trimSpaces(rbuffp);
                          if (sscanf(rbuffp, "%d %d", &samples, &delay) != 2)
                              printError(rBuff);
                          measurePower(samples, delay);
                      }
                  }
                  else {
                      printError(rBuff);
                  }
            }

            else if(startsWith(rbuffp, "help")) {
              printHelp();
            }

            else if(startsWith(rbuffp, "version")) {
                printMsg(version);
            }

            else {
              printError(rBuff);
            }
            clearBuffer();
          }
        Task_sleep(10);
    }

}
