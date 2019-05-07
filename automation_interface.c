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
#include <stdlib.h>
#include <evms_power_rails.h>

/* Driver Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Watchdog.h>
#include <ti/sysbios/knl/Task.h>

/* Example/Board Header files */
#include "Board.h"

/* Application Header files */
#include "ina226_reg_defns.h"
#include "tca6424a.h"
#include "automation_interface.h"

struct Pins_Mapping iface_v1_mappings = {
   .auto_rstout    = J1_8,
   .auto_gpio1     = J4_9,
   .auto_gpio2     = 255,   /* not available on automation iface v1 dra71x/76x */
   .auto_gpio3     = 255,   /* not available on automation iface v1 dra71x/76x */
   .auto_gpio4     = 255,   /* not available on automation iface v1 dra71x/76x */
   .auto_reset     = J1_5,
   .auto_por       = J2_3,
   .auto_power     = J2_2,
   .auto_sysboot0  = J4_1,
   .auto_sysboot1  = J4_2,
   .auto_sysboot2  = J4_3,
   .auto_sysboot3  = J4_4,
   .auto_sysboot4  = J4_7,
   .auto_sysboot5  = J2_8,
   .mux_select     = J4_10,
   .mux_l_microsd_led = 255,
   .mux_r_microsd_led = 255,
   .i2c_power_buses = {Board_I2C1, Board_I2C0},
   .i2c_gpio_buses  = {-1, -1}
};

struct Pins_Mapping iface_v2_mappings = {
    .auto_rstout    = 255,   /* not available on iface v2 am654x */
    .auto_gpio1     = J4_8,
    .auto_gpio2     = J2_9,   /* available since iface v2 am654x */
    .auto_gpio3     = J4_9,   /* available since iface v2 am654x */
    .auto_gpio4     = J2_10,   /* available since iface v2 am654x */
    .auto_reset     = J1_5,
    .auto_por       = J2_3,
    .auto_power     = J2_2,
    .auto_sysboot0  = 255,
    .auto_sysboot1  = 255,
    .auto_sysboot2  = 255,
    .auto_sysboot3  = 255,
    .auto_sysboot4  = 255,
    .auto_sysboot5  = 255,
    .mux_select     = J4_10,
    .mux_l_microsd_led = 255,
    .mux_r_microsd_led = 255,
    .i2c_power_buses = {Board_I2C1, -1},
    .i2c_gpio_buses  = {Board_I2C0, -1}
};

/* J7es mappings */
struct Pins_Mapping iface_v3_mappings = {
    .auto_rstout    = 255,   /* not available on iface v2 am654x */
    .auto_gpio1     = J4_8,
    .auto_gpio2     = J2_9,   /* available since iface v2 am654x */
    .auto_gpio3     = J4_9,   /* available since iface v2 am654x */
    .auto_gpio4     = J2_10,   /* available since iface v2 am654x */
    .auto_reset     = J1_5,
    .auto_por       = J2_3,
    .auto_power     = J2_2,
    .auto_sysboot0  = 255,
    .auto_sysboot1  = 255,
    .auto_sysboot2  = 255,
    .auto_sysboot3  = 255,
    .auto_sysboot4  = 255,
    .auto_sysboot5  = 255,
    .mux_select     = J4_10,
    .mux_l_microsd_led = 255,
    .mux_r_microsd_led = 255,
    .i2c_power_buses = {Board_I2C1, Board_I2C0},
    .i2c_gpio_buses  = {Board_I2C0, -1}
};

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
            return pinsMapping->auto_sysboot0;
        case 1:
            return  pinsMapping->auto_sysboot1;
        case 2:
            return  pinsMapping->auto_sysboot2;
        case 3:
            return  pinsMapping->auto_sysboot3;
        case 4:
            return  pinsMapping->auto_sysboot4;
        case 5:
        default:
            return  pinsMapping->auto_sysboot5;
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

/*
 *  Set boot mode pins via GPIO
 */
void bootModeGpio(char *mode)
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

void lowPulseGpio(short pin)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    Task_sleep(100);
    pinMode(pin, INPUT_PULLUP);
}

void lowPulseKeepHigh(short pin)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    Task_sleep(10);
    digitalWrite(pin, LOW);
    Task_sleep(10);
    digitalWrite(pin, HIGH);
    Task_sleep(100);
}

/*
 *  Set boot mode pins via I2C expander present on some EVMs
 */
void bootModeI2C(char *mode)
{
    printMsg("I2C Boot mode implementation");
    char *p = mode;
    char word[3] = {NULL, NULL, NULL};
    u8 bank = 0;
    u8 out;
    while (*p != NULL)
        p++;
    if (p == mode) {
        printError("Must provide at least one boot mode word");
        return;
    }
    while ((p - mode) > 2) {
        word[1] = *(--p);
        word[0] = *(--p);
        out = (u8)strtol(word, NULL, 16);
        tca6424WriteBank(bank, out, i2c_gpio_bus[0]);
        bank++;
    }
    word[1] = *(--p);
    if (p > mode)
        word[0] = *(--p);
    else {
        word[0] = word[1];
        word[1] = NULL;
    }
    out = (u8)strtol(word, NULL, 16);
    tca6424WriteBank(bank, out, i2c_gpio_bus[0]);
}

void reportError(char *message)
{
    Display_printf("ERROR: %s\n", message);
}

/*
 * Define devices available in DUT's PM1 and PM2/BM buses and
 * initialize them.
 */
void setDutType(char *dut_name)
{
    int result;
    bootMode = &bootModeI2C;
    pinsMapping = &iface_v2_mappings;
    if (strcmp("dra71x-evm", dut_name) == 0) {
        bootMode = &bootModeGpio;
        pinsMapping = &iface_v1_mappings;
        rails = dra71x_evm_rails;
        num_rails = ARRAY_SIZE(dra71x_evm_rails);
    }
    else if (strcmp("dra76x-evm", dut_name) == 0) {
        bootMode = &bootModeGpio;
        pinsMapping = &iface_v1_mappings;
        rails = dra76x_evm_rails;
        num_rails = ARRAY_SIZE(dra76x_evm_rails);
    }
    else if (strcmp("am654x-evm", dut_name) == 0 || strcmp("am654x-idk", dut_name) == 0) {
        rails = am654x_evm_rails;
        num_rails = ARRAY_SIZE(am654x_evm_rails);
    }
    else if (strcmp("j721e-evm", dut_name) == 0 || strcmp("j721e-idk-gw", dut_name) == 0) {
        pinsMapping = &iface_v3_mappings;
        rails = j721e_evm_rails;
        num_rails = ARRAY_SIZE(j721e_evm_rails);
    }
    else {
        set_dut = 0;
        Display_printf("Error: Unsupported DUT %s", dut_name);
    }

    if (set_dut) {
        // Clean I2C connections
        cleanup();
        // Open I2C connections
        if (initDutI2cBuses()) {
            printError("Could not properly open I2C buses");
            set_dut = 0;
            return;
        }
        mapI2cBuses();
        // Init I2C power measurement devices
        if (i2c_power_bus[0]) {
            result = ina226_init(rails, num_rails, i2c_power_bus);
            if (result) {
                set_dut = 0;
                reportError("initializing ina226s");
            }
        }
        // Init I2C I/O expander devices
        lowPulseKeepHigh(pinsMapping->auto_gpio4);  // toggle auto_gpio4 to reset gpio expander
        if (i2c_gpio_bus[0]) {
            if (!tca6424TestConnection(i2c_gpio_bus[0])) {
                set_dut = 0;
                reportError("Connecting to tca6424");
            }
            if (tca6424Init(i2c_gpio_bus[0])) {
                set_dut = 0;
                reportError("Initializing tca6424");
            }
            // enable bootmode
            pinMode(pinsMapping->auto_gpio3, OUTPUT);
            digitalWrite(pinsMapping->auto_gpio3, LOW);
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

        ret = ina226_configure(rail, i2c_power_bus);
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
            ret = ina226_sample_one(&rails[rail_idx], i2c_power_bus);
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
    int index = 0;
    while (i2c_power_bus[index] != NULL) {
        I2C_close(i2c_power_bus[index]);
        index++;
    }
    index=0;
    while (i2c_gpio_bus[index] != NULL) {
        I2C_close(i2c_gpio_bus[index]);
        index++;
    }
    Display_printf("I2C closed!\n");
}

void digitalWrite(uint_least8_t index, unsigned int value)
{
    if (index == 255)
        return;
    GPIO_PinConfig pinConfig;
    GPIO_getConfig(index, &pinConfig);
    GPIO_write(index, value);
}

void pinMode(uint_least8_t index, GPIO_PinConfig mode)
{
    if (index == 255)
        return;
    GPIO_PinConfig pinConfig;
    GPIO_getConfig(index, &pinConfig);
    pinConfig &= ~GPIO_INOUT_MASK;
    pinConfig |= (mode & GPIO_INOUT_MASK);
    GPIO_setConfig(index, pinConfig);
}


int initDutI2cBuses()
{
    I2C_Params      i2cParams;
    int index;
    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.bitRate = I2C_100kHz;

    for(index = 0; index < MSP_EXP432P401R_I2CCOUNT; index++)
    {
        i2c_bus[index] = I2C_open(index, &i2cParams);
        if (i2c_bus[index] == NULL) {
            Display_printf("Error Initializing I2C %i\n", index);
            return 1;
        }
    }
    return 0;
}


void mapI2cBuses()
{
    /* Map i2c power measurement bus if any */
    int index;
    for(index = 0; pinsMapping->i2c_power_buses[index] != -1; index++)
        i2c_power_bus[index] = i2c_bus[pinsMapping->i2c_power_buses[index]];

    /* Map i2c gpio bus if any */
    for(index = 0; pinsMapping->i2c_gpio_buses[index] != -1; index++)
        i2c_gpio_bus[index] = i2c_bus[pinsMapping->i2c_gpio_buses[index]];

    Display_printf("Done initializing I2C buses!");
}

/*
 *  ======== watchdogCallback ========
 */
void watchdogCallback(uintptr_t watchdogHandle)
{
    /*
     * If the Watchdog Non-Maskable Interrupt (NMI) is called,
     * loop until the device resets. Some devices will invoke
     * this callback upon watchdog expiration while others will
     * reset. See the device specific watchdog driver documentation
     * for your device.
     */
    while (1) {}
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    UART_Params     uartParams;
    unsigned int    i;
    Watchdog_Handle watchdogHandle;
    Watchdog_Params wdParams;

    /* Call driver init functions */
    I2C_init();
    GPIO_init();
    UART_init();
    Watchdog_init();

    /* Initialize Watchdog to reset board if not pet */
    Watchdog_Params_init(&wdParams);
    wdParams.callbackFxn = (Watchdog_Callback) watchdogCallback;
    wdParams.debugStallMode = Watchdog_DEBUG_STALL_ON;
    wdParams.resetMode = Watchdog_RESET_ON;

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

    printHelp();

    pinsMapping = &iface_v2_mappings;

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

        /* Open a Watchdog driver instance with timeout of 11 seconds, timeout defined at WatchdogMSP432_HWAttrs */
        watchdogHandle = Watchdog_open(Board_WATCHDOG0, &wdParams);
        if (watchdogHandle == NULL) {
            /* Error opening Watchdog */
            while (1) {}
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
                    digitalWrite(pinsMapping->mux_select, HIGH);
                    digitalWrite(pinsMapping->mux_r_microsd_led, LOW);
                    digitalWrite(pinsMapping->mux_l_microsd_led, HIGH);
                }
                else if (startsWith(rbuffp, "r-microsd")) {
                    printMsg("Using Right microSD connection");
                    digitalWrite(pinsMapping->mux_select, LOW);
                    digitalWrite(pinsMapping->mux_r_microsd_led, HIGH);
                    digitalWrite(pinsMapping->mux_l_microsd_led, LOW);
                }
                else printError(rBuff);
            }

            else if (startsWith(rbuffp, "auto ")) {
                rbuffp += 5;
                rbuffp = trimSpaces(rbuffp);
                if (startsWith(rbuffp, "reset")) {
                    printMsg("Resetting DUT");
                    lowPulseGpio(pinsMapping->auto_reset);
                }
                else if (startsWith(rbuffp, "por")) {
                    printMsg("Power-On-Reset on DUT");
                    lowPulseGpio(pinsMapping->auto_por);
                }
                else if (startsWith(rbuffp, "power ")) {
                    rbuffp += 6;
                    rbuffp = trimSpaces(rbuffp);
                    if (startsWith(rbuffp, "off")) {
                        printMsg("Powering Off DUT");
                        digitalWrite(pinsMapping->auto_power, HIGH);
                    } else if(startsWith(rbuffp, "on")) {
                        printMsg("Powering On DUT");
                        digitalWrite(pinsMapping->auto_power, LOW);
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
        if (watchdogHandle != NULL) {
            Watchdog_close(watchdogHandle);
        }
        Task_sleep(10);
    }

}
