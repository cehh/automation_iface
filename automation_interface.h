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

#ifndef AUTOMATION_INTERFACE_H_
#define AUTOMATION_INTERFACE_H_

#define MAX_MESSAGE_LEN 256
#define TABLE_MAX_ROW 15
#define TABLE_MAX_COL 6
#define TABLE_MAX_ELT_LEN 20
#define BUFF_LENGTH (128)

void clearBuffer();
void printHelp();
void printError(char *cmd);
void printMsg(char msg[]);
int getSysbootPin(int pin);
char *trimSpaces(char *str);
int startsWith(char *str, char *substr);
void bootMode(char *mode);
void reportError(char *message);
void setDutType(char *dut_name);
void configureRails(int num);
void measurePower(int num, int duration_ms);
void algo_average_data(int num, int dur_ms);
int autoadjust_table_init();
void Display_table(char table[][][], u16 num_rows);
void Display_printf(char *fmt, ...);
void cleanup();

UART_Handle uart;
I2C_Handle i2c_bus[2];
static char version[] = "automation interface v0.1";
char echoPrompt[] = "=>\r\n";
char max_len_msg[] = "Max string length reached, resetting ....\r\n";
static int rBytes = 0;
static int set_dut = 0;
static char rBuff[BUFF_LENGTH];
char *rbuffp = rBuff;
struct ina226_rail *rails = NULL;
size_t num_rails = 0;

#endif /* AUTOMATION_INTERFACE_H_ */
