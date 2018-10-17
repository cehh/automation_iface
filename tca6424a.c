/*
 * TCA6424 Low-Voltage 24-bit I2C I/O Expander
 * For more information see http://www.ti.com/lit/ds/symlink/tca6424a.pdf
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *  Carlos Hernandez
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
*/

#include "tca6424a.h"

/* Driver Header files */
#include <ti/drivers/I2C.h>

static int _tca6424aReadBytes(u8 slave_addr, u8 reg, u8 *data, size_t num_bytes, I2C_Handle i2c)
{
    I2C_Transaction i2cTransaction;
    bool status = false;
    i2cTransaction.slaveAddress = slave_addr;
    i2cTransaction.writeBuf = &reg;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = data;
    i2cTransaction.readCount = num_bytes;
    i2cTransaction.arg = NULL;
    i2cTransaction.nextPtr = NULL;
    status = I2C_transfer(i2c, &i2cTransaction);
    return !status; // So that 0 is returned on success like in Linux
}


static int _tca6424aWriteBytes(u8 slave_addr, u8 reg, u8 *data, size_t num_bytes, I2C_Handle i2c)
{
    I2C_Transaction i2cTransaction;
    u8 txdata[num_bytes+1];
    bool status = false;
    txdata[0] = reg;
    memcpy(txdata + 1, data, num_bytes);
    i2cTransaction.slaveAddress = slave_addr;
    i2cTransaction.writeBuf = txdata;
    i2cTransaction.writeCount = num_bytes+1;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;
    i2cTransaction.arg = NULL;
    i2cTransaction.nextPtr = NULL;
    status = I2C_transfer(i2c, &i2cTransaction);
    return !status;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool tca6424TestConnection(I2C_Handle i2c) {
    u8 tca6424Buffer[3];
    return _tca6424aReadBytes(TCA6424A_DEFAULT_ADDRESS, TCA6424A_RA_INPUT0, tca6424Buffer, 3, i2c) == 0;
}


/** Get all pin logic levels from one bank.
 * @param bank Which bank to read (0/1/2 for P0*, P1*, P2* respectively)
 * @param data Pointer to place to save read data at
 * @param i2c Handler to I2C master
 * @return 0 on Sucess, 1 on Error
 */
u8 tca6424ReadBank(u8 bank, u8 *data, I2C_Handle i2c) {
    return _tca6424aReadBytes(TCA6424A_DEFAULT_ADDRESS, TCA6424A_RA_INPUT0 + bank, data, 1, i2c);
}


/** Get all pin logic levels from all banks.
 * Reads into single 3-byte data container.
 * @param data Pointer to place to save read data at
 * @param i2c Handler to I2C master
 * @return 0 on Sucess, 1 on Error
 */
u8 tca6424ReadAll(u8 *data, I2C_Handle i2c) {
    return _tca6424aReadBytes(TCA6424A_DEFAULT_ADDRESS, TCA6424A_RA_INPUT0| TCA6424A_AUTO_INCREMENT, data, 3, i2c);
}


/** Set all OUTPUT pins' logic levels in one bank.
 * @param bank Which bank to write (0/1/2 for P0*, P1*, P2* respectively)
 * @param value New pins' output logic level (0 or 1 for each pin)
 * @param i2c Handler to I2C master
 * @return 0 on Sucess, 1 on Error
 */
u8 tca6424WriteBank(u8 bank, u8 value, I2C_Handle i2c) {
    return _tca6424aWriteBytes(TCA6424A_DEFAULT_ADDRESS, TCA6424A_RA_OUTPUT0 + bank, &value, 1, i2c);
}


/** Set all OUTPUT pins' logic levels in all banks.
 * @param values All pins' new logic values (P00-P27) in 3-byte array
 * @param i2c Handler to I2C master
 * @return 0 on Sucess, 1 on Error
 */
u8 tca6424WriteAll(u8 *values, I2C_Handle i2c) {
    return _tca6424aWriteBytes(TCA6424A_DEFAULT_ADDRESS, TCA6424A_RA_OUTPUT0 | TCA6424A_AUTO_INCREMENT, values, 3, i2c);
}


/** Set all pin direction (I/O) settings in one bank.
 * @param bank Which bank to read (0/1/2 for P0*, P1*, P2* respectively)
 * @param direction New pins' direction settings (0:output|1:input for each pin)
 * @param i2c Handler to I2C master
 * @return 0 on Sucess, 1 on Error
 */
u8 tca6424SetBankDirection(u8 bank, u8 direction, I2C_Handle i2c) {
    return _tca6424aWriteBytes(TCA6424A_DEFAULT_ADDRESS, TCA6424A_RA_CONFIG0 + bank, &direction, 1, i2c);
}


/** Set all pin direction (I/O) settings in all banks.
 * @param direction All pins' new direction values (P00-P27) in 3-byte array
 * @param i2c Handler to I2C master
 * @return 0 on Sucess, 1 on Error
 */
u8 tca6424SetAllDirection(u8 *direction, I2C_Handle i2c) {
    return _tca6424aWriteBytes(TCA6424A_DEFAULT_ADDRESS, TCA6424A_RA_CONFIG0 | TCA6424A_AUTO_INCREMENT, direction, 3, i2c);
}


/** Initialize TCA6424a for default usage
 * @param i2c Handler to I2C master
 * @return 0 on Sucess, 1 on Error
 */
u8 tca6424Init(I2C_Handle i2c) {
    u8 result;
    u8 data[3] = { 0, 0, 0};
    result = _tca6424aWriteBytes(TCA6424A_DEFAULT_ADDRESS, TCA6424A_RA_POLARITY0 | TCA6424A_AUTO_INCREMENT, data, 3, i2c);
    result |= _tca6424aWriteBytes(TCA6424A_DEFAULT_ADDRESS, TCA6424A_RA_CONFIG0 | TCA6424A_AUTO_INCREMENT, data, 3, i2c);
    result |= _tca6424aWriteBytes(TCA6424A_DEFAULT_ADDRESS, TCA6424A_RA_OUTPUT0 | TCA6424A_AUTO_INCREMENT, data, 3, i2c);
    return result;
}


