/*
 * Copyright (c) 2017, Texas Instruments Incorporated
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

#ifndef EVMS_POWER_RAILS_H_
#define EVMS_POWER_RAILS_H_

#include "power_data.h"
#include "ina226.h"

struct ina226_rail dra71x_evm_rails[] = {
    { .name = "vdd_core",     .i2c_bus_index = 0, .i2c_slave_addr = 0x44, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdd_dsp",      .i2c_bus_index = 0, .i2c_slave_addr = 0x45, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdds_1v8",     .i2c_bus_index = 0, .i2c_slave_addr = 0x46, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdd_ddr_1v35", .i2c_bus_index = 0, .i2c_slave_addr = 0x47, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vda_1v8_pll",  .i2c_bus_index = 0, .i2c_slave_addr = 0x48, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vda_1v8_phy",  .i2c_bus_index = 0, .i2c_slave_addr = 0x49, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vddshv8",      .i2c_bus_index = 0, .i2c_slave_addr = 0x4a, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdda_usb3v3",  .i2c_bus_index = 0, .i2c_slave_addr = 0x4b, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vddshv_3v3",   .i2c_bus_index = 0, .i2c_slave_addr = 0x4c, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdd_ddr",      .i2c_bus_index = 1, .i2c_slave_addr = 0x43, .shunt_resistor_value = 0.010, .reg={}}
};

struct ina226_rail dra76x_evm_rails[] = {
    { .name = "vdd_mpu",      .i2c_bus_index = 0, .i2c_slave_addr = 0x40, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdd_gpu",      .i2c_bus_index = 0, .i2c_slave_addr = 0x41, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdd_dspeve",   .i2c_bus_index = 0, .i2c_slave_addr = 0x42, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdd_core",     .i2c_bus_index = 0, .i2c_slave_addr = 0x43, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdd_iva",      .i2c_bus_index = 0, .i2c_slave_addr = 0x44, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vddr",         .i2c_bus_index = 0, .i2c_slave_addr = 0x46, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vddr_soc",     .i2c_bus_index = 0, .i2c_slave_addr = 0x47, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdds_1v8",     .i2c_bus_index = 0, .i2c_slave_addr = 0x49, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdd_sdio",     .i2c_bus_index = 0, .i2c_slave_addr = 0x4a, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vda_usb",      .i2c_bus_index = 0, .i2c_slave_addr = 0x4b, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vda_pll",      .i2c_bus_index = 0, .i2c_slave_addr = 0x4c, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vda_phy2",      .i2c_bus_index = 0, .i2c_slave_addr = 0x4d, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vda_phy1",      .i2c_bus_index = 0, .i2c_slave_addr = 0x4e, .shunt_resistor_value = 0.010, .reg={}}
};

struct ina226_rail am654x_evm_rails[] = {
    { .name = "vdd_core",     .i2c_bus_index = 0, .i2c_slave_addr = 0x40, .shunt_resistor_value = 0.002, .reg={}},
    { .name = "vdd_mcu",      .i2c_bus_index = 0, .i2c_slave_addr = 0x41, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdd_mpu",      .i2c_bus_index = 0, .i2c_slave_addr = 0x42, .shunt_resistor_value = 0.002, .reg={}},
    { .name = "soc_dvdd3v3",  .i2c_bus_index = 0, .i2c_slave_addr = 0x43, .shunt_resistor_value = 0.002, .reg={}},
    { .name = "soc_dvdd1v8",  .i2c_bus_index = 0, .i2c_slave_addr = 0x44, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "soc_avdd1v8",  .i2c_bus_index = 0, .i2c_slave_addr = 0x45, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "soc_vdds_ddr", .i2c_bus_index = 0, .i2c_slave_addr = 0x46, .shunt_resistor_value = 0.010, .reg={}},
    { .name = "vdd_ddr",      .i2c_bus_index = 0, .i2c_slave_addr = 0x47, .shunt_resistor_value = 0.010, .reg={}}
};
#endif /* EVMS_POWER_RAILS_H_ */
