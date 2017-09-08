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
#ifndef __INA226__
#define __INA226__

#include <proj-types.h>
#include "power_data.h"
#include <ti/drivers/I2C.h>

#define INA226_MAX_STRING	30

/* Just the registers of the chip */
struct reg_ina226 {
	u16 config;
	u16 shunt;
	u16 bus;
	u16 power;
	u16 current;
	u16 cal;
	u16 irq_enable;
	u16 die_id;
};

/**
 * struct ina226_rail - rail description that this INA is measuring
 * @name: name of the voltage rail (identifier)
 * @i2c_bus_index: i2c slave address
 * @i2c_slave_addr:	i2c slave address
 * @shunt_resistor_value: Shunt resistor value in OHms
 */
struct ina226_rail {
    char  name[INA226_MAX_STRING];
    u8	  i2c_bus_index;
    u8	  i2c_slave_addr;
	float shunt_resistor_value;

	/* internal */
	struct reg_ina226 reg;
	struct power_data_sample *data;
};

int ina226_init(struct ina226_rail *rail, I2C_Handle *i2c_bus);






int ina226_detect(struct ina226_rail *rail);

int ina226_configure(struct ina226_rail *rail);
int ina226_sample_one(struct ina226_rail *rail);
int ina226_process_one(struct ina226_rail *rail, struct power_data_sample *data);

int ina226_parse_config(struct ina226_rail *rail,
				char *param,
				void *data);

struct ina226_rail *ina226_find(
		struct ina226_rail *root_rail,
		const char *rail_name);
struct ina226_rail *ina226_find_or_allocate(
		struct ina226_rail **root_rail,
		const char *rail_name);
int ina226_alloc_data_buffers(struct ina226_rail *rail, int num);
void ina226_free(struct ina226_rail *rail);

struct pm_bus;
int ina226_bus_init_i2c(struct pm_bus *bus);
void ina226_bus_deinit_i2c(struct pm_bus *node);
int ina226_bus_setup(struct ina226_rail *rail);
#endif	/* __INA226__ */
