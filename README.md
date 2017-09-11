## Example Summary

Application to control TI's EVMs using automation interface header.
Requires TI RTOS.
INA226 code heavily based on https://github.com/nmenon/powertool

## Peripherals Exercised

* This example requires 2 MSP432P401R LaunchPads. MASTER (i2cmasterexample1.c)
is run on one board and SLAVE (i2cslaveexample1.c) is run on the other

* `Board_I2C_TMP` - I2C instance used at the master end

## Resources & Jumper Settings

