## Summary

Application to control TI's EVMs using automation interface header.
Requires TI RTOS.

Power measurement INA226 code based on https://github.com/nmenon/powertool

## Setup

    Parlex-050R40-76B                               Serial
      .5mm 3" cable       [SDmux BoosterPack]       Console
DUT <===============> Automation Iface BoosterPack <=======> Control Host
                         MSP432P401R LaunchPad
  

## Resources & Jumper Settings
Default MSP432P401R settings work, it is not required to set or modify any jumpers.
The application uses following MSP432P401R resources:

Uart Ports:
* Board_UART0: For serial console, connect at 115200 bps

GPIO In ports:
* AUTO_RSTOUT: Soc Reset Detection, currently not used by SW.
* AUTO_GPIO:   General purpose GPIO, currently not used by SW.

GPIO Out ports:
* AUTO_RESET: Force Warm Reset
* AUTO_POR:   Force Cold Reset
* AUTO_POWER: Turn EVM power On/Off 
* AUTO_SYSBOOT0: Set/clear SYSBOOT0 pin
* AUTO_SYSBOOT1: Set/clear SYSBOOT1 pin
* AUTO_SYSBOOT2: Set/clear SYSBOOT2 pin
* AUTO_SYSBOOT3: Set/clear SYSBOOT3 pin
* AUTO_SYSBOOT4: Set/clear SYSBOOT4 pin
* AUTO_SYSBOOT5: Set/clear SYSBOOT5 pin
 Used by optional SDmux boosterpack
 * MUX_L_MICROSD_LED: Turn L_MICROSD LED on/off
 * MUX_R_MICROSD_LED: Turn R_MICROSD LED on/off
 * MUX_SELECT: Select MICROSD side left/right

I2C ports:
* Board_I2C0: Connects to DUT PM2 bus
* Board_I2C1: Connects to DUT PM1 bus

## Software Interface
automation interface v0.1 commands:
 mmc <l-microsd|r-microsd>                      :Connect to left/right uSD card
 auto reset                                     :Warm reset DUT
 auto por                                       :Power on reset DUT
 auto power <on|off>                            :Power on|off DUT
 auto sysboot <setting>                         :e.g. 110000
 auto set dut <DUT type>                        :Initialize i2c for DUT
 auto measure power <iter(<=150)> <delay(ms)>   :Measure DUT power
 version                                        :Show SW version
 help                                           :Print this menu

