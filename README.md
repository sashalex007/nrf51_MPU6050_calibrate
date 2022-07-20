# DMP calibration firmware for nrf51 sensor tag

This is a mpu6050 calibration firmware for an nrf51802 based sensor tag. This firmware will compute xyz offsets for both gyro and accelerometer.
https://www.aliexpress.com/item/32859423925.html

Prerequisites 
1. STlink v2, chinese clone will work
2. Keil uVision 5 (MDK)
3. Some jumper wires

### Guide 1 (for flashing firmware.hex file)

We are using Keil uVision for flashing because ST tools will not recognize the SoC.

1. Download the NordicSemiconductor software packs with keil package installer and create a new project
2. Select nrf51802_xxAA as your device
3. Go to target options. In the output tab click "select folder for objects" and browse to the folder containing firmware.hex. Change name of executable to firmware.hex.
4. Staying in target options, select debug tab and then select "ST-link debugger" from the drop down menu. Then select settings
5. Enter 4mhz for the STlink
6. Select flash download tab and then select all the following "erase full chip, program, verify, reset and run"
7. Staying in flash download tab, remove nrf51xxx from programming algorithms and add nrf51xxx external connectivity board.
8. Connect the board to your STlink. Make sure you use 3.3v in the board VCC
9. From the top menu, Flash ==> Download. The flashing should complete successfully.
10. Download Serial Bluetooth Terminal from the play store (android) and connect to your board (should be named "Calibrate")
11. Once connected, send the device the number "1" from the serial console. Calibration should commence, give it 5 minutes to complete. Device should remain as flat as possible for callibration accuracy.

### Guide 2 (build hex file from source)

I am using Keil studio cloud for compiling. It is possible to build offline, but its too time consuming. 

1. Open your Keil studio cloid and import project from this URL (https://github.com/sashalex007/nrf51_MPU6050_calibrate)
2. Select "Seeed Arch BLE" as the target hardware
3. Build. Hex file will start downloading automatically 
4. Follow guide 1 for flashing the hex file


