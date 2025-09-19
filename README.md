# phyfob
 
## Sensors
- [BMP580](#bmp581) (Pressure, Temperature) [[Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf)]
- [LSM6DSR](#lsm6dsr) (Acceleration, Rotation) [[Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsr.pdf)]
- [HDC1080](#hdc1080) (Temperature, Humidity) [[Datasheet](https://www.ti.com/lit/gpn/hdc1080)]
- [STCC4](#stcc4) (CO₂) [[Datasheet](https://sensirion.com/resource/datasheet/STCC4)]

### BMP581
sensor data | |    
-------------------|-----
uuid          | cddf1013-30f7-4671-8b43-5e40ba53514a
byte 0-3          | pressure (float32LittleEndian) 
byte 4-7          | temperature (float32LittleEndian)
byte 8-11         | timestamp (float32LittleEndian)

Data is repeating every 12 byte.

configuration | |    
-------------------|-----
uuid        | cddf1014-30f7-4671-8b43-5e40ba53514a
byte 0          | enable (bool)
byte 1          | pressure oversampling rate (uint8) 
byte 2          | iir filter (uint8)

oversampling rate | |
-----------------|------
1x | 0x00
2x | 0x01
4x | 0x02
8x | 0x03
16x | 0x04
32x | 0x05
64x | 0x06
128x | 0x07

IIR filter | |
-----------------|------
bypass | 0x00
coefficient 1 | 0x01
coefficient 3 | 0x02
coefficient 7 | 0x03
coefficient 15 | 0x04
coefficient 31 | 0x05
coefficient 63 | 0x06
coefficient 127 | 0x07


### LSM6DSR

rotation rate data | (for float configuration) |    
-------------------|-----
uuid          | cddf1000-30f7-4671-8b43-5e40ba53514a
byte 0-3          | x (float32LittleEndian) 
byte 4-7          | y (float32LittleEndian)
byte 8-11         | z (float32LittleEndian)
byte 12-15         | timestamp (float32LittleEndian)

Data is repeating every 16 byte.

acceleration data | (for float configuration) |    
-------------------|-----
uuid          | cddf1001-30f7-4671-8b43-5e40ba53514a
byte 0-3          | x (float32LittleEndian) 
byte 4-7          | y (float32LittleEndian)
byte 8-11         | z (float32LittleEndian)
byte 12-15         | timestamp (float32LittleEndian)

Data is repeating every 16 byte.


configuration | |    
-------------------|-----
uuid        | cddf1002-30f7-4671-8b43-5e40ba53514a
byte 0          | enable <br/> bit 0 = accelerometer <br/> bit 1 gyroscope <br/> e.g. enable both with 0x03
byte 1          | measurement rate <br/> 12.5Hz = 0x01 <br/> 26Hz = 0x02 <br/> 52Hz = 0x03 <br/> 104Hz = 0x04 <br/> 208Hz = 0x05 <br/> 416Hz = 0x06 <br/> 816Hz = 0x07 <br/> 1666Hz = 0x08 <br/> 3332Hz = 0x09 <br/> 6667Hz = 0x0A
byte 2          | range accelerometer <br/> 2g = 0x00 <br/> 16g = 0x01 <br/> 4g = 0x02 <br/> 8g = 0x03
byte 3          | range gyroscope <br/> 125°/s = 0x02 <br/> 250°/s = 0x00 <br/> 500°/s = 0x04 <br/> 1000°/s = 0x08<br/> 2000°/s = 0x0C<br/> 4000°/s = 0x01
byte 4          | data format <br/> float = 0 <br/> uint16_t = 1
byte 5          | number of measurements per ble package


### HDC1080
sensor data | |    
-------------------|-----
uuid          | cddf1005-30f7-4671-8b43-5e40ba53514a
byte 0-3          | temperature (float32LittleEndian) 
byte 4-7          | humidity (float32LittleEndian)
byte 8-11         | timestamp (float32LittleEndian)

Data is repeating every 12 byte.

configuration | |    
-------------------|-----
uuid        | cddf1006-30f7-4671-8b43-5e40ba53514a
byte 0          | enable (bool)
byte 1          | measurement interval in multiple of 10ms (minimum 30ms)<br/> e.g. 0x04 = 40ms


### STCC4
sensor data | |    
-------------------|-----
uuid          | cddf100b-30f7-4671-8b43-5e40ba53514a
byte 0-3          | CO₂ (float32LittleEndian) 
byte 4-7         | timestamp (float32LittleEndian)

Data is repeating every 12 byte.

configuration | |    
-------------------|-----
uuid        | cddf100c-30f7-4671-8b43-5e40ba53514a
byte 0          | enable (bool)
byte 1          | measurement interval in multiple of 1s (minimum 1s)<br/> e.g. 0x04 = 4s
