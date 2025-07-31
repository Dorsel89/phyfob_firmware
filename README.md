# phyfob
 
## Sensors
- [BMP580](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf)
- LSM6DSR
- HDC1080
- STCC4

### BMP581
data characteristic | |    
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
