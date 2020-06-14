## Arduino3DCube

Port of OneLoneCoder.com - 3D Graphics Part #1 - Triangles & Projections to Arduino

[![Youtube demo](https://img.youtube.com/vi/P0NabuIyvMk/0.jpg)](https://www.youtube.com/watch?v=P0NabuIyvMk)

Code is based on original works by javidx9/onelonecoder:
* https://www.youtube.com/watch?v=ih20l3pJoeU
* https://www.github.com/onelonecoder
* https://www.onelonecoder.com
* https://www.youtube.com/javidx9
Under GNU GPLv3 https://github.com/OneLoneCoder/videos/blob/master/LICENSE
 
 
### Hardware

* Arduino UNO
* SSD1306 128x64 I2C OLED

### Libraries

* U8g2: Library for monochrome displays, version 2 https://github.com/olikraus/u8g2

### Notes

* Most of the OLC code is unchanged; modifications include
* Addition of triangle function drawTri as u8g2's triangle functions are shaded
* mesh (vector) replaced with cube (array of triangles) - vector libraries ate too much space
* Store cube in PROGMEM to provide enough memory for u8g2 to run with full frame buffer (best speed)
* Rotation speed static
* Show FPS

Sketch uses 12740 bytes (39%) of program storage space. Maximum is 32256 bytes.
Global variables use 1777 bytes (86%) of dynamic memory, leaving 271 bytes for local variables. Maximum is 2048 bytes.
Low memory available, stability problems may occur.
