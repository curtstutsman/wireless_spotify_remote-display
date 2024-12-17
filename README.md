Wireless Spotify Remote Control and Display
Curt Stutsman & Nicholas Watanabe
https://github.com/curtstutsman/wireless_spotify_remote-display

Introduction
	Our group elected to implement a wireless Spotify remote control and album art display utilizing Spotify’s API to control the listening state of the user. Some features of the project include a rotary encoder (which doubled as a push button) and a 64x64 LED matrix. The rotary encoder was purposed for all input, allowing the user to adjust volume with rotations and toggle play/pause and skip to next or previous songs with single, double, and triple taps. The LED matrix was the primary output of the system, displaying in real time the album art of whatever song was currently playing. To achieve wireless communication with Spotify’s api, our group employed and ESP32-WROOM-32E wireless module. The rest of the functionality (LED driving, GPIO interface, etc.) was implemented on an STM32F103.

Figure 1: System Block Diagram

 	
Hardware Overview
	Our project did not require an extensive list of hardware components to achieve the desired functionality. For the LED matrix, we used a 64x64 panel purchased from adafruit with a 2.5mm pitch. Each LED held a one bit value for each of red, green, and blue channels. The matrix as a whole was controlled via the HUB75 protocol, a standard for these types of LED matrices that utilized 16 different signals in the case of our matrix. The 16 signals of the HUB75 protocol were connected and controlled from the GPIO pins of the STM32. The rotary encoder was also controlled via GPIO pins on the MCU. We used a simple UART connection between the STM32 and the ESP32 to send commands to the ESP and to receive image data into the STM. A few other trivial components were included in the final PCB design such as reset buttons, headers for programming, and decoupling capacitors. The full schematic and PCB layout is given below and is linked on our github repository.

Figure 2: Schematic

Figure 3: PCB Layout

Software Overview
	Because we utilized two different chips in our project, the software was split between the STM32F103, which handled LED driving and input control, and the ESP32, which handled communication with the Spotify API and JPEG Decoding. All code can be found in the main.c file of the STM32 source code and the esp32_sketch.ino file of the ESP32 source code, both available on the github repo.

	The first functionality implemented was the button control. The rotary encoder consists of three signals, two for rotation encoding and one for the button. All three channels were connected to GPIO pins and configured to interrupt the CPU at the rising edge of the signal. Pushes to the button increase the tap count variable. The main loop would then check if tap count was greater than zero and enough time had passed since the last tap (to account for double and triple taps) and then send the corresponding command over UART to the ESP32. Interrupts for rotation of the encoder worked in a similar fashion, except a comparison had to be done between the two rotation signals to determine if the rotation was clockwise or counter clockwise so we could tell whether to send the increase or decrease volume command. The LED driving code is by far the most complicated of all the software we wrote. The first task was understanding and implementing the HUB75 protocol, which involved the control of 6 different color signals, 5 addressing signals, and 3 control signals all operating in unison to even turn on the display. Another challenge stemmed from the fact that each LED actually only held one bit for red, green, and blue each, meaning if we wanted to display a color that wasn’t a direct combination of these three, we had to implement a PWM control over the LEDs. For example, to display a red color with 50% intensity, we would have to turn on the red color for that pixel for 50% of the cycles. We were able to adequately control this by looping through the range of the bit magnitude of our colors and adjusting the delay the pixel was on for by this magnitude, giving the appearance of different colors. This however, required extremely fast speeds in order to avoid flickering in the display. The actual pixel data was received directly from ESP32 into an image buffer, which was used to draw from.

	In comparison the code for the ESP32 was simpler apart from a few small details. All of the ESP32 development was done in the Arduino IDE, which made programming the chip much easier. Many of the functions were simple HTTP requests based on their respective commands from the rotary encoder input. The ESP32 was constantly listening for these commands over the serial line it shared with the STM32. The ESP was also constantly checking if the album link had changed, meaning a new song was playing and new image data needed to be sent to the STM32. If there was a new song, it was downloaded from the link (which is provided by the Spotify API), saved to SPIFFS (the ESP32’s built in filesystem), decoded into the raw pixel values, formatted to fit our 8 bits of color standard (RGB332) and sent over UART to the STM32 for storage in its image buffer. There are a few authorization requirements as well as set up steps in the Spotify Developer Dashboard to grant permissions to our project to view and control a selected spotify account, but these are not included in our source code and can be deduced from Spotify’s Web API Documentation. 

Design Challenges and Solutions:
An initial challenge we ran into was the wifi module. To begin we first used the ESP8266 which had less capabilities (and was much harder to use than the ESP32). We were originally trying to rely on the AT command set from Espressif to control the chip, however both the ESP8266 and the AT Firmware were unable to connect to the WPA2-Enterprise level security deployed by the IllinoisNet wifi network. Also, employing only AT commands would not give us enough control over the wireless chip. As a solution, we switched to the more powerful ESP32 and the Arduino IDE, which allowed us to upload code straight to the chip as well as utilize some helpful WiFi libraries.

Another large issue we faced was the refresh rate of the LED matrix. Matrices controlled by the HUB75 protocol only actually turn on two rows of LEDs at a time, and only while the Output Enable signal is high. The rows are cycled on and on so fast that to the human eye it appears to be a fully lit screen. However, if the driver is not updating the rows fast enough, major flickering will occur on the screen. Furthermore, implementing PWM and more than 3 bits of color meant even more cycles until an entire image was properly displayed (and more memory to store the image buffer). To overcome this, we elected to use a larger and faster MCU than the standard STM32L0 given to us. We also spent a lot of time optimizing our main display loop, trying to avoid calculations within the loop wherever possible and instead relying on lookup-tables defined on startup for proper calculations. We also had to bypass the HAL interface for writing to the GPIO pins, and instead wrote directly to the GPIO registers on the chip. 

 When we implemented the PCB board we ran into the issue uploading code. The new STM we used for the PCB (STM32F103) needs to have the reset pin connected to the programmer when uploading code. To solve this we soldered a wire to both pins on the PCB. Furthermore, we had to create two different voltage sources as our rotary encoder required 5V to operate while the rest of the components like the microcontroller and wifi module required 3.3V.

Project Replication

If one wishes to replicate this project in its entirety, the steps taken are as follows:

1. Download the gbr files from github repo, and send them to your favorite pcb manufacturer (you may want to make the wiring changes mentioned in the third paragraph of section IV).
2. Purchase the parts listed in FinalPCBBoard1/FinalPCBBoard1.xml in the repo
3. Solder all of the components to the board
4. Program the ESP32 using the Arduino IDE and the provided sketch + libraries
5. Create a new STM32 project selecting the F103 as the part and add all of the files from the github repo (be sure the pinouts are the same as the final.ioc file)
6. Program the STM32 and ESP32 on your PCB
7. Supply 3.3V directly to the LDO output pins
8. Apply a 5V souce to the power source on the rotary encoder
