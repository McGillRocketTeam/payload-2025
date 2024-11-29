# BME280 Temperature Sensor
This project uses a BME280 sensor to read temperature, pressure, and humidity. During testing, these values were read in the debug console, but the values for each variable are stored in a float in main.c. *Please note that this project was designed and tested on a nucleo F303RE board, and so you may need to adjust pinout accordingly.*

**All resources are linked down below.**

### Setup
To setup the project, I first followed the tutorial to configure the clock and communication channel, enabling the crystal clock and setting it to 72Mhz. I also enabled I2C (though this sensor supports sbi as well) and turned on the Serial Wire debug. From Siger's repository, I copied the BME280_STM32.c and BME280_STM32.h files and also the main file. 

### Code
There are three floats defined in main.c: Temperature, Pressure, and Humidity. These variables can be used how you like. There's also a BME280_Config() function that configures the BME280; the settings used work well at room temperature indoors, but please refer to the tutorial video and datasheet to configure it to your needs. In the main while loop, use BME280_Measure() to measure, and you can change HAL_Delay(<delay_in_milliseconds>) to whatever you like, or even remove it. To see the values of temp/pressure/humidity, do the following: build the project -> run in debug mode -> open the debug menu in the upper right corner -> go to live expressions -> add expressions for "Temperature", "Pressure", and "Humidity" (I am unsure if these are case sensitive) -> run in debug mode.

### Cautions

- As stated in Siger's repository, use a resistor between 2 kOhms and 10 kOhms
- Make sure to use 3.3V
- Make sure your pinout is correct
- Make sure to click debug instead of run to be able to view the variables in the debug menu!

### Clock Configuration

The tutorial set the maximum clock frequency to 72Mhz, and I did too. This works. Changing the maximum frequency can cause the sensor to fail to read data; I believe this is because of the synchronous nature of I2C (I think this is mentioned in the tutorial, but I skipped most of it). You can read more about it in the pages linked below.

## Resources
[Tutorial](https://www.youtube.com/watch?v=jDhkfe2YG_o) for an explanation of the BME280 drivers, how it works, and the clock configuration

[Siger's repository](https://github.com/McGillRocketTeam/Orbital_2023-24_Payload/tree/training/BME280_Training) for the pinout layout

[Datasheet](https://cdn.sparkfun.com/assets/e/7/3/b/1/BME280_Datasheet.pdf)

[I2C Protocol](https://en.wikipedia.org/wiki/I%C2%B2C)

[More about the I2C Protocol](https://www.geeksforgeeks.org/i2c-communication-protocol/)
