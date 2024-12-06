# PWM

## Changes
**1. Choose Output Pin** 

I chose PWM/MOSI/D11 (on board) corresponding to `PA7` on the ioc diagram. Different port can be chosen but you have to edit the variables accordingly. Set `PA7` to `TIM3_CH2` (3rd Timer, 2nd Channel) or the one you like (configure next steps accordingly)

**2. Set up timer**

In the Timer Category, pick `TIM3`

**Modes**

- Clock Source: Internal Clock
- Channel 2: PWM Generation CH2

**Configuration**

- Prescaler: `0`
- Counter Period: `1000` (Example Value)
- Mode: `PWM mode 1`

Now we're ready to generate the code now

**3. Base Code**

Make sure that the values you configured are the right ones in main.c

Under "Initialize all configured peripherals", in `USER CODE BEGIN 2` (approx line `100`), initialize the PWM link (not exactly sure how it works) with the following code:

```c
HAL_TIM_PWN_Start(&htim3, TIM_CHANNEL_2);
```

The pointer is set to the timer used and the channel is set the chosen one. (Timer 3 and Channel 2 in this case)

**4. Changing the Duty Cycle**

Use this method to change the Duty cycle:

```c
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, RATIO_NB);
```

The two first variables are the same ones as before. 

`RATIO_NB` is the fraction of the duty cycle that we want, dividing `RATIO_NB` by the Counter Period (Ex: `1000`):

For example, using the examples values:

- `RATIO_NB = 500` (gives 1/2 duty cycle)
- `RATIO_NB = 100` (gives 1/10 duty cycle)
- `RATIO_NB = 1000` (gives full duty cycle)

**5. Example Code**

If we want the Duty Cycle to increase by 10% every 2 seconds, we add the following code in the user code while loop:

```c
for (int i = 0; i < 1000; i = i+100) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, i);
	HAL_Delay(2000);
}
```

**6. Measuring the Output**

Since we chose PA7 as the port, we can measure the ouput at PWM/MOSI/D11 (on the board). You can connect an Oscilloscope and connect the ground of the oscilloscope probe into GND
## Config
### IOC Config

- Internal Clock: `64Mhz` (Example Value)
- Prescaler: `0`
- TIM# Counter Period: `1000` (Example Value)

**Note that the PWM frequency output is:**

```
(Internal Clock)/ ((1+Prescaler) * Counter Period)
```

In the example, `(64MHz) / ((1+0) * 1000) = 64KHz`

**Note:** To get a higher amount of possible percentage of duty cycle, increase the Counter Period which determines the nb of possible RATIO_NB.

## Resources

https://www.youtube.com/watch?v=k1jHQ7oW4Uw&t=223s&ab_channel=CMTEQ

https://www.youtube.com/watch?v=iXWyISYmeQ0&ab_channel=BINARYUPDATES


