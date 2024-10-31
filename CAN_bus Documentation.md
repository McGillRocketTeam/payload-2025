# CAN Bus Driver for Avionics Flight Computer Emulator

## 1. Introduction

The Controller Area Network (CAN) bus is a robust and reliable communication system widely used in automotive and industrial applications. Developed by Bosch in the 1980s, it enables microcontrollers and devices to communicate with each other without a host computer, making it ideal for environments where high-speed data transfer and real-time communication are crucial. Its ability to operate in harsh conditions with minimal interference has made the CAN bus a standard in industries that require precise and secure data exchange.

Here we created a CAN bus driver for Avionics flight computer (AV FC) emulator. It is based on the [tutorial](https://www.youtube.com/watch?v=KHNRftBa1Vc) from ControllersTech. This emulator uses a STM32F446ZETx 144pins Nucleo board because it is the same board that is on the Avionics flight computer. The board of AV FC emulator has a blue user button. We can press the blue user button which sends a message on address 0x011 to toggle between Payload Sampling and Payload not Sampling. It can also print out the received Payload data that was sent on address 0x300 and 0x301. The following image is a picture of the nucleo board with the blue user button on the bottom left. When Payload is sampling, we receive actual numbers. When Payload is not sampling, we receive mostly 0’s except the PL_seconds that keeps track of how long the Payload has been ON.

![nucleo_board](assets/nucleo_board.png)

This documentation is a detailed guide for creating a working CAN bus driver of the emulator. For the remainder of the documentation, we will demonstrate the steps. One can follow along the guide in the order that we present.

## 2. Connections

The CAN bus module is connected to the microcontrollers/devices as follows.
![connections](assets/connections.png)

In the above image, two Teensy 4.0 boards are the devices that will communicate through CAN bus. Certainly, we can use any other microcontrollers that support CAN bus communication such as the STM32F446ZETx Nucleo board we are using for the AV FC emulator.

## 3. Project Setup & Code

After having proper connections, we will create the project on STM32CubeIDE. We will then setup the project and write the code.

### 3.1 Create a new project

Let's start by creating a new STM32 project.

1. Go to the board selector and type in "NUCLEO-F446ZE".
2. Select the only board shown.
3. Click "Next" at the bottom.
   ![create_project_select_board](assets/create_project_select_board.png)

Give the project a name and click "Finish" at the bottom. Then we will see the software is downloading everything needed for the project. This may take a while.
![create_project_add_name](assets/create_project_add_name.png)

### 3.2 IOC Setup

#### 3.2.1 Clock

Let's open and ioc file and set up the clock properly first.

1. Go to "Clock Configuration".
2. Select "HSI".
3. Select "PLLCLK".
4. Type "168" in the field "HCLK (MHz)" and press enter.

We are using 168 MHz because Payload and Avionics have agreed to run their STM32 at 168 Mhz.
![setup_clock](assets/setup_clock.png)

#### 3.2.2 GPIO

Go back to "Pinout & Configuration" and clear all the pinouts first (if not already cleared yet).
![setup_clear_pins](assets/setup_clear_pins.png)

Left click on PC13 and select "GPIO_EXTI13".
![setup_pc13_usage](assets/setup_pc13_usage.png)

Right click on PC13 and enter a user label for convenience. Here we use "USER_Btn [B1]".
![setup_pc13_label](assets/setup_pc13_label.png)

#### 3.2.3 CAN

In our case, we are using pins PD1 and PD0 for TXDCAN_A and RXDCAN_A of the nucleo board which corresponds to the following pins on the physical nucleo board.

![can_pins](assets/can_pins.png)

Left click on PD1 and select "CAN1_TX". PD0 should then be automatically set to "CAN1_RX".
![setup_can_pin](assets/setup_can_pin.png)

Go to "Parameter Settings" of CAN1 and set the three parameters marked by red rectangles to the values shown in the image below. By setting these three parameters, the baud rate will be 100000 bit/s, which is the baud rate that Payload and Avionics agreed on using. It is very important that the baud rate is 100000 bit/s since otherwise the CAN bus will not work.
![setup_can_param](assets/setup_can_param.png)

#### 3.2.4 USART

We enable USART so that we are able to see the output of the myprint method (shown later) on the serial monitor. Having this is very useful for the debugging purposes.

Left click on PD8 and select "USART3_TX".
![setup_usart_pin](assets/setup_usart_pin.png)

Left click on PD9 and select "USART3_RX".
![setup_usart_pin2](assets/setup_usart_pin2.png)

Set the mode of USART3 to "Asynchronous".
![setup_usart_mode](assets/setup_usart_mode.png)

Go to "Parameter Settings" of USART3. We must make sure that "Baud Rate" is set to 115200 Bits/s, otherwise it will not work.
![usart_baud_rate](assets/usart_baud_rate.png)

#### 3.2.5 NVIC

Go to "System Core" -> "NVIC" and enable "CAN1 RX0 interrupt" and "EXTI line[15:10] interrupts". We enable this to be able to use the Interrupt Service Routine (ISR).
![setup_nvic](assets/setup_nvic.png)

Now we have finished the setting of the ioc file. We can save it and generate code now.

### 3.3 Code (main.c)

The main.c file should automatically open when we choose to generate code. If it does not open, you can always find it in "Core" -> "Src" -> "main.c" and open it whenever you want.

Let's open main.c and write some code.

#### 3.3.1 Private includes

First we inlcude some libraries that we will be using later.

```C
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h> //for va_list var arg functions
#include <stdbool.h>

/* USER CODE END Includes */
```

#### 3.3.2 Filter setup

Filter is an important aspect of CAN bus since it controls what message/information a device would like to receive. For example, AV flight computer can set up filters that only allow devices with ID 0x68 and/or ID 0x88 to send a message to itself. Similarly, PL flight computer can do the same. Each device use filter to control if it wants or does not want the message from another specific device.

Payload flight computer (PL FC) sends a total of 16 bytes over 2 CAN messages (8 bytes on CAN ID 0x300 and 8 bytes on CAN ID 0x301). The details of the two messages are shown below.
![pl_messages](assets/pl_messages.png)

Remember now we are building an AV flight computer (AV FC) emulator. AV FC needs to allow PL FC's messages to pass through, and now we will set up some filters for that.

Find ``USER CODE BEGIN CAN1 Init 2`` which is located in the function ``static void MX_CAN1_Init(void)``, and write the following code inside that block. The following code sets up two filters, one is for listening to the device with ID 0x300 and one is for listening to the device with ID 0x301.

```C
/* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x300<<5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x300<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 1;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x301<<5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x301<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */
```

There are a lot of fields that have been configured for a filter. We will not go into detail of what each field does. If you are interested, two tutorials by ControllersTech on YouTube, [STM32 CAN LOOPBACK Mode || FILTER Configuration](https://www.youtube.com/watch?v=JfWlIY0zAIc) and [STM32 CAN Communication || NORMAL Mode](https://www.youtube.com/watch?v=KHNRftBa1Vc), provide decent explanation about the filter. You can also hold "ctrl" key and left click the fields to see some details of these fields.

While most of the fields stay the same from filter to filter, there are three fields that need to be changed for different filters, they are:

1. ``FilterIdHigh``: This field indicates what ID can pass through this filter. In the above code, it is "0x300<<5" and "0x301<<5", which allows IDs 0x300 and 0x301 to pass.
2. ``FilterMaskIdHigh``: Same as FilterIdHigh.
3. ``FilterBank``: Increment this by one when creating a new filter.

Let's see an example. We will add a new filter that listens to ID 0x123. The filter configuration will be the following code.

```C
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 2;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x123<<5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x123<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
```

Notice that ``FilterBank`` is incremented from 1 to 2, and both ``FilterIdHigh`` and ``FilterMaskIdHigh`` are changed to the ID it wants to listen to. Others can stay unchanged.

#### 3.3.3 Interrupt service routine

The devices gets an interrupt when it receives a message from a device it wants (by passing through the filter). Then the device will enter interrupt service routine. In this section, we will see how we can write the code for interrupt service routine.

```C
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *format, ...)
{
  static char buffer[256];
  va_list args;
  va_start (args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end (args);
  int len = strlen(buffer);
  HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 1000);
}

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeadermsg1;
CAN_RxHeaderTypeDef RxHeadermsg2;


uint32_t TxMailbox[4];

uint8_t TxData[8];
uint8_t RxDatamsg1[8];
uint8_t RxDatamsg2[8];

volatile int FCSampling = 1; // all variables that are used inside an ISR should be volatile

volatile int msg1ID0x300_received = 0; // all variables that are used inside an ISR should be volatile
volatile int msg2ID0x301_received = 0; // all variables that are used inside an ISR should be volatile
volatile uint8_t msg1copy[8];
volatile uint8_t msg2copy[8];


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeadermsg1, RxDatamsg1);
	uint32_t id = RxHeadermsg1.StdId;
	if (id == 0x300){
		msg1ID0x300_received = 1;
//		msg1copy = memcpy(RxDatamsg1);
		memcpy(msg1copy, RxDatamsg1, sizeof(RxDatamsg1));

	}
	if (id == 0x301){
		msg2ID0x301_received = 1;
//		msg2copy = RxDatamsg1;
		memcpy(msg2copy, RxDatamsg1, sizeof(RxDatamsg1));
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// if the user button is pressed
	if (GPIO_Pin == USER_Btn_Pin)
	{
		if (FCSampling == 1) {
			FCSampling = 0;
			TxData[0] = 0;
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox[0]);
		}else if (FCSampling == 0) {
			FCSampling = 1;
			TxData[0] = 1;
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox[0]);
		}

	}
}
/* USER CODE END 0 */
```

We write the above code because we want the ISR of FIFO0 called HAL_CAN_RxFifo0MsgPendingCallback to be called when a message is received on the filtered address since in section 3.3.2 we configured CAN1 RX0 interrupt:

```C
canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0
```

``TxHeader`` is the header information that is sent along with the data when transmitting (sending) a message. The header typically contains metadata about the data being sent, such as an identifier (ID), the type of message, and control bits that define how the data should be interpreted.

``TxData`` is the actual data payload that is sent after the ``TxHeader``. It contains the meaningful information that the sender wants to communicate to the receiver.

``RxHeader`` (wrote ``RxHeadermsg1``/``RxHeadermsg2`` in the code above) is the header information that is received along with the data when a message is received. It mirrors the ``TxHeader`` on the transmitting side.

``RxData`` (wrote ``RxDatamsg1``/``RxDatamsg2`` in the code above) is the data payload that is received along with the ``RxHeader``. It should match the ``TxData`` that was originally sent by the transmitting device.

``TxHeader`` and ``TxData`` work together on the transmitting side. The ``TxHeader`` provides context for the ``TxData``, ensuring that the receiving device knows how to handle the data correctly.

``RxHeader`` and ``RxData`` work together on the receiving side. The ``RxHeader`` provides the necessary context for interpreting the ``RxData``, ensuring that the data is processed correctly according to the sender's intentions.

The function ``void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)`` indicates the routine when the AV FC emulator gets interrupted by receiving a message from wanted devices (i.e., 0x300 and 0x301). Depends on who the sender is, ``msg1ID0x300_received``/``msg2ID0x301_received`` is set to 1 and the content in ``RxDatamsg1`` is copied into ``msg1copy``/``msg2copy``.

The function ``void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)`` indicates the routine when the blue user button is pressed. ``FCSampling`` is set from 1 to 0 or from 0 to 1. This toggles the mode between Payload Sampling and Payload not Sampling.

For all the variables used in an interrupt service routine, we must declare them as ```volatile``` variables. Declaring a variable ```volatile``` directs the compiler to load the variable from RAM and not from a storage register, which is a temporary memory location where program variables are stored and manipulated. Under certain conditions, the value for a variable stored in registers can be inaccurate. More about ```volatile``` variable can be found [here](https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/).

We can then write the following code in the ``main`` method.

```C
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t status = HAL_CAN_Start(&hcan1);

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  TxHeader.DLC = 8;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x011;
  TxHeader.TransmitGlobalTime = DISABLE;

  myprintf("Welcome to the AV emulator!\n\r");
  myprintf("Press on the blue user button once to make PL FC stop sampling. Press the same button a second time to make the PL FC start sampling.\n\r");
  myprintf("Essentially, the blue user button toggles the PL FC from not sampling to sampling\n\r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (msg1ID0x300_received == 1){
		  myprintf("Received message with Standard ID: 0x300\n\r");
		  myprintf("byte1: %d\n\r",msg1copy[0]);
		  myprintf("byte2: %d\n\r",msg1copy[1]);
		  myprintf("byte3: %d\n\r",msg1copy[2]);
		  myprintf("byte4: %d\n\r",msg1copy[3]);
		  myprintf("byte5: %d\n\r",msg1copy[4]);
		  myprintf("byte6: %d\n\r",msg1copy[5]);
		  myprintf("byte7: %d\n\r",msg1copy[6]);
		  myprintf("byte8: %d\n\r",msg1copy[7]);

		  msg1copy[0] = 0;
		  msg1copy[2] = 0;
		  msg1copy[2] = 0;
		  msg1copy[3] = 0;
		  msg1copy[4] = 0;
		  msg1copy[5] = 0;
		  msg1copy[6] = 0;
		  msg1copy[7] = 0;

		  msg1ID0x300_received = 0;
	  }

	  if (msg2ID0x301_received == 1){
		  myprintf("Received message with Standard ID: 0x301\n\r");
		  myprintf("byte1: %d\n\r",msg2copy[0]);
		  myprintf("byte2: %d\n\r",msg2copy[1]);
		  myprintf("byte3: %d\n\r",msg2copy[2]);
		  myprintf("byte4: %d\n\r",msg2copy[3]);
		  myprintf("byte5: %d\n\r",msg2copy[4]);
		  myprintf("byte6: %d\n\r",msg2copy[5]);
		  myprintf("byte7: %d\n\r",msg2copy[6]);
		  myprintf("byte8: %d\n\r",msg2copy[7]);

		  msg2copy[0] = 0;
		  msg2copy[2] = 0;
		  msg2copy[2] = 0;
		  msg2copy[3] = 0;
		  msg2copy[4] = 0;
		  msg2copy[5] = 0;
		  msg2copy[6] = 0;
		  msg2copy[7] = 0;

		  msg2ID0x301_received = 0;
	  }

  }
  /* USER CODE END 3 */
}
```

In the while loop, the message received from 0x300 and/or 0x301 will be printed. The variables ``msg1copy`` and/or ``msg2copy`` will then be set to 0 so that they are ready to get the next message. ``msg1ID0x300_received`` and/or ``msg2ID0x301_received`` are also set back to 0.

For the `myprintf` function to work with float numbers, you need to enable `-u _printf_float` in the `Project Properties`. Otherwise, you will get an error. The float formatting support can be enabled in your MCU Settings from "Project Properties > C/C++ Build > Settings > Tool Settings", or add manually "-u _printf_float" in linker flags.

![myprintf](assets/myprintf.png)

Please note that the UART/USART associated with `myprintf` (see `huart[X]` in `myprintf` function) must be enabled at a baud rate of 115200.

Now we can connect the nucleo board to the PC and flash the code into the nucleo board.

### 3.4 PuTTY session and serial monitor

As mentioned above, seeing the outputs is very important for many purposes like debugging. We will open a PuTTY seesion to see the serial monitor.

We first open Device Manager on the PC. The easiest way I find to open it is through searching located on the task bar.
![device_manager](assets/device_manager.png)

In the Device Manager interface, we can see many categories. Look for a category called "Ports (COM and LPT)" and expand it. If the nucleo board is properly connected to the PC, we should able to see a new device named "STMicroelectronics STLink Virtual COM Port" show up in this category. Take note of what is in the bracket after the device name. It should be in the format of "(COMx)" where x is an integer.
![ports](assets/ports.png)

Now open PuTTY and have the "Terminal" configuration set as follows.
![putty_terminal_config](assets/putty_terminal_config.png)

In "Serial line" of the following interface, put in the "COMx" that we took note of before. Set "Speed" to 115200, which corresponds to the baud rate of USART3. It can be inconvenient to set these every time we open PuTTY. Hence we can set up these once and save the session. In the future, we can load it quickly from "Saved Sessions".

![putty_session](assets/putty_session.png)

Now click "Open" at the bottom. We will see a terminal show up. We can then press the blue user button on the nucleo board and see the outputs (if any) in the terminal.

## 4 Useful links

[STM32 CAN LOOPBACK Mode || FILTER Configuration](https://www.youtube.com/watch?v=JfWlIY0zAIc)

[STM32 CAN Communication || NORMAL Mode](https://www.youtube.com/watch?v=KHNRftBa1Vc)

[volatile variable](https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/)

[Introduction to the Controller Area Network (CAN)](https://www.ti.com/lit/an/sloa101b/sloa101b.pdf)

[CAN Bus Explained - A Simple Intro [v1.0 | 2019]](https://www.youtube.com/watch?v=FqLDpHsxvf8)

[Learn How The CAN Bus Works (Controller Area Network) | Embedded Systems Explained](https://www.youtube.com/watch?v=v_y65h68z3U)
