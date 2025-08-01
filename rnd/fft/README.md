# Fast Fourier Transform (FFT)

> **Note:** This document is best viewed in the VSCode Markdown preview. Some of the equations don't show up correctly using other Markdown Renderers.

The FFT algorithm is used to perform the [discrete Fourier transform](https://en.wikipedia.org/wiki/Discrete_Fourier_transform) (DFT). This takes a *discrete* set of points *sampled from a continuous signal* in the *time domain* and turns it into a set of points in the *frequency domain* with associated amplitudes.

![Fourier Transform image](https://www.nti-audio.com/portals/0/pic/news/FFT-Time-Frequency-View-540.png)

In other words, the FFT allows us to input a relatively continuous signal (usually sourced from an analog to digital converter (ADC)) and returns the amplitude of the sine/cosine wave of a given frequency in that signal.

---

#### Example

For a signal given by 

$$f(t) = \cos\left(2\pi t\right) + \sin\left(2 \cdot 2\pi t\right) + 3\sin\left(3 \cdot 2\pi t \right) + 4\cos\left(3 \cdot 2\pi t \right) + 3\sin\left(5 \cdot 2\pi t\right) + 0.5\sin\left(6 \cdot 2\pi t\right)$$

the output amplitudes over the frequencies $0-6$ would be

$$A_0 = 0, \quad A_1 = 1, \quad A_2 = 1, \quad A_3 = 5, \quad A_4 = 0, \quad A_5 = 3 \quad A_6 = 0.5$$

A graph of this signal can be seen [here](https://www.desmos.com/calculator/gx5dpzjwd9).

---

# The Discrete Fourier Transform (DFT)

This section is going to have a lot of math, but if you just bear with me, we'll end up with one final equation for what we need.

The Discrete Fourier Transform takes a finite number of data points (readings from sensors) which we sample from at a constant rate (using ADCs to convert the analog signal into a digital one and timers to ensure this happens at a constant rate). Since the data we are sampling consist fully of real numbers, this is also sometimes called the **Real DFT**.

## Definition

Let $\mathbf{x}$ be a buffer or array of $N$ data points which we want to perform Fourier analysis on. Let $f_s$ be the sampling frequency in Hertz. This means that $f$ times per second, we add a data point to a $\mathbf{x}$. The $k^\text{th}$ element of $\mathbf{x}$ is denoted by $x_k$, with $0 \le k \le N - 1$. 

The Discrete Fourier Transform, notated by $\mathcal{F}$, outputs $N$ **complex numbers** called **coefficients**. Let $\mathbf{X}$ be an array or buffer of these coefficients such that

$$\mathcal{F}(\mathbf{x}) = \mathbf{X}.$$

The equation for the $k^\text{th}$ coefficient is given by 

$$X_k = \sum_{n=0}^{N-1} x_n \cdot e^{-2\pi i \frac{k}{N} n}.$$

Consequently, the inverse DFT $\mathcal{F}^{-1}(\mathbf{X})=\mathbf{x}$ is given by

$$x_n=\frac{1}{N} \sum_{k=0}^{N-1} X_k \cdot e^{2\pi i \frac{k}{N} n}.$$

This may look complicated, but essentially, each $X_k$ is the **cross-correlation** of the signal $\mathbf{x}$ with sinusoidal functions of the frequency $\frac{k}{N}$. The cross-correlation is essentially how much of a given function is present in the signal. As seen in the equation for the inverse DFT, if you added together all of the functions multiplied by their cross-correlations, then you would end up with the original signal.

Using [Euler's formula](https://en.wikipedia.org/wiki/Euler%27s_formula), we can find that $X_k$ is of the form

$$X_k = Ae^{i\phi} = A(\cos\phi + i\sin\phi) = a + bi .$$

This can be interpreted in a two different ways.

1. Cartesian

In this case, $a$ measures the cross-correlation (amplitude) of the signal with a $\cos$ function of frequency $\frac{k}{N}$ while $b$ measures the cross-correlation (amplitude) of the signal with a $\sin$ function of frequency $\frac{k}{N}$. Since a sum of any number of sinusoids of the same freuency is just another sinusoid of the same frequency (but with different a different phase angle), the sum of the two functions completely describe the sinusoidal component of the signal with frequency $\frac{k}{N}$.

2. Polar

In this case, $A$ is the amplitude of the coefficient and $\phi$ is its phase angle. The whole coefficient tells us that the sinusoidal component of the signal with frequency $\frac{k}{N}$ has an amplitude $A$ and a phase angle $\phi$.

In either interpretation, the coefficient accounts for both the amplitude and phase angle of the sinusoidal component of the signal. Even though the coefficients are complex, when fully added together in the inverse DFT the original, real signal can be recreated.

## DFT Parameters

This whole time, we've been concerning ourselves with sinusoids of frequency $\frac{k}{N}$, but this has implicitly been multiplied by the sampling frequency $f_s$ the whole time. So, the actual frequency measured by each coefficient $X_k$ is

$$f_k = f_s \frac{k}{N}.$$

This gives us two relevant parameters which determine the "quality" of our DFT: 

1. Sampling Frequency $f_s$:

Since $k$ ranges from $0$ to $N - 1$, this means that we can detect frequences ranging from $0$ to $f_s$. If we want to detect higher frequencies, we need to increase our sampling rate. Therefore, $f_s$ determines our **frequency range**.

2. Number of Samples $N$:

Keeping $f_s$ constant, we see that since $k$ is a discrete value, so there are $N$ values of $f_k$, so we detect $N$ frequencies, evenly spaced between $0$ and $f_s$. If we want to detect *more* frequencies in between those two, we need to increase the number of samples we take. Therefore, $N$ determines our **frequency resolution**.

> $f_0 = 0$ is called the **DC frequency** (short for direct current, since that also has a frequency of zero compared to AC), and its amplitude is a constant offset underneath the sinusoidal functions.

### Nyquist Frequency

However, for reasons a bit beyond us right now, the **Real DFT**'s coefficients from $k = 0$ to $k = \frac{N}{2} - 1$ are the complex conjugates of the coefficients from $k = \frac{N}{2} + 1$ to $k = N$. This means that we don't actually get any useful information from the second half of the coefficients, so only the frequencies from $f_0 = 0$ to $f_{\frac{N}{2} - 1}$ have meaningful information attached to them.

This means that the *actual* highest frequency we can measure with a sampling rate of $f_s$ is $\frac{f_s}{2}$. This is called the **Nyquist Frequency**.

## Extracting Useful Information from the DFT

Now that we know all about the DFT (or at least, we know what kind of data the DFT returns once we throw all of our data in the funny little magical black box), we can actually get something useful from it. Our main goal in utilizing the FFT is to find the frequency and ampltiude of vibrations, so let's see how to find those. 

### Frequency

For a given $k$, we already know that 

$$f_k = f_s \frac{k}{N}.$$

As discussed earlier, our choices of $f_s$ and $N$ determine the number and values of our $f_k$s, and these may not be integers. If you want to measure a specific frequency, you need to make sure that it is some integer multiple of $\frac{f_s}{N}$. *We* are not interested in measuring any specific frequency, so we can freely choose $f_s$ and $N$ based on other constraints and measure the frequencies that we get.

### Amplitude

Earlier, I said that the DFT coefficient $X_k = a_k + b_k i$ is essentially the amplitudes of the $\sin$ and $\cos$ wave in the signal, but this is not quite true. 

The way the DFT is defined, $X_k$ is actually those amplitudes *scaled by a factor of $N$*; additionally, the amplitude of the frequencies $f_k$ for $k \ne 0$ (DC Frequency) and $k \ne \frac{N}{2}$ (Nyquist Frequency) is multiplied by a factor of two. Therefore, to find the actual amplitudes, we need to scale $X_k$ by the appropriate factor.

Finally, to find the magnitude/amplitude of a complex number $z = a + bi$, we just need to use the Pythagorean Theorem, so

$$|X_k| = \sqrt{a_k^2 + b_k^2}.$$

Putting this all together the amplitude $A_k$ of the frequency $f_k$ is

$$A_k = 2 \frac{|X_k|}{N} \text{ for } k \ne 0 \text{ and } k \ne \frac{N}{2},$$
$$A_k = \frac{|X_k|}{N} \text{ for } k = 0 \text{ and } k = \frac{N}{2}.$$

---

#### Example

The [example at the beginning](#example) is a bit idealized. Using the DFT, you can't just choose which frequencies to find the amplitudes of, but it depends on $f_s$ and $N$. If you choose the closest $f_k$ to your target frequency, you will find that the amplitude $A_k$ is a bit less than the actual one, but it can be approximated to be the same.

For our purposes, we're mostly interested in finding the peak frequency, so finding the $f_k$ with the highest $A_k$. This will approximate the actual peak frequency with the closest $f_k$, but that's a good enough approximation with a sufficiently high $N$.

---

# Setup of ARM CMSIS DSP Library

Enough theory, let's finally get into some coding (kind of). Implementing the FFT from scratch is not trivial (more math!), let alone making it run quickly, so we'll use an external library to do it for us.

## Terms

- ARM: CPU manufacturer whose architecture the STM32 uses.
- CMSIS: Common Microcontroller Software Interface Standard
  - Made by ARM
  - Makes use of peripherals and other microcontroller functions easier and more consistent
- DSP: Digital signal processing

Since we don't want to write the FFT algorithm ourselves, we are going to be using (stealing) functions from the ARM CMSIS Library to use for ourself.

## Determine Processor Type

Before we set up the library, we need to know what type of processor we're using. This is something of the form *Cortex-M**X***, and you can find it on the datasheet for the STM32 version you're using. We also need to know if it has a *Floating Point Unit (FPU)*, which can also be found on the datasheet.

## Files

First, go to the [ARM CMSIS Library](https://github.com/ARM-software/CMSIS_4). Even if it's not the most up to date version, it has precompiled binaries which make it easier to use. 

We want two files: `libarm_cortexMXx_math.a` and `arm_math.h`. The first is the precompiled library which contains all of the code that will do the relevant math, while the second is the header file which defines the interface for the library so we know what functions are contained in the binary and can use them in our code.

---

### Header File `arm_math.h`

From the CMSIS Library, navigate to [`CMSIS/Include/arm_math.h`](https://github.com/ARM-software/CMSIS_4/blob/master/CMSIS/Include/arm_math.h) and download the raw file. 

Place the file in the `Core/Inc` directory of your project. 

Then, we need to specify the processor type that we determined earlier. Depending on which one your STM32 has, insert one of the following lines after line `300` of `arm_math.h`. 

```c
#define ARM_MATH_CM7
#define ARM_MATH_CM4
#define ARM_MATH_CM0
#define ARM_MATH_CM0_PLUS
```

---

#### Example

Since this project is using an STM32-F446 which has a Cortex-M4 processor, I used 
```c
#define ARM_MATH_CM4
```

---

If you forget to do this, the preprocessor will raise and error when you try to build the project.

### Precompiled Library `libarm_cortexMXx_math.a`

From the CMSIS Library, navigate to [`CMSIS/Lib/GCC/`](https://github.com/ARM-software/CMSIS_4/tree/master/CMSIS/Lib/GCC). Each file is a precompiled binary specific to a processor type.

The explanation below on which one to pick is modified from lines 69-82 of [`arm_math.h`](./Core/Inc/arm_math.h).

> - `libarm_cortexM7lfdp_math.a`: Little endian and Double Precision Floating Point Unit on Cortex-M7
> - `libarm_cortexM7bfdp_math.a`: Big endian and Double Precision Floating Point Unit on Cortex-M7
> - `libarm_cortexM7lfsp_math.a`: Little endian and Single Precision Floating Point Unit on Cortex-M7
> - `libarm_cortexM7bfsp_math.a`: Big endian and Single Precision Floating Point Unit on Cortex-M7
> - `libarm_cortexM7l_math.a`: Little endian on Cortex-M7
> - `libarm_cortexM7b_math.a`: Big endian on Cortex-M7
> - `libarm_cortexM4lf_math.a`: Little endian and Floating Point Unit on Cortex-M4
> - `libarm_cortexM4bf_math.a`: Big endian and Floating Point Unit on Cortex-M4
> - `libarm_cortexM4l_math.a`: Little endian on Cortex-M4
> - `libarm_cortexM4b_math.a`: Big endian on Cortex-M4
> - `libarm_cortexM3l_math.a`: Little endian on Cortex-M3
> - `libarm_cortexM3b_math.a`: Big endian on Cortex-M3
> - `libarm_cortexM0l_math.a`: Little endian on Cortex-M0 / CortexM0+
> - `libarm_cortexM0b_math.a`: Big endian on Cortex-M0 / CortexM0+

Download the correct raw `.a` file for the processor you're using. Use little endian by default if you're not sure.

---

#### Example

Since this project is using an STM32-F446 which has a Cortex-M4 processor with an FPU, I chose `libarm_cortexM4lf_math.a`.

---

If it doesn't exist already, create a directory called `Lib` in your project root, and place the `.a` file in it. **In this project's [`Lib`](./Lib/) directory are the `.a` files for Cortex-M4, Cortex-M4 with FPU, and Cortex-M3 since those are the most commonly used versions for our purposes.**

Now, we need to tell the C linker to link this library in with our project when we build it. In STM32CubeIDE, go to `Project > Properties > C/C++ Build > Settings > MCU/MPU GCC Linker > Libraries`. 

In the bottom box, `Library Search Path (-L)`, click the `Add...` button (the leftmost button on the right of the Library Search Path bar). In the popup, click `Workspace...`, then navigate and select the `Lib` directory in the current project. This should result in the string `"${workspace_loc:/${ProjName}/Lib}"`.

In the top box, `Libraries (-l)`, click the `Add...` button (the leftmost button on the right of the Libraries bar). In the popup, type the name of the library file you chose based on your processor type, excepting the `lib` from the beginning and the `.a` extension from the end.

---

#### Example

Since for this project I chose `libarm_cortexM4lf_math.a`, I would input `arm_cortexM4lf_math`.

---

Finally, click `Apply and Close`. 

---

You should now be able to write
```c
#include "arm_math.h"
```
and build the project without errors.

# Programming

## Code Overview

First, we define a few global variables. We'll need a buffer for the input data points and its current index, and a buffer for the output coefficients. We'll also need a handler struct for the FFT instance as well as a boolean flag which indicates whether the buffer of data points is full and analysis is ready to be performed on it.

Depending on what we want to do with our data we may want other variables. If we want to know the full spectrum amplitude, we can make a buffer containing the correct amplitudes for each $f_k$. If we just want to know the peak frequency and its amplitude, then we can make one variable to store the frequency and one for the amplitude.

The code will have three major regions:

1. Initialization

This code simply initializes the FFT handler struct with relevant parameters so it can be passed in when calling the FFT function.

2. Sampling

This code is actually gets the data points to perform Fourier Analysis on. It is called periodically via STM32 timer interrupts to maintain a constant sampling frequency. 

Every time this code is called, it gets the current value from whatever sensor it is in charge of sampling, and writes it into the input buffer at the current index, then increments the index. 

If the input buffer is now full, it calls the FFT function which fills the output buffer with the DFT coefficients. It then sets the flag to true to signal that they are ready to be analyzed and resets the input buffer index to zero so it can start filling it up again the next time it's called.

3. Analysis

This code is in the main while loop, and once every loop it checks if the FFT flag is true. If so, then it performs the analysis.

For a peak frequency detector, all it does is find the coefficient in the output buffer with the highest amplitude and calculate and store the correct frequency and amplitude for that coefficient.

For a full spectrum detector, it fills the amplitude buffer with the correct amplitudes for each coefficient, and the correct frequency $f_k$ can be calculated later based the amplitude's index $k$.

## Macros/Defines

Since the DFT requires [parameters](#dft-parameters), we'll define them as macros. In the private macro section, define:

1. Number of Samples ($N$/`FFT_SIZE`)

The FFT algorithm runs fastest when the number of points is a power of two. For this library, the FFT size must be a power of 2.

2. Sample Rate/Frequency ($f_s$/`SAMPLE_RATE`)

This is in Hertz, and can be whatever frequency you want, as long as the frequency is constant. This can be achieved with timers and/or interrupts.

---

#### Example

```c
/* USER CODE BEGIN PM */
#define FFT_SIZE 2048
#define SAMPLE_RATE 1000.0f /* Hertz */
/* USER CODE END PM */
```

---

## Library Functions

From the ARM CMSIS DSP libarary, we'll be using two functions and one struct. 

All of these will begin with `arm_rfft_fast` and end with `f32`. `arm` is obvious, `rfft` stands for Real Fast Fourier Transform, and `fast` means do it fast please. `f32` is the type that we'll be using, 32-bit floating point numbers, which is usually the default for the `float` type in C.

### `typedef struct arm_rfft_fast_instance_f32`

This struct is used to store the data about the current Real FFT instance. We won't need to deal with it directly, we'll just pass it between the next two functions to keep track of the parameters.

### `arm_rfft_fast_init_f32`

```c
arm_status arm_rfft_fast_init_f32 (
   arm_rfft_fast_instance_f32 * S,
   uint16_t fftLen);
```

This function is used to initialize the FFT instance (`arm_rfft_fast_instance_f32` struct). First, declare one, then pass its address as the first argument. The number of samples `FFT_SIZE` should be passed as the second argument.

### `arm_rfft_fast_f32`

```c
void arm_rfft_fast_f32(
  arm_rfft_fast_instance_f32 * S,
  float32_t * p, float32_t * pOut,
  uint8_t ifftFlag);
```

Like the previous function, the first argument should be the `arm_rfft_fast_instance_f32` declared earlier. The second argument `p` is the buffer of input data (of length `FFT_SIZE`) and the second argument `pOut` is the buffer for the output data to go in (also of length `FFT_SIZE`). The final argument is a single boolean; if true or `1`, it will compute the inverse DFT on the input data and put the corresponding signal into the output buffer. For our purposes, this should always be false or `0`.

However, since the output of the DFT is a set of `FFT_SIZE` complex coefficients, how does it fit into a buffer of the same size as the input? Well, as seen in the section on the [Nyquist Frequency](#nyquist-frequency), only the first half of the coefficients contain useful information (for the Real DFT). So, the output buffer *actually* contains the real and complex values of those *first `FFT_SIZE / 2` coefficients*, alternating real and complex parts. The real part of the `i`th coefficient is found at index `2 * i` and the complex part found at `2 * i + 1 `.

## Global Variables

As described in the [code overview](#code-overview), here the global variables to put in the `USER CODE BEGIN PV` (private variables) section.

```c
/* USER CODE BEGIN PV */
arm_rfft_fast_instance_f32 fft_handler;

/**
 * Buffer of sample points from signal to input to the FFT
 */
float fft_input[FFT_SIZE] = {0};
/**
 * FFT coefficients buffer.
 * Contains FFT_SIZE / 2 coefficients with each even and odd element being the
 * real and imaginary components of each coefficient, respectively.
 */
float fft_output[FFT_SIZE] = {0};
/**
 * Buffer containing the amplitudes of each coefficient.
 */
float fft_amplitudes[FFT_SIZE / 2] = {0};

int fft_index = 0;
uint8_t fft_ready = 0;

float peak_amplitude;
uint16_t peak_freq; /* Hertz */
/* USER CODE END PV */
```

`fft_handler` is the struct used to store the parameters for the `arm_rfft_fast_*_f32` functions. 

The `fft_input` and `fft_output` buffers are declared and initialized as zero; both must have the same length, `FFT_SIZE`.

## Useful Functions

### Index to Frequency

```c
/**
 * Converts the index of the list of amplitudes to a frequency in Hz,
 * rounded to an integer.
 */
uint16_t index_to_frequency(int i)
{
  return (uint16_t) roundf(i * SAMPLE_RATE / FFT_SIZE);
}
```

This uses the relationship between frequencies found in the [DFT parameters section](#dft-parameters) to determine the frequency represented by the `i`th coefficient.

It also rounds it to the nearest integer and returns it as a `uint16_t`. This is because for our purposes, finding the exact frequency and storing it as a `float` (32 bits) is a bit too big and unnecessarily precise. If you need the exact real number value, change the return type of the function and remove the cast and the round function. 

> **Note:** If using this function with the raw FFT output buffer, since each coefficient takes up two slots, the values at indices `i` and `i + 1` are the values of the `i / 2`th coefficient. Therefore, if you're keeping track of the raw FFT output indices, you should call this function with `i / 2` as the argument.

### Normalized Amplitude

```c
/**
 * Find the amplitude of a complex number a + bi.
 * @param a: The real part of the complex number
 * @param b: The Imaginary part of the complex number.
 */
float normalized_amplitude(float a, float b)
{
  return sqrtf(a * a + b * b) / FFT_SIZE;
}
```

This uses the values found for the actual amplitudes of the frequencies represented by each coefficient in the [Extracting Useful Information - Amplitude](#amplitude) section.

> **Note:** This does not apply the additional scaling depending on the index/frequency. This function can be used with no altering of the output for the DC Frequency/index `0`, but the value needs to be multiplied by 2 for all the others. This is a small optimization.

## Simulated Examples

This example uses the previously defined [global variables](#global-variables), and implements all three sections outlined in the [code overview](#code-overview). Keep in mind that for slightly different use cases, the code will be different.

1. Initialization

In the `USER CODE BEGIN 2` (initialization) section, use the `arm_rfft_fast_init_f32` function to initialize the FFT instance. Pass a pointer to the FFT handler and the FFT size.

```c
/* USER CODE BEGIN 2 */
arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);
/* USER CODE END 2 */
```

2. Sampling

Instead of sampling from an actual signal, we'll be using a signal function which takes in the current time and gives an output based on mathematical functions. This makes it so we know what frequencies are already present in the signal for sure and it's easy to tweak and test with.

```c
float signal_function(int t_ms)
{
  float t_s = t_ms / SAMPLE_RATE;
  float scaled_time = t_s * (2 * PI);
  float value = cosf(scaled_time) + 2 * sinf(2 * scaled_time) + 3 * sinf(3 * scaled_time) + 4 * cosf(3 * scaled_time) + 3 * sinf(5 * scaled_time) + 0.5 * sinf(6 * scaled_time);
  return value;
}
```

This function takes as input the time in milliseconds, then converts it such that the sinsudoial functions have periods of integer frequencies measured in Hertz. The signal that it outputs is the same one given in [the first example](#example) (graph can be found [here](https://www.desmos.com/calculator/gx5dpzjwd9)).

In this example, all the sampling just happens in the `Infinite Loop`/`USER CODE BEGIN WHILE`/`USER CODE BEGIN 3` section. However this is just for simulation and testing purposes. **In the final code, all of this code should be put in a callback function triggered by a timer to maintain a constant polling rate**.

```c
/* Infinite loop */
/* USER CODE BEGIN WHILE */
int counter = 0;
while (1)
{
    /*
     * Get signal from source
     * For this example, the signal is simulated from a sum of sinusoidal functions
     * In the final code, this section would probably be put in a callback function
     * 	triggered by the sensor sampling timer
     */
    float signal = signal_function(counter);
    fft_input[fft_index] = signal;
    fft_index++;

    if (fft_index == FFT_SIZE)
    {
        /*
         * Perform Fast Fourier Transform
         * Cast the input and output buffers to (float *) to avoid a warning since
         * we already gave the FFT instance the buffer size when instantiating it and
         * it doesn't expect an array of a specific size.
         */
        arm_rfft_fast_f32(&fft_handler, (float *) &fft_input, (float *) &fft_output, 0);
        // Reset FFT tracking
        fft_ready = 1;
        fft_index = 0;
    }

    /*
      * Give a bit of visualization for the value of the signal
      * Turn LED on when the signal is positive and off when it's negative
      */
     HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, signal > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
     // Increase simulation step
     counter++;

     // Other things that happen in the main loop
     ... 
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Delay 1ms to simulate sampling frequency of 1000 Hz
    HAL_Delay(1);
}
/* USER CODE END 3 */
```

First we initialize a counter variable to act as a simulated timer for input to the signal function.

We then get the signal from the simulated signal function, append the value to the array, and increment the FFT input array index.

Next, we check if the FFT input array is full. If so, we perform the FFT using the `arm_rfft_fast_f32` function. We then set the `fft_ready` flag to `true`/`1` to signal that the output array has been updated and is ready to be analyzed and set the `fft_index` back to `0`.

Next is just a bit of visualization to give a little idea just what the simulated signal looks like to the tester, where we set the LED to be on if the simulated signal is positive and off if it's negative. This is entirely unnecessary in the final code.

We then increase the counter to step forward the simulated signal by one millisecond, and after the rest of the code in the main loop is done running, we delay by `1` millisecond to simulate a sample rate of `1000` Hertz.

3. Analysis

This code actually belongs in the main loop and is not just there for simulation and testing purposes. 

```c
/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
    // Other things that happen in the main loop
    ...

    /*
     * Handle the FFT output data
     * This code can go in the main function as is
     */
    if (fft_ready)
    {
        // Amplitude of zero frequency
        fft_amplitudes[0] = normalized_amplitude(fft_output[0], fft_output[1]);
        for (int i = 2; i < FFT_SIZE; i += 2)
        {
            // Amplitude of nonzero frequencies are twice the normalized amplitude of the FFT coefficients
            fft_amplitudes[i / 2] = 2 * normalized_amplitude(fft_output[i], fft_output[i + 1]);
        }

        // Find peak frequency and its amplitude
        peak_amplitude = 0;
        int peak_index = 0;
        for (int i = 0; i < FFT_SIZE / 2; i++)
        {
            if (fft_amplitudes[i] > peak_amplitude)
            {
                peak_amplitude = fft_amplitudes[i];
                peak_index = i;
            }
        }
        peak_freq = index_to_frequency(peak_index);

        // Reset FFT ready flag
        fft_ready = 0;
    }

    // Other things that happen in the main loop
    ...

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
```

Every time through the loop, the `fft_ready` flag is checked. If it is set to `true`/`1`, then it proceeds with the analysis of the FFT output array. At the end of the analysis, it resets the `fft_ready` flag back to `false`/`0`.

The analysis section contains two examples: first, a full spectrum analysis and second a simple peak detector.

The first section uses the [`normalized_amplitude`](#normalized-amplitude) function on every FFT coefficient to create an array of amplitudes. 

The second section loops over this array to find the peak (highest) amplitude and its index. Once the entire array has been checked, it uses the [`index_to_frequency`](#index-to-frequency) function to find the frequency of that amplitude.

> **Note:** For a simple peak detector, the `normalized_amplitude` function should not be used. This is because the square root function is relatively expensive and does not need to be used on every coefficient to find the maximum amplitude. Instead, the two sections should be combined and the `fft_amplitudes` array should be forgone entirely; simply compute the square of the normalized amplitude (the value before being square rooted) and keep track of the largest value of that. Then, at the end, use the `normalized_amplitude` function once, only on the peak frequency, to get its accurate amplitude.

## Observing Results with Debugging

### Peak Detection

To see if peak detection is working correctly, we're going to use live expressions. Since `peak_frequency` and `peak_amplitude` are global variables, they can be read from anywhere, all the time.

When debugging, navigate to the top right panel, and click the tab labeled `Live Expressions` (the icon is spiky glasses) (make sure you chose `Live Expressions` instead of just normal `Expressions`).

Click *`+ Add new expression`* and type in `peak_frequency`  and `peak_amplitude`. Depending of your sampling frequency and FFT size, you will see these values update periodically every few seconds. 

The values may vary a bit even with a pure signal, since the exact data point that the input array starts at will be different evey time. For this example, the peak frequency should be about `3` and the peak amplitude should be about `5.0`.

### Full Spectrum

To see the results of the full spectrum analysis of the last FFT outputs, pause the debugging by clicking the `Suspend` button which looks like a pause section in the debugging section of the top bar.

It will pause execution in a pretty random location, but that won't matter. Navigate to where the `fft_amplitudes` array is defined and hover over it. This will allow you to see every amplitude in the array and its index. 

Keep in mind that each index is not a frequency in Hertz, but you will have to mentally or manually transform the index via the [`index_to_frequency`](#index-to-frequency) function to find the actual frequency that it represents.

# Resources

### Conceptual

- [DFT Wikipedia Article](https://en.wikipedia.org/wiki/Discrete_Fourier_transform)
- [3Blue1Brown Video](https://youtu.be/spUNpyF58BY?si=p6CJkcmEaX1rmqaR)

### Practical

- [YouTube tutorial](https://www.youtube.com/watch?v=d1KvgOwWvkM)
- [Frequencies from FFT StackOverflow](https://stackoverflow.com/questions/4364823/how-do-i-obtain-the-frequencies-of-each-value-in-an-fft)
