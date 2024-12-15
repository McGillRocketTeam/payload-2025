# Fast Fourier Transform (FFT)

The FFT algorithm is used to perform the [discrete Fourier transform](https://en.wikipedia.org/wiki/Discrete_Fourier_transform) (DFT). This takes a *discrete* set of points *sampled from a continuous signal* in the *time domain* and turns it into a set of points in the *frequency domain* with associated amplitudes.

![Fourier Transform image](https://www.nti-audio.com/portals/0/pic/news/FFT-Time-Frequency-View-540.png)

In other words, the FFT allows us to input a relatively continuous signal (usually sourced from an analog to digital converter (ADC)) and returns the amplitude of the sine/cosine wave of a given frequency in that signal.

---

#### Example

For a signal given by 

$$f(t) = \sin\left(2\pi t\right) + \sin\left(2\cdot2\pi t\right) + 3\sin\left(5\cdot2\pi t\right) + 0.5\sin\left(6\cdot2\pi t\right)$$

the output amplitudes over the frequencies $0-6$ would be

$$A_0 = 0, \quad A_1 = 1, \quad A_2 = 1, \quad A_4 = 0, \quad A_5 = 3 \quad A_6 = 0.5$$

---

# The Discrete Fourier Transform (DFT)

This section is going to have a lot of math, but if you just bear with me, we'll end up with one final equation for what we need.

The Discrete Fourier Transform takes a finite number of data points (readings from sensors) which we sample from at a constant rate (using ADCs to convert the analog signal into a digital one and timers to ensure this happens at a constant rate). Since the data we are sampling consist fully of real numbers, this is also sometimes called the **Real DFT**.

## Definition

Let $\mathbf{x}$ be a buffer or array of $N$ data points which we want to perform Fourier analysis on. Let $f_s$ be the sampling frequency in Hertz. This means that $f$ times per second, we add a data point to a $\mathbf{x}$. The $k^\text{th}$ element of $\mathbf{x}$ be denoted by $x_k$, with $0 \le k \le N - 1$. 

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

Enough theory, let's finally get into some coding. Implementing the FFT from scratch is not trivial (more math!), let alone making it run quickly, so we'll use an external library to do it for us.

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

# Resources

### Conceptual

- [DFT Wikipedia Article](https://en.wikipedia.org/wiki/Discrete_Fourier_transform)
- [3Blue1Brown Video](https://youtu.be/spUNpyF58BY?si=p6CJkcmEaX1rmqaR)

### Practical

- [YouTube tutorial](https://www.youtube.com/watch?v=d1KvgOwWvkM)
- [Frequencies from FFT StackOverflow](https://stackoverflow.com/questions/4364823/how-do-i-obtain-the-frequencies-of-each-value-in-an-fft)
