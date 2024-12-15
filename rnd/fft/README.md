# Fast Fourier Transform (FFT)

The FFT algorithm is used to perform the [discrete Fourier transform](https://en.wikipedia.org/wiki/Discrete_Fourier_transform) (DFT). This takes a *discrete* set of points *sampled from a continuous signal* in the *time domain* and turns it into a set of points in the *frequency domain* with associated amplitudes.

![Fourier Transform image](https://www.nti-audio.com/portals/0/pic/news/FFT-Time-Frequency-View-540.png)

In other words, the FFT takes in a relatively continuous signal (usually sourced from an analog to digital converter (ADC)) and returns the amplitude of the sine/cosine wave of a given frequency in that signal.

---

#### Example

For a signal given by 

$f(t) = \sin\left(2\pi t\right) + \sin\left(2\cdot2\pi t\right) + 3\sin\left(5\cdot2\pi t\right) + 0.5\sin\left(6\cdot2\pi t\right)$,

the output amplitudes over the frequencies $0-6$ would be

$A(0) = 0, \quad A(1) = 1, \quad A(2) = 1, \quad A(4) = 0, \quad A(5) = 3 \quad A(6) = 0.5$.

---

Since this is an implementation of the DFT, the output is an array of amplitudes, with the first (two) elements representing the amplitude of the signal of frequency $0$, then the next representing the amplitude of the signal of frequency $1$, etc.

# Setup ARM CMSIS DSP Library

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

[YouTube tutorial](https://www.youtube.com/watch?v=d1KvgOwWvkM)
