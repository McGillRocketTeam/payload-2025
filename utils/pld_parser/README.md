# SD Card Data Parser

This tool is used to convert the Payload data written on the SD card from our binary `PLD` (PayLoad Data) format into two `CSV` (Comma Separated Value) files.

## Compilation

The compiler is written in one C header file ([`parse.h`](./parse.h)) and one C source file ([`parse.c`](./parse.c)).
To use the parser, you must first compile it with your C compiler of choice.

On UNIX (Linux or Max) you should have gcc automatically installed, or be able to install it easily.

If you're on Windows, you can use [`MSVC`](https://visualstudio.microsoft.com/vs/features/cplusplus/) or [`mingw`](https://www.mingw-w64.org/).
Alternatively, you can install [`wsl`](https://learn.microsoft.com/en-us/windows/wsl/) (Windows Subsystem for Linux) which allows you to use a Linux environment on a Windows machine.

### Makefile

If your preferred C compiler is `gcc`, a Makefile is already set up in this directory, so all you need to do is run `make`.

## Usage

First, ensure you have [compiled](#compilation) the parser.

If you're in the current directory, use can run the parser with `./parse` on UNIX and `.\parse.exe` on Windows.
If you're in the project root, you can run the parser with `./utils/pld_parser/parse` or `.\utils\pld_parser\parse.exe`.

Each argument to the parser should be a path to a `.pld` file (case insensitive).
For each file passed to the parser, two `.csv` files will be created in the same directory as the corresponding `.pld` file.
One will contain the telemetry data, and one will contain the accelerometer data.

> ### Example
>
> ```bash
> # Parses the file DATAn.PLD in the current directory into DATAn_accelerometer.csv and DATAn_telemetry.csv
> ./parser ./DATAn.PLD
> # Parses the file DATAx.PLD in the data directory into ~/data/DATAx_accelerometer.csv and ~/data/DATAx_telemetry.csv
> ./parser ~/data/DATAx.PLD
> ```

## Customization

The SD card driver and parser are designed to be flexible.
If the data being logged changed, simply change the macros `ACCELEROMETER_PACKET` and `TELEMETRY_PACKET` in `parse.h` to match the definitions in `SD_card.h`.
Also, change the definitions of `CSV_HEADER_ACCELEROMETER`, `CSV_FORMAT_ACCELEROMETER`, `CSV_HEADER_TELEMETRY`, and `CSV_FORMAT_TELEMETRY` to match.
