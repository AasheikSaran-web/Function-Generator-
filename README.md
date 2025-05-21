Function Generator using Tiva C TM4C123GH6PM
Authors
Hariram

Aasheik Saran

Overview
This project implements a two-channel function generator using the Tiva C Series TM4C123GH6PM microcontroller. It outputs various analog waveforms by interfacing with the LTC1661 DAC (Digital-to-Analog Converter) and provides user interaction through a Python GUI and onboard hardware switches.

Features
Waveforms Generated:

Sine wave

Square wave

Triangular wave

Sawtooth wave

Adjustable Parameters:

Frequency

Amplitude

Interface:

UART for waveform configuration via Python GUI

SPI for DAC communication

Hardware switch (SW1) to change waveform type

LED indicators for waveform type

Kentec QVGA Display (Enhanced to support 2-channel output)

Methodology
Lookup Table (LUT) Generation:

LUTs for sine, square, triangular, and sawtooth waves are created using MATLAB.

Each LUT contains 1024 samples, each 12-bit wide.

DAC Communication:

Values from the LUT are transmitted using SPI to the LTC1661 DAC.

SPI uses:

SCLK: Clock signal

DOUT: Data signal

~CS: Chip select

Two DACs are used: one for waveform output and one for amplitude control.

User Interaction:

Switch SW1 triggers waveform change and is accompanied by LED indicators:

Red: Sine

Blue: Triangular

Green: Sawtooth

Green + Blue: Square

Python GUI allows dynamic control of waveform type, frequency, and amplitude via UART.

Hardware Setup
Tiva C TM4C123GH6PM

LTC1661 DAC x2

Kentec QVGA Display

LED indicators and switch inputs

Dependencies
TivaWare Library for TM4C123GH6PM

Python (for GUI)

UART and SPI peripheral drivers

MATLAB (for LUT generation, optional)

How to Run
Flash the microcontroller with the embedded code.

Connect the DACs as described.

Use Python GUI to set waveform parameters via UART.

View output on oscilloscope or Kentec display.

Press SW1 to toggle waveform types.

Acknowledgment
Code for the Kentec display was adapted and enhanced from a senior’s previous project.
