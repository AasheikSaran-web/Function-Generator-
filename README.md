Function Generator using Tiva C TM4C123GH6PM
Authors
Hariram

Aasheik Saran

🧠 Overview
This is a 2-channel Function Generator project using the Tiva C Series TM4C123GH6PM microcontroller. It interfaces with two LTC1661 DACs to produce analog waveforms. Users can select waveform type, frequency, and amplitude using a Python GUI or onboard switch SW1, with status indicated by LEDs.

🔧 Features
Waveform Types:

Sine

Square

Triangular

Sawtooth

Adjustable Parameters:

Frequency

Amplitude

User Interfaces:

UART (Python GUI)

SW1 (Toggling waveform types)

LED Indicators (Waveform status)

Kentec Display (2-channel visual output)

⚙️ Methodology
Lookup Table: 1024 12-bit samples per waveform (generated using MATLAB)

SPI Communication: Used to send LUT values to LTC1661 DAC

Waveform Switching: Detected via GPIO interrupt on SW1

Amplitude Control: Managed by second DAC

Python GUI: Sends waveform settings via UART

🔌 Hardware Components
Component	Description
Tiva C TM4C123GH6PM	ARM Cortex-M4 Microcontroller
LTC1661 DAC (x2)	12-bit DAC for waveform & amplitude
Kentec Display	QVGA graphical output display
LEDs	Red, Green, Blue – waveform indicators
SW1 Button	Used to switch waveform types

💻 Embedded Code Setup
Program written in C using TivaWare libraries. Uses SPI, UART, and GPIO.

Directory Structure
ruby
Copy
Edit
/Firmware
  ├── main.c
  ├── waveform.c
  ├── dac.c
  ├── spi_config.c
  ├── uart_gui.c
  └── tivaware/ (include TivaWare driverlib)
How to Build & Flash
Open Code Composer Studio

Import project

Connect Tiva C LaunchPad

Build the project

Flash to the board

🖥️ Python GUI
A GUI is used to send waveform parameters (type, frequency, amplitude) to the Tiva board.

GUI Requirements
pyserial

tkinter (for GUI)

Run the GUI
bash
Copy
Edit
python3 function_generator_gui.py
GUI Features
Select waveform type

Input frequency (Hz)

Input amplitude (0–4095 range)

Press "Send" to transmit via UART

📡 UART Protocol
Format: TYPE,FREQ,AMPL\n

Example: SINE,1000,2048\n

Parsed in real-time by the TM4C microcontroller.

🧾 Circuit Schematic (Text Overview)
DAC1 (Waveform Output):

CS: GPIO pin X

DIN: SSI_TX

CLK: SSI_CLK

DAC2 (Amplitude Control):

CS: GPIO pin Y

UART:

TX: USB Debug UART to PC

SW1: GPIO (Interrupt-enabled)

LEDs: GPIOs (for waveform indication)

Kentec Display: SPI + Control lines (as per BoosterPack spec)

(You can attach a KiCad or Fritzing diagram for visual reference)

📺 Kentec Display
Enhanced from senior batch code

Displays two waveform channels simultaneously

Output can be routed to oscilloscope via DAC output pins

🧠 Future Work
Frequency sweep

Phase modulation

Signal mixing

Saving/recalling custom waveform presets

🙏 Acknowledgments
Thanks to seniors for display codebase.

MATLAB was used for LUT generation.

