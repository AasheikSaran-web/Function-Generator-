#AASHEIK SARAN, IISC BANGALORE

#include <stdint.h>
#include <waveforms.h> //Contains the lookup table for different waveforms
#include "inc/tm4c123gh6pm.h"
#include <driverlib/sysctl.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include "./inc/tm4c123gh6pm.h"
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/interrupt.h>
#include <strings.h>
#include <stdint.h>
#include <inc\tm4c123gh6pm.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include "./inc/tm4c123gh6pm.h"
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include "driverlib/timer.h"
#include <stdint.h>
#include <inc\tm4c123gh6pm.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include "./inc/tm4c123gh6pm.h"
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include "driverlib/timer.h"
#define BUFFER_SIZE 30
#define UART_BAUD_RATE 115200

#define SPACE ' '
#define NULL_CHAR '\0'
#define ENTER '\r'
#define BACKSPACE '\b'

char command[BUFFER_SIZE] = {NULL_CHAR};
int commandIndex = 0;



// Function generator variables

volatile uint16_t function_frequency = 1; // CURRENT FREQUENCY SEND BY USER IN UART which we take as phase increment
volatile uint16_t function_amplitude = 1; // CURRENT AMPLITUDE  SEND BY USER IN UART which we take as amplitude data
volatile uint16_t function_frequency_PAST = 1; // PAST FREQUENCY SEND BY USER IN UART which we take as phase increment
volatile uint16_t function_amplitude_PAST = 1; // PAST AMPLITUDE  SEND BY USER IN UART which we take as amplitude data
volatile uint16_t adc_amp_past=0;//past value of pot and current values is we reading it directly from register
volatile uint16_t adc_phase_past=0;//past phase from adc




void sendUARTMessage(const char *message);
void clearBuffer();
int validate_function_command();
void run_command();
void initSystem();
void UART0_Handler(void);
void verification_Command();



void DisableInterrupts(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);
void GPIOPort_Init(void);
void GPIOPortF_Handler(void);
void Systick_Handler(void);
void delay_32us();
int flag=0,k=0,x;
uint16_t phase=0;
uint16_t phase_increment=1;
uint16_t result=1;
int amp =1023;//5v
int f=10;                           /* The frequency right now is 10 Hz; so, we must send 1 sample in 100/120 ms */
extern int wt[4][1024];              /* The waveform table is initialized as an external variable */

void Systick_handler(void)
{   int data1;
    int data2;
    uint16_t adc1;
    uint16_t adc2;

    // Start conversions for both ADC0 and ADC1
    ADC0_PSSI_R |= 0x08;          // Start ADC0 Sequence 3
    ADC1_PSSI_R |= 0x08;          // Start ADC1 Sequence 3

    //channel 1
    data1 = wt[flag][phase>>6]|0xA000;                 /* Append A000 as the control word for LTC1661*/

    GPIO_PORTB_DATA_R &= ~0x04;     /* Assert Slave Select low, as it is active low */
    while((SSI1_SR_R & 2) == 0);    /* Wait until transmit FIFO not full */
    SSI1_DR_R = data1 >> 8;          /* Transmit higher byte */

    while((SSI1_SR_R & 2) == 0);    /* Wait until FIFO not full */
    SSI1_DR_R = data1 & 0xFF;        /* Transmit low byte */

    while(SSI1_SR_R & 0x10);        /* Wait until transmit complete */
    GPIO_PORTB_DATA_R |= 0x04;      /* Keep Slave Select idle high */
    data2 = (amp)|0x9000;                 /* Append 9000 as the control word for LTC1661 Amp contol DAC*/
    //channel 2 amplitude control
    GPIO_PORTB_DATA_R &= ~0x02;     /* Assert Slave Select low, as it is active low */
    while((SSI1_SR_R & 2) == 0);    /* Wait until transmit FIFO not full */
    SSI1_DR_R = data2 >> 8;          /* Transmit higher byte */

    while((SSI1_SR_R & 2) == 0);    /* Wait until FIFO not full */
    SSI1_DR_R = data2 & 0xFF;        /* Transmit low byte */

    while(SSI1_SR_R & 0x10);        /* Wait until transmit complete */
    GPIO_PORTB_DATA_R |= 0x02;      /* Keep Slave Select idle high */


    // Wait for both conversions to complete
    while((ADC0_RIS_R & 0x08) == 0); // Wait for ADC0 conversion
    while((ADC1_RIS_R & 0x08) == 0); // Wait for ADC1 conversion


    adc1 = ADC0_SSFIFO3_R ;  //  (PE3)
    adc2 = ADC1_SSFIFO3_R;  //  (PE2)

    // Clear completion flags
    ADC0_ISC_R = 0x08;            // Clear ADC0 flag
    ADC1_ISC_R = 0x08;            // Clear ADC1 flag
    //decision tree
    //phase
    if (adc_phase_past!=(adc1|0x3))//frequency control pot value changes
    {
        phase_increment=adc1|0x3;
        adc_phase_past=adc1|0x3;

    }
    if (function_frequency_PAST!=function_frequency)//when user set frequency in uart
    {
        phase_increment=function_frequency;
        function_frequency_PAST=function_frequency;
    }
    phase+=phase_increment;
    //amplitude
    if (adc_amp_past!=(adc2|0x3))//amplitude control pot value changes
     {
         amp=adc2|0x3;
         adc_phase_past=adc2|0x3;

     }
     if (function_amplitude_PAST!=function_amplitude)//when user set frequency in uart
     {
         amp=function_amplitude;
         function_frequency_PAST=function_amplitude;
     }

}
void int_systick_timer(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // Set system clock to 16 MHz in main function

    // Configure SysTick Timer
    SysTickPeriodSet(800); // 1 second interrupt

    SysTickIntRegister(Systick_handler); // Register SysTick ISR
    SysTickIntEnable(); // Enable SysTick interrupt
    SysTickEnable(); // Enable SysTick timer
}

void GPIOPort_Init(void)
{
    volatile unsigned long del;

    SYSCTL_RCGC2_R|=0x0000002F;     /* For assigning to the variable del*/
    SYSCTL_RCGCSSI_R |= 2;          /* Enable clock to SSI1 */
    SYSCTL_RCGCGPIO_R |= 0xFF;      /* Enable clock to all GPIOs-we'll be using Ports B,D,F */
    del = SYSCTL_RCGC2_R;           /* Allow time for clock to start */

    /* Configure PORTD 1 and 3 for SSI1 clock and Tx respectively */
    GPIO_PORTD_AMSEL_R &= ~0x09;    /* Disable analog for these pins */
    GPIO_PORTD_DEN_R |= 0x09;       /* And make them digital */
    GPIO_PORTD_AFSEL_R |= 0x09;     /* Enable alternate function */
    GPIO_PORTD_PCTL_R &= ~0x0000F00F; /* Assign pins to SSI1 */
    GPIO_PORTD_PCTL_R |= 0x00002002;  /* Assign pins to SSI1 */

    GPIO_PORTF_LOCK_R = 0x4C4F434B; /* Unlock GPIO PortF */
    GPIO_PORTF_CR_R = 0x1F;         /* Allow changes to PF4-0 */
    GPIO_PORTF_AMSEL_R = 0x00;      /* Disable analog on PF */
    GPIO_PORTF_PCTL_R = 0x00000000; /* PCTL GPIO on PF4-0 */
    GPIO_PORTF_DIR_R = 0x0E;        /* PF4,PF0 in, PF3-1 out-we'll be using switches for changing waveform type and frequency */
    GPIO_PORTF_AFSEL_R = 0x00;      /* Disable alt funct on PF */
    GPIO_PORTF_PUR_R = 0x11;        /* Enable pull-up on PF0 and PF4 */
    GPIO_PORTF_DEN_R = 0x1F;        /* Enable digital I/O on PF4-0 */

    GPIO_PORTF_IS_R = 0x00;         /*  PF0-4 is edge-sensitive */
    GPIO_PORTF_IBE_R = 0x00;        /*  PF0-4 is not both edges */
    GPIO_PORTF_IEV_R= 0x11;         /*  PF0-4 rising edge event */
    GPIO_PORTF_ICR_R = 0xFF;        /*  Clear interrupt flags */
    GPIO_PORTF_IM_R |= 0x11;        /*  Arm interrupt on PF4 and PF0 */
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF1FFFFF) | 0x00200000; /*  Priority 1 */
    NVIC_EN0_R = 0x40000000;        /*  Enable interrupt 30 in NVIC */
    GPIO_PORTF_DATA_R |= 0x02;       /* Initially, red light on-we'll get a sine wave output */

    /*PB2 is being used as the slave select line for the ADC*/
    GPIO_PORTB_LOCK_R = 0x4C4F434B; /* Unlock GPIO PortB */
    GPIO_PORTB_CR_R = 0xFF;         /* Allow changes */
    GPIO_PORTB_DIR_R = 0xFF;        /* PB out */
    GPIO_PORTB_DEN_R = 0xFF;        /* Enable digital I/O on PB */

    /* SPI Master, POL = 0, PHA = 0, clock = 4 MHz, 8 bit data */
    SSI1_CR1_R = 0;                 /* Disable SSI and make it master */
    SSI1_CC_R = 0;                  /* Use system clock */
    SSI1_CPSR_R = 2;                /* Prescaler divided by 2 */
    SSI1_CR0_R = 0x0007;            /* 8 MHz SSI clock, SPI mode, 8 bit data */
    SSI1_CR1_R |= 2;                /* Enable SSI1 */


}

void int_adc(void) {
    // ADC Initialization: ADC0 on PE3 (AIN0), ADC1 on PE2 (AIN1)
    SYSCTL_RCGCADC_R |= 0x03;      // Enable ADC0 and ADC1 clocks
    SYSCTL_RCGCGPIO_R |= 0x10;     // Enable Port E clock
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)); // Wait for Port E ready

    // Configure PE3 (AIN0 for ADC0) and PE2 (AIN1 for ADC1)
    GPIO_PORTE_DIR_R &= ~0x0C;     // PE3, PE2 as inputs
    GPIO_PORTE_AFSEL_R |= 0x0C;    // Enable alternate function for PE3, PE2
    GPIO_PORTE_DEN_R &= ~0x0C;     // Disable digital function for PE3, PE2
    GPIO_PORTE_AMSEL_R |= 0x0C;    // Enable analog function for PE3, PE2

    // Configure ADC0 Sequence 3 (AIN0, PE3)
    ADC0_ACTSS_R &= ~0x08;         // Disable SS3 during configuration
    ADC0_EMUX_R &= ~0xF000;        // Software trigger conversion
    ADC0_SSMUX3_R = 0;            // Channel 0 (AIN0)
    ADC0_SSCTL3_R |= 0x06;        // Take one sample, set flag
    ADC0_ACTSS_R |= 0x08;         // Enable SS3 after configuration

    // Configure ADC1 Sequence 3 (AIN1, PE2)
    ADC1_ACTSS_R &= ~0x08;         // Disable SS3 during configuration
    ADC1_EMUX_R &= ~0xF000;        // Software trigger conversion
    ADC1_SSMUX3_R = 1;            // Channel 1 (AIN1)
    ADC1_SSCTL3_R |= 0x06;        // Take one sample, set flag
    ADC1_ACTSS_R |= 0x08;         // Enable SS3 after configuration
}

void GPIOPortF_Handler(void)
{
    volatile int readback;

    /* Press SW1(PF4) for changing frequency and SW2(PF0) for changing the flag, which decides the type of waveform */
    if (GPIO_PORTF_RIS_R & 0x10){   /* Checking if the interrupt came from SW1 */
        f+=1.00;                   /* Increasing f by 10 increases frequency by almost 6 Hz */
        GPIO_PORTF_ICR_R=0xFF;      /* Clear interrupt */
    }
    else if (GPIO_PORTF_RIS_R & 0x01){ /* Checking if the interrupt came from SW2 */
        k=0;                        /* When the waveform is changed, we start from the 1st value in the LUT for that waveform */
        flag++;                     /* Increment flag to move on to the next waveform */
        if (flag==4)
            flag=0;
        GPIO_PORTF_ICR_R=0xFF;      /* Clear interrupt */
        switch(flag){
        case 0:
            GPIO_PORTF_DATA_R=0x02;
            break;
        case 1:
            GPIO_PORTF_DATA_R=0x04;
            break;
        case 2:
            GPIO_PORTF_DATA_R=0x08;
            break;
        case 3:
            GPIO_PORTF_DATA_R=0x06;
            break;
        }

    }
}
void verification_Command()
{
    int i, j = 0;
    char temp[BUFFER_SIZE] = {0};
    for (i = 0; command[i] != NULL_CHAR && i < BUFFER_SIZE; i++)
    {
        if (isalpha(command[i]) || isdigit(command[i]) || command[i] == SPACE || command[i] == '.')
        {
            temp[j++] = command[i];
        }
    }
    temp[j] = NULL_CHAR;
    strcpy(command, temp);
}

int validate_function_command()
{
    char entered_wave[16];  // Buffer to hold waveform type (up to 9 characters + null terminator)


    char *type;
    int frequency, amplitude;

    type = strtok(command, " "); // "func"
    type = strtok(NULL, " ");    // "sine"
    if (type) strcpy(entered_wave, type);
    sendUARTMessage(entered_wave);
    char *freq_str = strtok(NULL, " ");
    char *amp_str = strtok(NULL, " ");
    sendUARTMessage(freq_str);
    sendUARTMessage(amp_str);


    if (freq_str && amp_str) {
        frequency = atoi(freq_str);
        amplitude = atoi(amp_str);
    }
    if (strcasecmp(entered_wave,"square")==0)
    {
                        flag = 2;
                        sendUARTMessage("in square \n\r");
                        GPIO_PORTF_DATA_R = 0x08;
    }
    if (strcasecmp(entered_wave, "triangle") == 0) {
                    flag = 1;
                    GPIO_PORTF_DATA_R = 0x04;
    }
    if (strcasecmp(entered_wave, "sine") == 0) {
                    flag = 0;
                    GPIO_PORTF_DATA_R = 0x02;
    }
    if (strcasecmp(entered_wave, "tanh") == 0) {
                    flag = 3;
                    GPIO_PORTF_DATA_R=0x06;
    }
    function_frequency = (frequency << 16) / 20000;  // Convert to fixed-point format
    function_amplitude = (amplitude *4095) / 5000;



    return 1;
}

void run_command()
{
    if (!validate_function_command())
    {
        sendUARTMessage("\n\rGIVEN COMMAND IS INVALID. VALID COMMANDS:\n\r"
                       "[func <sine/square/triangle> <frequency> <amplitude>]\n\r");
    }
    clearBuffer();
}



void clearBuffer()
{
    memset(command, NULL_CHAR, BUFFER_SIZE);
    commandIndex = 0;
}

void sendUARTMessage(const char *message)
{
    while (*message) {
        UARTCharPut(UART0_BASE, *message++);
    }
    UARTCharPut(UART0_BASE, '\n');
    UARTCharPut(UART0_BASE, '\r');
}

void UART0_Handler(void)
{
    uint32_t status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, status);

    while (UARTCharsAvail(UART0_BASE)) {
        char receivedChar = UARTCharGet(UART0_BASE);

        if (receivedChar == ENTER) {
            command[commandIndex] = NULL_CHAR;
            verification_Command();
            sendUARTMessage("\n\rReceived Command: ");
            sendUARTMessage(command);
            sendUARTMessage("\n\r");
            run_command();
            clearBuffer();
            commandIndex = 0;
        }
        else if (receivedChar == BACKSPACE || receivedChar == 0x7F) {
            if (commandIndex > 0) {
                commandIndex--;
                command[commandIndex] = NULL_CHAR;
                UARTCharPut(UART0_BASE, BACKSPACE);
                UARTCharPut(UART0_BASE, SPACE);
                UARTCharPut(UART0_BASE, BACKSPACE);
            }
        }
        else if (isalpha(receivedChar) || isdigit(receivedChar) || receivedChar == SPACE || receivedChar == '.') {
            if (commandIndex < BUFFER_SIZE - 1) {
                command[commandIndex++] = tolower(receivedChar);
                UARTCharPut(UART0_BASE, receivedChar);
            } else {
                UARTCharPut(UART0_BASE, '\a');
            }
        }
    }
}

void init_uart(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0)); // Wait for GPIOA to be ready
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), UART_BAUD_RATE,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(UART0_BASE); // Enable UART FIFO to handle multiple characters
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); // Enable RX and timeout interrupts
    IntPrioritySet(INT_UART0, 0x00); // Set UART0 interrupt priority (optional, adjust as needed)
    IntEnable(INT_UART0);
    UARTEnable(UART0_BASE);
    IntMasterEnable();
}

/* The following function produces a constant delay of 32 us */
void delay_32us()
{
    SYSCTL_RCGCTIMER_R |= 2;        /* Enable clock to Timer Block 1 */

    TIMER1_CTL_R = 0;               /* Disable Timer before initialization */
    TIMER1_CFG_R = 0x04;            /* 16-bit option */
    TIMER1_TAMR_R = 0x02;           /* Periodic mode and down-counter */
    TIMER1_TAILR_R = 512;           /* TimerA interval load value reg */
    TIMER1_ICR_R = 0x1;             /* Clear the TimerA timeout flag */
    TIMER1_CTL_R |= 0x01;           /* Enable Timer A after initialization */

    while ((TIMER1_RIS_R & 0x1) == 0);/* wait for TimerA timeout flag */
}

int main(void)
{
    // Set system clock to 16 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    GPIOPort_Init();               /* Initialize GPIO Ports */
    int_systick_timer();
    int_adc();
    init_uart();
    EnableInterrupts();            /* Enable global Interrupt flag (I) */
    sendUARTMessage("\n\rSystem Initialized Successfully!\n\r");


    while(1) {}
}

void DisableInterrupts(void)
{
    __asm ("    CPSID  I\n");
}

void EnableInterrupts(void)
{
    __asm  ("    CPSIE  I\n");
}

void WaitForInterrupt(void)
{
    __asm  ("    WFI\n");
}
