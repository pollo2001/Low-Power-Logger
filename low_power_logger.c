/*
  Sanitized Low-Power DMA UART Logger Example
  NOTE: Vendor-specific initialization and register details have been removed
  for NDA compliance. This code demonstrates structure and logic only.
*/

// This firmware demonstrates low-power serial logging using UART_LOW_PWR, RTC_COUNTER, and DMA_ENGINE on the MCU_PLATFORMGG11 microcontroller.
 // The system runs in EM2 low-energy mode and uses interrupt-driven peripherals to minimize CPU usage.
 // RTC_COUNTER is configured in counter mode with a DIV1 prescaler, providing 30.5 µs resolution per tick.
 // A 64-bit variable tracks total elapsed time in microseconds to prevent overflow.
 // Every 1000 ticks (~30.5 ms), LED0 toggles for visual feedback.
 // UART_LOW_PWR0 operates at 9600 baud and is fully interrupt-driven for RX and TX.
 // DMA_ENGINE transfers 24-byte UART RX packets into memory, timestamped with the current RTC_COUNTER counter value.
 // This setup is ideal for logging timestamped ADC data with minimal power and processing overhead.


#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_ldma.h"
#include "em_leuart.h"
#include "em_rtcc.h"
#include "em_core.h"
#include "bsp.h"
#include <stdio.h>

//UART_LOW_PWR PRE-DEFS
#define RX_BUFFER_SIZE  24
#define BAUD_RATE 9600
static uint32_t rxDataReady = 0; //flag for receiver not having data
static volatile char rxBuffer[RX_BUFFER_SIZE]; //24 byte buffer
static char txBuffer[RX_BUFFER_SIZE];
static volatile uint32_t txIndex=0;

//ldma PRE-DEFS
volatile uint8_t dmaRxBuffer[RX_BUFFER_SIZE];
#define DMA_CHANNEL 0 //this channel to receiev in and then move with dma

//RTC_COUNTER PRE-DEFS
volatile bool rtccWakeFlag = false; // RTC_COUNTER interrupt flag
volatile uint32_t sysSeconds = 0;
volatile uint32_t lastLogged = 0;
static const float set_DIV = 30.5 ;//period we want(30.5 micro)

//log entry struct
typedef struct{
  uint8_t rxBuffer[RX_BUFFER_SIZE]; //dealing with bytes from 7 channel ADC
  uint64_t tickTimeStamp;       //time stamp to go with block recieved(30.5 us res)
} ADC_LogEntry;

volatile ADC_LogEntry adcLog;

//global transfer config and descriptors
DMA_ENGINE_TransferCfg_t rxConfig;
DMA_ENGINE_Descriptor_t rxDesc;

//bare minium ldma setup, one channel rx
void setupDMA_ENGINE(void)
{
  DMA_ENGINE_Init_t init = DMA_ENGINE_INIT_DEFAULT;
  DMA_ENGINE_Init(&init);

  DMA_ENGINE_TransferCfg_t rxCfg = DMA_ENGINE_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_UART_LOW_PWR0_RXDATAV);

  DMA_ENGINE_Descriptor_t rxDesc = DMA_ENGINE_DESCRIPTOR_SINGLE_P2M_BYTE(
    &(UART_LOW_PWR0->RXDATA),       // Source: UART_LOW_PWR RX register
    rxBuffer,                 // Destination: local buffer
    RX_BUFFER_SIZE           // Transfer count
  );

  rxDesc.xfer.doneIfs = true; // Fire interrupt when done

  DMA_ENGINE_StartTransfer(DMA_CHANNEL, &rxCfg, &rxDesc);
}

//implement this in
void DMA_ENGINE_IRQHandler(void)
{
  uint32_t flags = DMA_ENGINE_IntGet();
  DMA_ENGINE_IntClear(flags);

  if (flags & DMA_ENGINE_IF_ERROR) while (1);

  // Timestamp the arrival
  adcLog.tickTimeStamp = (uint64_t)RTC_COUNTER_CounterGet() * 30.5;

  // Copy into log buffer
  memcpy(adcLog.rxBuffer, rxBuffer, RX_BUFFER_SIZE); //verfy this is 24 bytes indeed

  // Optional: restart DMA for next chunk, copy of next 24 byte packet
  //DMA_ENGINE_StartTransfer(DMA_CHANNEL, &rxCfg, &rxDesc);
}

//setup LFXO for low power
void setupLFXO(void)
{
    CLOCK_MODULE_LFXOInit_TypeDef lfxoInit = CLOCK_MODULE_LFXOINIT_DEFAULT; //default init
    CLOCK_MODULE_LFXOInit(&lfxoInit); //init lfxo
    CLOCK_MODULE_OscillatorEnable(cmuOsc_LFXO, true, true); //enable lfx
}

//setup GPIO pins for UART_LOW_PWR
void setupGPIO(void)
{
  CLOCK_MODULE_ClockEnable(cmuClock_GPIO, true); //enable gpio clk, for gpio use
  GPIO_PinModeSet(gpioPortC, 15, gpioModeInput, 0);  // RX
  GPIO_PinModeSet(gpioPortC, 14, gpioModePushPull, 1); // TX
}

//setup leuart, 9600 baud 8N! serial
void setupUART_LOW_PWR(void)
{

  //NOTE: High Frequency Low Energy necessary for LE modules!!!
    CLOCK_MODULE_ClockEnable(cmuClock_HFLE, true); //must be set for LE module access

    //lfb clk(low freq bus) allows different peripherals to use lfxo clk
    CLOCK_MODULE_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO); //select lfb clk to be lfxo as the reference clk

    CLOCK_MODULE_ClockEnable(cmuClock_UART_LOW_PWR0, true); //enable leuart clk, to use leuart
    CLOCK_MODULE_ClockDivSet(cmuClock_UART_LOW_PWR0, cmuClkDiv_1); //do not prescale UART_LOW_PWR clock

    //default leuart and enable/init
    UART_LOW_PWR_Init_TypeDef leuartInit = UART_LOW_PWR_INIT_DEFAULT;

    //config, verified in teraterm
    leuartInit.baudrate = BAUD_RATE;
    leuartInit.databits = leuartDatabits8;
    leuartInit.stopbits = leuartStopbits1;
    leuartInit.parity = leuartNoParity;

    UART_LOW_PWR_Init(UART_LOW_PWR0, &leuartInit); //apply the init(update me)

    //route locations to LOC 5, PC14/15
    UART_LOW_PWR0->ROUTEPEN = UART_LOW_PWR_ROUTEPEN_RXPEN | UART_LOW_PWR_ROUTEPEN_TXPEN;
    UART_LOW_PWR0->ROUTELOC0 = (UART_LOW_PWR_ROUTELOC0_RXLOC_LOC5 | UART_LOW_PWR_ROUTELOC0_TXLOC_LOC5);

    //enable interrupts on char data receive or tranSmit
    //enabe interrupt on rx data ping
    //enable interrupt on transmit buffer level
    UART_LOW_PWR_IntEnable(UART_LOW_PWR0, UART_LOW_PWR_IEN_RXDATAV | UART_LOW_PWR_IEN_TXBL); //WHEN BUFFER EMPTY, AND READY TO RECIEVE NEW DATA
    NVIC_EnableIRQ(UART_LOW_PWR0_IRQn); //enable NVIC interrupt for UART_LOW_PWR

    //enable rx and tx after setup
    UART_LOW_PWR0->CMD = leuartEnable;
}

//keep receiving data while there is still data left in rx buffer
//store and set receive flag on '\n' or when buffer is full
void UART_LOW_PWR0_IRQHandler(void)
{
  //rx/tx indices for traversing buffer chars
  static uint32_t rxIndex = 0;

  //RX HANDLER
  //implement the ldma when rx pattern is recognized(in the rx handler for LEUARt)
  //enable ldma here when rx flag set high, disable flag, run ldma process..
  //check interrupt flag
    if (UART_LOW_PWR_IntGet(UART_LOW_PWR0) & UART_LOW_PWR_IF_RXDATAV) //check rx data is available
    {
        char incoming = UART_LOW_PWR_Rx(UART_LOW_PWR0); //store the incmoing char, don't reread

        while (UART_LOW_PWR0->STATUS & UART_LOW_PWR_STATUS_RXDATAV) //while there is incoming data
          {
              //two char spaces for '\n' and '\0', note that UART_LOW_PWR_Rx(UART_LOW_PWR0) is current data char being pointed at
              if((rxIndex < RX_BUFFER_SIZE - 2 ) && (incoming != '\n')) //when there is still room in buffer and not last character
                {
                  rxBuffer[rxIndex++] = incoming; //store byte and increment index of buffer
                }
              else{
                  //assign two known chars, maybe opportunity to use comparator with PRS peripheral in hardware, offloads cpu
                 rxBuffer[rxIndex++] = '\n'; //N-1
                 rxBuffer[rxIndex] = '\0'; //terminate at Nth
                 rxDataReady = 1; //raise flag
                 rxIndex = 0;//reset
                 GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN); //toggle when full
                 break;
              }
          }
      }


    //TX HANDLER, use TXBL transmit buffer level = ready, TXC ONLY FIRES ONCE,
    if (UART_LOW_PWR_IntGet(UART_LOW_PWR0) & UART_LOW_PWR_IF_TXBL){

        UART_LOW_PWR_IntClear(UART_LOW_PWR0, UART_LOW_PWR_IEN_TXBL); //clear

        if(txBuffer[txIndex] != '\0') //check that tx buffer isnt full or last character
            UART_LOW_PWR_Tx(UART_LOW_PWR0, txBuffer[txIndex++]); //trasnmit data
        else {
            UART_LOW_PWR_IntDisable(UART_LOW_PWR0, UART_LOW_PWR_IF_TXBL); //disable tx interruprt once done
            txIndex = 0;
        }
    }
}

void setupRTC_COUNTER(void)
{
    // Enable necessary clocks for low energy domain
    CLOCK_MODULE_ClockEnable(cmuClock_HFLE, true);             // Needed for LE modules
    CLOCK_MODULE_ClockEnable(cmuClock_CORELE, true);           // Core clock for LE domain
    CLOCK_MODULE_ClockSelectSet(cmuClock_LFE, cmuSelect_LFXO);  // Route LFXO to LFE
    CLOCK_MODULE_ClockEnable(cmuClock_RTC_COUNTER, true);              // Enable RTC_COUNTER clock

    // Configure RTC_COUNTER to tick at 1Hz using prescaler
    RTC_COUNTER_Init_TypeDef rtccInit = RTC_COUNTER_INIT_DEFAULT;
    rtccInit.enable = false;                           // Disable until fully configured
    rtccInit.cntMode =   rtccCntModeNormal;            // counter mode, for micro-sec precision, we have scale options here
    rtccInit.presc = rtccCntPresc_1;               // Divide by DIV1 (30.5 us tick)
    rtccInit.prescMode = rtccCntTickPresc;             // Use prescaler for CNTTICK
    rtccInit.cntWrapOnCCV1 = false;                    // No wrap

    //bits (0-8)= days, (9-16)= months, (17-32) = years
    RTC_COUNTER_DateSet((0x25 << 24) | (0x06 << 16) | (0x06 << 8)); // example:  June 6, 2025,
    RTC_COUNTER_TimeSet((0x14 << 24) | (0x00 << 16) | (0x00 << 8)); // 14:00:00    get into uint64 for further precisionin microsecs..

    RTC_COUNTER_Init(&rtccInit);                              // Initialize RTC_COUNTER

    //clear interrupts
    RTC_COUNTER_IntClear(_RTC_COUNTER_IF_MASK);                      // Clear all pending flags
    RTC_COUNTER_IntClear(RTC_COUNTER_IF_CNTTICK);

    RTC_COUNTER_IntEnable(RTC_COUNTER_IF_CNTTICK);                  // Enable CNTTICK interrupt

    NVIC_ClearPendingIRQ(RTC_COUNTER_IRQn);                   // Clear NVIC pending
    NVIC_EnableIRQ(RTC_COUNTER_IRQn);                         // Enable NVIC interrupt

    RTC_COUNTER_Enable(true);                                 // Enable RTC_COUNTER now
}

void setupLED(void){
  GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, 0);  // Debug LED start ON
}

//Init DCDC regulator with kit specific params, UART_LOW_PWR did not work until I Set this up...
void setupDCDC_Reg(void){
  POWER_CTRL_DCDCInit_TypeDef dcdcInit = POWER_CTRL_DCDCINIT_DEFAULT;
  POWER_CTRL_DCDCInit(&dcdcInit);
}


// to config later for other interrupts....
// timestamp inclusion for logging leuart transactions
// schedule data logging for different sensors
// disable peripherals for power management
void RTC_COUNTER_IRQHandler(void)
{
    if(RTC_COUNTER_IntGet() & RTC_COUNTER_IF_CNTTICK)
      {
        RTC_COUNTER_IntClear(RTC_COUNTER_IF_CNTTICK);
        sysSeconds++;
        rtccWakeFlag = true;
      }
}

void formTime(uint8_t *in_time_arr, uint32_t time){

  in_time_arr[0] = (time >> 16) & 0xFF;  //hours: pos 0
  in_time_arr[1] = (time >> 8) & 0xFF;  //minutes: pos 1
  in_time_arr[2]  = time & 0xFF;  //seconds: pos 2
}

void formDate(uint8_t *out_time_arr, uint32_t date){

  out_time_arr[0]  = (date >> 24) & 0xFF;  //year: pos 0
  out_time_arr[1]  = (date >> 16) & 0xFF;  //month: pos 1
  out_time_arr[2]  = (date >> 8) & 0xFF;   //day: pos 2
}

int main(void){
    CHIP_Init(); // chip errata

    setupDCDC_Reg(); //do this first (EM2), STK power management, verify with user manual...link to peripheral demos is in docs
    setupGPIO();
    setupLED();
    setupLFXO();
    setupUART_LOW_PWR();
    setupRTC_COUNTER();

    // Prepare the first message
    const char the_message[] = "Initial Connect\r\n";
   // strncpy(txBuffer, the_message, RX_BUFFER_SIZE - 1);
    txBuffer[RX_BUFFER_SIZE - 1] = '\0';
    txIndex = 0;

    // Start the first character send
    UART_LOW_PWR_IntDisable(UART_LOW_PWR0, UART_LOW_PWR_IEN_TXBL); //disable tx interruprt once done
    UART_LOW_PWR_Tx(UART_LOW_PWR0, txBuffer[txIndex++]);     // start transmission
    UART_LOW_PWR_IntEnable(UART_LOW_PWR0, UART_LOW_PWR_IEN_TXBL);   // enable TX interrupt when buff ready

    while (1)
    {
        if (rxDataReady || rtccWakeFlag)
        {
            rxDataReady = 0; // Clear flag

            if (rtccWakeFlag)
            {
                rtccWakeFlag = false; // Clear tick flag

//               uint8_t current_time[3] = {0};
//               uint8_t current_date [3] = {0};
//
//               //format the dates and time through array
//               formTime(current_time, RTC_COUNTER_TimeGet());
//               formDate(current_date, RTC_COUNTER_DateGet());
//
//               //buffer probably will be full, so this will never print
//               snprintf(txBuffer, RX_BUFFER_SIZE,
//                        "DATE: 20%02d-%02d-%02d TIME: %02d:%02d:%02d\n",
//                        current_date[0], current_date[1], current_date[2],
//                        current_time[0], current_time[1], current_time[2]);



                //data into buffer goes here...ldma
                //move data into memory
                //timstamp this data
                //check full
                //transmit?
                //implement the ldma when rx pattern is recognized(in the rx handler for LEUARt)



                static uint32_t ticker = 0; //trak this guy

                if(ticker++ >= 1000){
                    ticker = 0; //reset, ticks only evey 1000th (30.5 ms)
                    //toggle led 0 for rtcc
                    GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN); //gpio 19 toggle, to veriify on adc to see high state signal
                //snprintf(txBuffer, RX_BUFFER_SIZE, "Time: %.2f ticked \r\n",
                  //       TicktTimeStamp);
               // txBuffer[RX_BUFFER_SIZE - 1] = '\0';
                }
                else continue;

            }

            else
            {
                // Copy received buffer into TX buffer
                strncpy(txBuffer, rxBuffer, RX_BUFFER_SIZE - 1);
                txBuffer[RX_BUFFER_SIZE - 1] = '\0';
            }

            txIndex = 0;
            UART_LOW_PWR_IntDisable(UART_LOW_PWR0, UART_LOW_PWR_IEN_TXBL); // disable TX interrupt while we restart
            UART_LOW_PWR_Tx(UART_LOW_PWR0, txBuffer[txIndex++]);     // start transmission
            UART_LOW_PWR_IntEnable(UART_LOW_PWR0, UART_LOW_PWR_IEN_TXBL);   // enable TX interrupt when buff ready
        }

        POWER_CTRL_EnterEM2(false); // snooze until interrupt, false we dont need to resotre HFCLK states
        //enter with true when using hfclk
    }
    return 0;
}


