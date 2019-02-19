/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Swi.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
// #include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
// #include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Driver Header files */
#include <ti/drivers/ADC.h>
#if defined(CC2650DK_7ID) || defined(CC1310DK_7XD)
#endif

#include <ti/drivers/ADCBuf.h>
/* Board Header files */
#include "Board.h"
#define TASKSTACKSIZE_sigfox   1024

/* ADC sample count */
#define ADC_SAMPLE_COUNT  (10)

#define TASKSTACKSIZE   768

Task_Struct task0Struct;
Task_Struct task1Struct;
Task_Struct task2Struct;
Task_Struct task3Struct;

Char task0Stack[TASKSTACKSIZE];
Char task1Stack[TASKSTACKSIZE];
Char task2Stack[TASKSTACKSIZE];
Char task3Stack[TASKSTACKSIZE_sigfox];

/* ADC conversion result variables */
uint16_t adcValue_level;
uint16_t adcValue_turbidity;
uint16_t adcValue_pH;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

#define CMD_CONFIG 0x00
#define CMD_ID 0x39
#define CMD_IDLE 0x58

extern Swi_Handle SWI0;

UART_Handle uart;
UART_Params uartParams;
uint8_t tx_buffer_config[1];
uint8_t rx_buffer_config[1];
uint8_t tx_buffer_data[5];
uint8_t rx_buffer_id[13];

//some variable
int i=0;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

Void taskFxn_sigfox(UArg arg0, UArg arg1) {
            UART_Params_init(&uartParams);
            uartParams.writeDataMode = UART_DATA_BINARY;
            uartParams.readDataMode = UART_DATA_BINARY;
            uartParams.readReturnMode = UART_RETURN_NEWLINE;
            uartParams.readEcho = UART_ECHO_OFF;
            uartParams.baudRate =19200;// baud rate of the interface
            uart = UART_open(Board_UART0, &uartParams);
            if (uart == NULL)
            {
                System_abort("Error opening the UART");
            }

            //Entering the configuration mode
                        tx_buffer_config[0]=CMD_CONFIG;
                        UART_write(uart, &tx_buffer_config, 1);
                        UART_read(uart, &rx_buffer_config, 1);
                        System_printf("sending the request for config mode\n");
                        System_printf("received ACK %d\n",rx_buffer_config[0]);
                        System_flush();

                        //Reading the ID using the Read ID command
                                    tx_buffer_config[0]=CMD_ID;
                                    UART_write(uart, &tx_buffer_config, 1);
                                    UART_read(uart, &rx_buffer_id, 12);
                                    System_printf("sending the request to send ID\n");
                                    System_flush();
                                    for(i=0;i<12;i++){
                                        if (i<4){
                                            System_printf("ID[%d] is %02x\n",i,rx_buffer_id[i]);
                                            System_flush();
                                        }
                                        else {
                                            System_printf("PAC[%d] is %02x\n",i,rx_buffer_id[i]);
                                        }
                                    }
                                    System_flush();

                                    //Sending the Exit Command to leave the configuration mode
                                                tx_buffer_config[0]=CMD_IDLE;
                                                UART_write(uart, &tx_buffer_config, 1);
                                                System_printf("sending the IDLE mode Req\n\n");
                                                System_flush();

                                               while(1)
                                               {
                                                   Water_Level();
                                                   Turbidity();
                                                   pH();

                                                   /* tx_buffer_data[0] --> command
                                                      tx_buffer_data[1] --> Safety
                                                      tx_buffer_data[2] --> turbidity
                                                      tx_buffer_data[3] --> pH
                                                      tx_buffer_data[4] --> water level*/


                                                   // For safe
                                                   if(tx_buffer_data[2] > 4 && tx_buffer_data[3] > 20 && tx_buffer_data[3] < 30){
                                                         System_printf("S\n\n");
                                                         tx_buffer_data[1]=0x01;
                                                   }
                                                   else{
                                                       System_printf("U\n\n");   //For unsafe
                                                       tx_buffer_data[1]=0x02;
                                                   }

                                                   //tx_buffer_data[1]=0x02;


                                                   System_printf("tx_buffer_data[0]: %d\n", tx_buffer_data[0]);
                                                   System_printf("tx_buffer_data[1]: %d\n", tx_buffer_data[1]);
                                                   System_printf("tx_buffer_data[2]: %d\n", tx_buffer_data[2]);
                                                   System_printf("tx_buffer_data[3]: %d\n", tx_buffer_data[3]);
                                                   System_printf("tx_buffer_data[4]: %d\n\n", tx_buffer_data[4]);

                                                   UART_write(uart, &tx_buffer_data, 5);
                                                   System_printf("sending Data \n\n");
                                                   System_flush();
                                                   Task_sleep(12000000);
                                               }
}

/*
 *  ======== taskFxn0 ========
 *  Open an ADC instance and get a sampling result from a one-shot conversion.
 */

//Void taskFxn_Water_Level(UArg arg0, UArg arg1)
Void Water_Level()
{
    //uint16_t     i;
    ADC_Handle   adc;
    ADC_Params   params;
    int_fast16_t res;


    ADC_Params_init(&params);
    adc = ADC_open(Board_ADC0, &params);

    if (adc == NULL) {
        System_abort("Error initializing WATER LEVEL SENSOR\n");
    }
    else {
        System_printf("WATER LEVEL SENSOR initialized\n");
    }

    /* Blocking mode conversion */

        //while(1){

            adcValue_level = 0;
            res = ADC_convert(adc, &adcValue_level);

            System_printf("Water Level: %d\n", adcValue_level);

            if (res == ADC_STATUS_SUCCESS) {
                if (adcValue_level < 0x3E8){
                    //Task_sleep((UInt)arg0);
                    System_printf("Water Level is safe\n\n");
                    tx_buffer_data[4]=0x01;
                    PIN_setOutputValue(ledPinHandle, Board_LED1, 1);
                }

                else {
                    //Task_sleep((UInt)arg0);
                    System_printf("Water Level is approaching limit\n\n");
                    tx_buffer_data[4]=0x02;
                    PIN_setOutputValue(ledPinHandle, Board_LED0, 1);
                }

            }

            else {
                System_printf("Failed to read water level\n");
            }

            ADC_close(adc);

            PIN_setOutputValue(ledPinHandle, Board_LED1, 0);
            PIN_setOutputValue(ledPinHandle, Board_LED0, 0);

            System_flush();

            tx_buffer_data[0]=0x04;
            //Data to be sent

            //UART_write(uart, &tx_buffer_data, 2);
            //System_printf("sending Data \n");
            //System_flush();
        //}
            //Task_sleep(30000);
}

//Void taskFxn_Turbidity(UArg arg0, UArg arg1)
Void Turbidity()
{
    //uint16_t     i;
    ADC_Handle   adc;
    ADC_Params   params;
    int_fast16_t res;
    uint32_t Res, volt;


    ADC_Params_init(&params);
    adc = ADC_open(Board_ADC1, &params);

    if (adc == NULL) {
        System_abort("Error initializing TURBIDITY SENSOR\n");
    }
    else {
        System_printf("TURBIDITY SENSOR initialized\n");
    }

    /* Blocking mode conversion */
        //int i=0;


      // while(1){
            //i++;
            adcValue_turbidity = 0;

            res = ADC_convert(adc, &adcValue_turbidity);
            //adcValue_turbidity = (adcValue_turbidity * 5)/1024;

            Res = ADC_convertRawToMicroVolts(adc, adcValue_turbidity);
            volt = (Res/1000000.0) + 2;

            System_printf("adc: %d and Voltage: %d\n\n", adcValue_turbidity, volt);

            if (res == ADC_STATUS_SUCCESS) {
                if (adcValue_turbidity < 0x3E8){

                    PIN_setOutputValue(ledPinHandle, Board_LED1, 1);
                }

                else {

                    PIN_setOutputValue(ledPinHandle, Board_LED0, 1);
                }

            }

            else {
                System_printf("Failed to read turbidity\n");
            }

            ADC_close(adc);

            PIN_setOutputValue(ledPinHandle, Board_LED1, 0);
            PIN_setOutputValue(ledPinHandle, Board_LED0, 0);

            System_flush();

            adcValue_turbidity=adcValue_turbidity/1000;
            tx_buffer_data[2]=adcValue_turbidity;

}

//Void taskFxn_pH(UArg arg0, UArg arg1)
Void pH()
{
        ADC_Handle   adc;
        ADC_Params   params;
        int_fast16_t res;
        uint32_t Res, volt;

        ADC_Params_init(&params);
           adc = ADC_open(Board_ADC2, &params);

           if (adc == NULL) {
               System_abort("Error initializing pH SENSOR\n");
           }
           else {
               System_printf("pH SENSOR initialized\n");
           }

               adcValue_pH = 0;

               res = ADC_convert(adc, &adcValue_pH);
               Res = ADC_convertRawToMicroVolts(adc, adcValue_pH);
               volt = Res/1000000.0;

               if (res == ADC_STATUS_SUCCESS) {
                   System_printf("adc: %d, Res: %d, Voltage: %d\n\n", adcValue_pH, Res, volt);
               }
               else {
                   System_printf("Failed to read pH\n\n");
               }

               ADC_close(adc);
               System_flush();

               adcValue_pH=adcValue_pH/100;
               tx_buffer_data[3]=adcValue_pH;

}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initADC();
    // Board_initI2C();
    // Board_initSPI();
    Board_initUART();
    // Board_initWatchdog();

    /* Create tasks */

        /* sigfox */
        Task_Params_init(&taskParams);
        taskParams.stackSize = TASKSTACKSIZE_sigfox;
        taskParams.stack = &task3Stack;
        Task_construct(&task3Struct, (Task_FuncPtr)taskFxn_sigfox, &taskParams, NULL);



        /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);


}
