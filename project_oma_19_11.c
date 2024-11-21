/* C Standard library */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"
#include "sensors/mpu9250.h"
#include "buzzer.h"
#include "CC2650STK.h"
/* Task */
#define STACKSIZE 2048
//#define MORSE_TABLE_SIZE 36
void interpret_mpu(void);
//void MpuFxn(PIN_Handle handle, PIN_Id pinId);
void buzzTaskFxn(UArg arg0, UArg arg1);
void uartFxn(UART_Handle uart, void *rxBuf, size_t len);
//void uartFxn(UART_Handle uart);
void buttonFxn(PIN_Handle handle, PIN_Id pinId);
void buttonFxn_pwr(PIN_Handle handle_pwr, PIN_Id pinId_pwr);
void interpretUart(void);
//void buzzing(int duration);
void bootTask(void);
void uartPrint(UART_Handle uart, const char *message);

Char mpuTaskStack[STACKSIZE];
//Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
Char buzzTaskStack[STACKSIZE];

enum main_state {MEASURE_ON=0, MEASURE_OFF, POWER_OFF};
enum main_state main_mode = MEASURE_ON;
enum state {WAITING=1, DATA_READY, SENDING, RECEIVING};
enum state program_state = WAITING;

char move_msg[100];
char uartBuffer[10] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '\0'}; //, '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '\0'}; // Vastaanottopuskuri
float ax, ay, az, gx, gy, gz = 0.0;
uint32_t timestamp = 0;
float ax_sum, ay_sum, az_sum = 0.0;
float ax_prev, ay_prev, az_prev, gx_prev, gy_prev, gz_prev = 0.0;

static PIN_Handle buttonHandle;
static PIN_State buttonState;
//static PIN_Handle ledHandle;
//static PIN_State ledState;
static PIN_Handle buttonHandle_pwr;
static PIN_State buttonState_pwr;

static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

//static PIN_Handle MpuHandle;
//static PIN_State MpuState;

static PIN_Handle hBuzzer;
static PIN_State sBuzzer;
//static PIN_Handle uartHandle;
//static PIN_State uartState;

PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};
PIN_Config buttonConfig_pwr[] = {
   Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};
/*PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};*/
// JTKJ: Teht�v� 1. Lis�� painonappien RTOS-muuttujat ja alustus

static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
/*static PIN_Config MpuConfig[] = {
    Board_MPU_INT | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_DIS | PIN_HYSTERESIS,
    PIN_TERMINATE
};*/

PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};
// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

Void mpu_sensorFxn(UArg arg0, UArg arg1) {
    //float ax, ay, az, gx, gy, gz;
    //PIN_setInterrupt(MpuConfig, PIN_ID(Board_MPU_INT) | PIN_IRQ_POSEDGE);
    I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;
    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }
    // MPU setup and calibration
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();
    // Loop forever
    while (1) {
        //System_printf("MPU...\n");
        //System_flush();
        if (main_mode == MEASURE_ON && program_state == WAITING){
        // MPU ask data
            //System_printf("Kohta mittaus...");
            //System_flush();
            //PIN_setInterrupt(MpuConfig, PIN_ID(Board_MPU_INT) | PIN_IRQ_POSEDGE);
            timestamp = Clock_getTicks();
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
            program_state = DATA_READY;
            interpret_mpu();
            //System_printf("Mitattu...");
            //System_flush();
            //Task_sleep(100000 / Clock_tickPeriod);
        }
        Task_sleep(200000 / Clock_tickPeriod);//TÄHÄN 100000???
    }
    // Program never gets here..
    // MPU close i2c
    // I2C_close(i2cMPU);
    // MPU power off
    // PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);
}

float *ax_po = &ax;
float *ay_po = &ay;
float *az_po = &az;
float *gx_po = &gx;
float *gy_po = &gy;
float *gz_po = &gz;
float *ax_sum_po = &ax_sum;
float *ay_sum_po = &ay_sum;
float *az_sum_po = &az_sum;
float *ax_prev_po = &ax_prev;
float *ay_prev_po = &ay_prev;
float *az_prev_po = &az_prev;
float *gx_prev_po = &gx_prev;
float *gy_prev_po = &gy_prev;
float *gz_prev_po = &gz_prev;

/*Void MpuFxn(PIN_Handle handle, PIN_Id pinId) {
    buzzTaskFxn(0, 0);
    *ax_sum_po = 0.0;
    *ay_sum_po = 0.0;
    *az_sum_po = 0.0;
    program_state = SENDING;
}*/

void interpret_mpu(void){

    //UART_Handle uart;

    if (program_state == DATA_READY){
        /*char mittausdata[70];
        sprintf(mittausdata, "%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", timestamp, *ax_po, *ay_po, *az_po, *gx_po, *gy_po, *gz_po);
        System_printf(mittausdata);//(move_msg);
        System_flush();*/
        if (fabs(*ax_po) > 0.03){
            *ax_sum_po += *ax_po;
        }
        if (fabs(*ay_po) > 0.03){
            *ay_sum_po += *ay_po;
        }
        if (fabs(*az_po+1.0) > 0.02){
            *az_sum_po += fabs(*az_po + 1.0);
        }
        if (((fabs(*ax_sum_po) + fabs(*ay_sum_po)) > 0.06) && (fabs(*az_sum_po) <= 0.02) && (fabs(*ax_prev_po - *ax_po) > 0.01) && (fabs(*ax_prev_po - *ax_po) > 0.01)){ //ax > 2.0 | ax < -2.0){ //= LIUKUU?
            //char move_msg_temp[30] = ".\r\n.\r\n.\r\n \r\n.\r\n-\r\n.\r\n.\r\n \r\n.\r\n.\r\n \r\n-\r\n.\r\n.\r\n \r\n.\r\n \r\n \r\n \r\n"; //="SLIDE"
            //sprintf(move_msg, "%s\n\r", move_msg_temp);
            sprintf(move_msg, ".\r\n\0");
            System_printf("SLIDING\n");
            System_flush();
            //PIN_setInterrupt(MpuConfig, PIN_ID(Board_MPU_INT) | PIN_IRQ_POSEDGE);
            buzzTaskFxn(0, 0);
            //buzzing(1);
            *ax_sum_po = 0.0;
            *ay_sum_po = 0.0;
            *az_sum_po = 0.0;
            program_state = SENDING;
         }else if (*az_sum_po > 0.50){
            //char move_msg_temp[30] = ".\r\n-\r\n-\r\n-\r\n \r\n.\r\n.\r\n-\r\n \r\n-\r\n-\r\n \r\n.\r\n-\r\n-\r\n.\r\n \r\n \r\n \r\n"; //="JUMP"
            //sprintf(move_msg, "%s\n\r", move_msg_temp);
            sprintf(move_msg, "-\r\n\0");
            System_printf("JUMP\n");
            System_flush();
            //PIN_setInterrupt(MpuConfig, PIN_ID(Board_MPU_INT) | PIN_IRQ_POSEDGE);
            buzzTaskFxn(0, 0);
            //buzzing(1);
            *ax_sum_po = 0.0;
            *ay_sum_po = 0.0;
            *az_sum_po = 0.0;
            program_state = SENDING;
         }
         //else if (fabs(*gx_po) > 45.0 || fabs(*gy_po) > 45.0 || fabs(*gz_po) > 45.0) {
         //    sprintf(move_msg, " \r\n\0");
         //    program_state = SENDING;
         //}
         else{
            program_state = WAITING;
        }
        *ax_prev_po = *ax_po;
        *ay_prev_po = *ay_po;
        *az_prev_po = *az_po;
    }
        //program_state = SENDING;
        //buzzTaskFxn(0, 0);
        //Task_sleep(1000000 / Clock_tickPeriod);
    //}
}

//TOISESTA NAPISTA PÄÄLLE/POIS...TOTEUTUS PUUTTUU...
void buttonFxn_pwr(PIN_Handle handle_pwr, PIN_Id pinId_pwr){
    if (main_mode < 2){
        main_mode = POWER_OFF;
        bootTask();
    }
    //buzzTaskFxn(0, 0);
    //System_printf("Sammuu...pr:%d...main%d\n", program_state, main_mode);
    //System_flush();
    //BIOS_exit(1);
}
void buttonFxn(PIN_Handle handle, PIN_Id pinId){
    //buzzTaskFxn(0, 0);
    //PIN_setInterrupt(MpuConfig, PIN_ID(Board_MPU_INT) | PIN_IRQ_DIS);
    if (main_mode == MEASURE_ON){
        main_mode = MEASURE_OFF;
    }else if (main_mode == MEASURE_OFF){
        main_mode = MEASURE_ON;
    }
}

void bootTask(void){
    if (main_mode == POWER_OFF){
        buzzTaskFxn(1, 1);
        System_printf("Sammuu...pr:%d...main%d\n", program_state, main_mode);
        System_flush();
        Power_shutdown(1, 100000); //(unsigned int shutdownState, uint32_t shutdownTime)
    }
}

/*void buzzing(int duration){
    if (duration == 1){
        //System_printf("Bz\n");
        //System_flush();
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(2000);
        Task_sleep(20000 / Clock_tickPeriod); //OLI 50000
        buzzerClose();
    }else if (duration == 2){
        //System_printf("Bzzz\n");
        //System_flush();
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(2000);
        Task_sleep(150000 / Clock_tickPeriod); //OLI 50000
        buzzerClose();
    }
}*/
Void buzzTaskFxn(UArg arg0, UArg arg1) {
    //while (1) {
    if (arg0 == 0 && arg1 == 0){
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(2000);
        Task_sleep(25000 / Clock_tickPeriod); //OLI 50000
        buzzerClose();
        //Task_sleep(950000 / Clock_tickPeriod);
    }else if (arg0 == 1 && arg1 == 1){
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(2000);
        Task_sleep(150000 / Clock_tickPeriod); //OLI 50000
        buzzerClose();
    }else if (arg0 == 0 && arg1 == 1){
        Task_sleep(150000 / Clock_tickPeriod);
    }
}
//int count = 0;

void interpretUart(void){
    if (program_state == RECEIVING){
        switch(uartBuffer[0]){
            case '.':
                buzzTaskFxn(0, 0);
                break;
            case '-':
                buzzTaskFxn(1, 1);
                break;
            case ' ':
                Task_sleep(150000 / Clock_tickPeriod);
                break;
            }
    }
    //if (count >= 3){
    program_state = WAITING;
    //}
}

void uartFxn(UART_Handle uart, void *rxBuf, size_t len){
    if (program_state == WAITING || program_state == DATA_READY){
        program_state = RECEIVING;
    }
    //interpretUart();
    //UART_read(uart, uartBuffer, 1);
}

/*void uartFxn2(UART_Handle uart){
    if (program_state == WAITING || program_state == DATA_READY){
        program_state = RECEIVING;
    }
}*/

void uartPrint(UART_Handle uart, const char *message) {
    UART_write(uart, message, strlen(message)+1);
}
        
Void uartTaskFxn(UArg arg0, UArg arg1) {

    char input;
    UART_Handle uart;
    UART_Params uartParams;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_CALLBACK;//UART_MODE_BLOCKING;
    //uartParams.writeMode = UART_MODE_BLOCKING;//TÄTÄ EI OLLUT <-> default
    uartParams.readCallback  = &uartFxn; // Käsittelijäfunktio
    //uartParams.readCallback  = &uartFxn2;
    uartParams.baudRate = 9600; // 9600 baud rate
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; //
    uartParams.stopBits = UART_STOP_ONE; // 1
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
       System_abort("Error opening the UART");
    }

    uartPrint(uart, "UART Initialized\r\n");
    //UART_read(uart, uartBuffer, 1);
    while(1){
       //UART_read(uart, uartBuffer, 1);
       UART_read(uart, &input, 1);
       //sprintf(uartBuffer, "%c", input);

       //char debugMsg[50];
       //sprintf(debugMsg, "Received: %c\n", input);
       //uartPrint(uart, debugMsg);

       if (program_state == RECEIVING){
            interpretUart();

       }else if (program_state == SENDING){
           //const char* morsemsg = getMorse(input);
           //uartPrint(uart, "Sending movement data...\n");
           uartPrint(uart, move_msg);
           //char uartTesting[10];
           //char test1[] = ".\r\n \r\n-\r\n \r\n \r\n \r\n"; // ="et";
           //sprintf(uartTesting, "%s", test1);
           //char e2[] = " \r\n";
           //char e3[] = " \r\n";
           //char e4[] = " \r\n";
           //System_printf("%s##\n", uartTesting);
           //System_flush();
           //UART_write(uart, uartTesting, strlen(uartTesting));
           //UART_write(uart, e2, 1); //strlen(e2));
           //UART_write(uart, e3, 1); //strlen(e3));
           //UART_write(uart, e4, 1); //strlen(e4));

           //UART_write(uart, move_msg, strlen(move_msg));
           program_state = WAITING;
        }
        Task_sleep(200000 / Clock_tickPeriod);
    }
}

/*Void sensorTaskFxn(UArg arg0, UArg arg1) {

    I2C_Handle      i2c;
    I2C_Params      i2cParams;

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    i2c = I2C_open(Board_I2C_TMP, &i2cParams);

    if (i2c == NULL) {
       System_abort("Error Initializing I2C\n");
    }
    I2C_close(i2c);
    //Task_sleep(100000 / Clock_tickPeriod);
    //opt3001_setup(&i2c);
    // JTKJ: Teht�v� 2. Alusta sensorin OPT3001 setup-funktiolla
    //       Laita enne funktiokutsua eteen 100ms viive (Task_sleep)
    while (0) {
        if (main_mode == MEASURE_ON && program_state == WAITING){
          // ambientLight = opt3001_get_data(&i2c);
        // JTKJ: Teht�v� 2. Lue sensorilta dataa ja tulosta se Debug-ikkunaan merkkijonona
           program_state = DATA_READY;
        // JTKJ: Teht�v� 3. Tallenna mittausarvo globaaliin muuttujaan
        //       Muista tilamuutos
        // Just for sanity check for exercise, you can comment this out
            System_printf("sensorTask\n");
            System_flush();
        }
        // Once per second, you can modify this
        Task_sleep(1000000 / Clock_tickPeriod);
    }
}*/

Int main(void) {
    // Task variables
    //Task_Handle sensorTaskHandle;
    //Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
    Task_Handle task;
    Task_Params taskParams;
    Task_Handle buzzTask;
    Task_Params buzzTaskParams;
    // Initialize board
    Board_initGeneral();
    Board_initUART();
    Board_initI2C();
        // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("MPU pin open failed!");
    }
    /*MpuHandle = PIN_open(&MpuState, MpuConfig);
    if (MpuHandle == NULL) {
       System_abort("MPU interrupt pin open failed!");
    }*/
    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
    if (hBuzzer == NULL) {
        System_abort("Buzzer pin open failed!");
    }

    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
        System_abort("Error initializing button pin\n");
    }
    buttonHandle_pwr = PIN_open(&buttonState_pwr, buttonConfig_pwr);
    if (!buttonHandle_pwr){
        System_abort("Error initializing button powerpin\n");
    }
    /*ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
          System_abort("Error initializing LED pins\n");
       }*/
    if(PIN_registerIntCb(buttonHandle, &buttonFxn) != 0){
        System_abort("Error registering button callback function");
    }
    if(PIN_registerIntCb(buttonHandle_pwr, &buttonFxn_pwr) != 0){
        System_abort("Error registering powerbutton callback function");
    }
    /*if(PIN_registerIntCb(MpuHandle, &MpuFxn) != 0){
        System_abort("Error registering MPU interrupt function");
    }*/

    Task_Params_init(&buzzTaskParams);
    buzzTaskParams.stackSize = STACKSIZE; //NÄISSÄ: taskParams --> buzzTaskParams
    buzzTaskParams.stack = &buzzTaskStack;
    buzzTaskParams.priority=2; //TÄMÄ LISÄTTY
    buzzTask = Task_create((Task_FuncPtr)buzzTaskFxn, &buzzTaskParams, NULL);
    if (buzzTask == NULL) {
      System_abort("Buzzertask create failed!");
    }

    Task_Params_init(&taskParams);
    taskParams.stackSize = STACKSIZE;
    taskParams.stack = &mpuTaskStack;
    taskParams.priority=2;
    task = Task_create((Task_FuncPtr)mpu_sensorFxn, &taskParams, NULL);
    if (task == NULL) {
        System_abort("Task create failed!");
    }
    // Task
    /*Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task create failed!");
    }*/

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create((Task_FuncPtr)uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    System_printf("Hello world!\n");
    System_flush();
    // Start BIOS
    BIOS_start();

    return (0);
}
