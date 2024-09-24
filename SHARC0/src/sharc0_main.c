/**
 * Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <assert.h>

/* CCES includes */
#include <services/int/adi_sec.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

/* Simple driver includes */
#include "spi_simple.h"
#include "twi_simple.h"
#include "sport_simple.h"
#include "uart_simple.h"
#include "uart_stdio.h"

/* Simple service includes */
#include "cpu_load.h"
#include "syslog.h"
#include "fs_devman.h"
#include "fs_devio.h"
#include "fs_dev_fatfs.h"
#include "fs_dev_spiffs.h"

/* oss-services includes */
#include "shell.h"
#include "umm_malloc.h"
#include "spiffs.h"

/* Project includes */
#include "context.h"
#include "init.h"
#include "clocks.h"
#include "util.h"
#include "vu_audio.h"
#include "a2b_slave.h"
#include "clock_domain.h"
#include "spiffs_fs.h"
#include "cpu_load.h"
#include "task_cfg.h"
#include "ss_init.h"
#include "a2b_irq.h"
#include "pushbutton.h"

/* Application context */
APP_CONTEXT mainAppContext;

/***********************************************************************
 * Shell console I/O functions
 **********************************************************************/
static void term_out( char data, void *usr )
{
    putc(data, stdout); fflush(stdout);
}

static int term_in( int mode, void *usr )
{
    int c;
    int timeout;

    if (mode == TERM_INPUT_DONT_WAIT) {
        timeout = STDIO_TIMEOUT_NONE;
    } else if (mode == TERM_INPUT_WAIT) {
        timeout = STDIO_TIMEOUT_INF;
    } else {
        timeout = mode / 1000;
    }

    uart_stdio_set_read_timeout(timeout);

    if ((c = getc(stdin)) == EOF) {
        return(-1);
    }

    return(c);
}

/***********************************************************************
 * CPU idle time / High precision timestamp functions
 **********************************************************************/
uint32_t getTimeStamp(void)
{
    uint32_t timeStamp;
    timeStamp = *pREG_CGU0_TSCOUNT0;
    return timeStamp;
}

void taskSwitchHook(void *taskHandle)
{
    cpuLoadtaskSwitchHook(taskHandle);
}

uint32_t elapsedTimeMs(uint32_t elapsed)
{
    return(((1000ULL) * (uint64_t)elapsed) / CGU_TS_CLK);
}

/***********************************************************************
 * Misc application utility functions (util.h)
 **********************************************************************/
void delay(unsigned ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

time_t util_time(time_t *tloc)
{
    APP_CONTEXT *context = &mainAppContext;
    time_t t;

    vTaskSuspendAll();
    t = context->now / configTICK_RATE_HZ;
    xTaskResumeAll();

    if (tloc) {
        *tloc = t;
    }

    return(t);
}

uint32_t rtosTimeMs()
{
    return(xTaskGetTickCount() * (1000 / configTICK_RATE_HZ));
}


/***********************************************************************
 * Tasks
 **********************************************************************/
void ledFlashOff(TimerHandle_t xTimer)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvTimerGetTimerID(xTimer);
    ss_set(context, SS_PIN_ID_LED4, 0);
}

/* Background housekeeping task */
static portTASK_FUNCTION( houseKeepingTask, pvParameters )
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    TickType_t flashRate, lastFlashTime, clk, lastClk;
    TimerHandle_t ledFlashTimer;
    bool calcLoad;

    /* Configure the LED to flash at a 1Hz rate */
    flashRate = pdMS_TO_TICKS(1000);
    lastFlashTime = xTaskGetTickCount();
    lastClk = xTaskGetTickCount();

    /* Calculate the system load every other cycle */
    calcLoad = false;

    /* Initialize the LED flash off timer */
    ledFlashTimer = xTimerCreate(
        "ledFlashTimer", pdMS_TO_TICKS(25), pdFALSE, context, ledFlashOff
    );

    /* Spin forever doing houseKeeping tasks */
    while (1) {

        /* Calculate the system load */
        if (calcLoad) {
            cpuLoadCalculateLoad(NULL);
            calcLoad = false;
        } else {
            calcLoad = true;
        }

        /* Flash the LED */
        ss_set(context, SS_PIN_ID_LED4, 1);
        xTimerStart(ledFlashTimer, portMAX_DELAY);

        clk = xTaskGetTickCount();
        vTaskSuspendAll();
        context->now += (uint64_t)(clk - lastClk);
        xTaskResumeAll();
        lastClk = clk;

        /* Sleep for a while */
        vTaskDelayUntil(&lastFlashTime, flashRate);
    }
}

/***********************************************************************
 * Application defaults
 **********************************************************************/
static void setAppDefaults(APP_CFG *cfg)
{
    cfg->a2bI2CAddr = DEFAULT_A2B_I2C_ADDR;
}

/***********************************************************************
 * Startup
 **********************************************************************/
static void execShellCmdFile(SHELL_CONTEXT *shell)
{
    FILE *f = NULL;
    char *name = NULL;
    char cmd[32];

    name = SPIFFS_VOL_NAME "shell.cmd";
    f = fopen(name, "r");

    if (f) {
        fclose(f);
        cmd[0] = '\0';
        strcat(cmd, "run "); strcat(cmd, name);
        shell_exec(shell, cmd);
    }
}

/* System startup task -> background shell task */
static portTASK_FUNCTION( startupTask, pvParameters )
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    SPI_SIMPLE_RESULT spiResult;
    TWI_SIMPLE_RESULT twiResult;
    SPORT_SIMPLE_RESULT sportResult;
    FS_DEVMAN_DEVICE *device;
    FS_DEVMAN_RESULT fsdResult;
    s32_t spiffsResult;

    /* Log the core clock frequency */
    syslog_printf("CPU Core Clock: %u MHz", (unsigned)(context->cclk / 1000000));

    /* Load default configuration */
    setAppDefaults(&context->cfg);

    /* Initialize the CPU load module. */
    cpuLoadInit(getTimeStamp, CGU_TS_CLK);

    /* Initialize the simple SPI driver */
    spiResult = spi_init();

    /* Initialize the simple TWI driver */
    twiResult = twi_init();

    /* Initialize the simple SPORT driver */
    sportResult = sport_init();

    /* Intialize the filesystem device manager */
    fs_devman_init();

    /* Intialize the filesystem device I/O layer */
    fs_devio_init();

#if 0
    /* Open up a global device handle for TWI0 @ 400KHz */
    twiResult = twi_open(TWI0, &context->twi0Handle);
    if (twiResult != TWI_SIMPLE_SUCCESS) {
        syslog_print("Could not open TWI0 device handle!");
        return;
    }
    twi_setSpeed(context->twi0Handle, TWI_SIMPLE_SPEED_400);

    /* Open up a global device handle for TWI1 @ 400KHz */
    twiResult = twi_open(TWI1, &context->twi1Handle);
    if (twiResult != TWI_SIMPLE_SUCCESS) {
        syslog_print("Could not open TWI1 device handle!");
        return;
    }
    twi_setSpeed(context->twi1Handle, TWI_SIMPLE_SPEED_400);
#endif

    /* Open up a global device handle for TWI2 @ 400KHz */
    twiResult = twi_open(TWI2, &context->twi2Handle);
    if (twiResult != TWI_SIMPLE_SUCCESS) {
        syslog_print("Could not open TWI2 device handle!");
        return;
    }
    twi_setSpeed(context->twi2Handle, TWI_SIMPLE_SPEED_400);

    /* Set the adau1962, adau1979, and soft switch device handles to TWI2 */
    context->adau1962TwiHandle = context->twi2Handle;
    context->adau1979TwiHandle = context->twi2Handle;
    context->softSwitchHandle = context->twi2Handle;
    context->a2bTwiHandle = context->twi2Handle;
    context->si5356Handle = context->twi2Handle;

    /* Initialize the soft switches */
    ss_init(context);

    /*
     * Get the SOMCRR Version.  This function also sets compatible default
     * soft switch values for Rev D SOMCRR boards.
     */
    context->SoMCRRVersion = somcrr_hw_version(context);

    /* Initialize the flash */
    flash_init(context);

    /* Initialize the SPIFFS filesystem */
    context->spiffsHandle = umm_calloc(1, sizeof(*context->spiffsHandle));
    spiffsResult = spiffs_mount(context->spiffsHandle, context->flashHandle);
    if (spiffsResult == SPIFFS_OK) {
        device = fs_dev_spiffs_device();
        fsdResult = fs_devman_register(SPIFFS_VOL_NAME, device, context->spiffsHandle);
        fsdResult = fs_devman_set_default(SPIFFS_VOL_NAME);
    } else {
        syslog_print("SPIFFS mount error, reformat via command line\n");
    }

    /* Initialize the audio routing table */
    audio_routing_init(context);

    /* Initialize the vu audio module */
    vu_audio_init(context);

    /* Probe for A2B installed in J10 */
    context->a2bPresent = a2b_probe(context);

    /* Configure A2B interrupt dispatcher */
    a2b_irq_init(context);

#if 0
    /* Enable A2B HW IRQ */
    a2b_pint_init(context);
#endif

    /* Configure A2B slave mode handler */
    a2b_slave_init(context);

    /* Disable main MCLK/BCLK */
    disable_mclk(context);

    /* Initialize main MCLK/BCLK */
    mclk_init(context);

    /* Initialize the ADAU1979 ADC */
    adau1979_init(context);

    /* Initialize the ADAU1962 DAC */
    adau1962_init(context);

    /* Initialize the SPDIF I/O */
    spdif_init(context);

    /* Initialize A2B (J10) in master mode */
    a2b_master_init(context);

    /* Initialize the various audio clock domains */
    clock_domain_init(context);

    /* Enable main MCLK/BCLK for a synchronous start */
    enable_mclk(context);

    /* Get the idle task handle */
    context->idleTaskHandle = xTaskGetIdleTaskHandle();

    /* Start the housekeeping tasks */
    xTaskCreate( houseKeepingTask, "HouseKeepingTask", GENERIC_TASK_STACK_SIZE,
        context, HOUSEKEEPING_PRIORITY, &context->houseKeepingTaskHandle );
    xTaskCreate( pushButtonTask, "PushbuttonTask", GENERIC_TASK_STACK_SIZE,
        context, HOUSEKEEPING_PRIORITY, &context->pushButtonTaskHandle );

    /* Lower the startup task priority for the shell */
    vTaskPrioritySet( NULL, STARTUP_TASK_LOW_PRIORITY);

    /* Initialize the shell */
    shell_init(&context->shell, term_out, term_in, SHELL_MODE_BLOCKING, NULL);

    /* Execute shell initialization command file */
    execShellCmdFile(&context->shell);

    /* Drop into the shell */
    while (1) {
        shell_start(&context->shell);
    }
}

int main(int argc, char *argv[])
{
    APP_CONTEXT *context = &mainAppContext;
    UART_SIMPLE_RESULT uartResult;

    /* Initialize the application context */
    memset(context, 0, sizeof(*context));

    /*
     * Our time starts at the FAT epoch (1-Jan-1980) which is
     * 10 years + 2 days after the UNIX epoch of 1-Jan-1970.
     * The 1980 epoch keeps FatFS happy.  See get_fattime() in diskio.c.
     *
     * https://www.timeanddate.com/date/timeduration.html
     */
    context->now = (uint64_t)315532800 * (uint64_t)configTICK_RATE_HZ;

    /* Initialize system clocks */
    system_clk_init(&context->cclk);

    /* Enable the CGU timestamp */
    cgu_ts_init();

    /* Initialize the SEC */
    adi_sec_Init();

    /* Initialize GPIO */
    gpio_init();

    /* Init the system heaps */
    umm_heap_init();

    /* Init the system logger */
    syslog_init();

    /* Initialize the simple UART driver */
    uartResult = uart_init();

    /* Open UART0 as the console device (115200,N,8,1) */
    uartResult = uart_open(UART0, &context->stdioHandle);
    uart_setProtocol(context->stdioHandle,
        UART_SIMPLE_BAUD_115200, UART_SIMPLE_8BIT,
        UART_SIMPLE_PARITY_DISABLE, UART_SIMPLE_STOP_BITS1
    );

    /* Initialize the UART stdio driver with the console device */
    uart_stdio_init(context->stdioHandle);

    /* Init the rest of the system and launch the remaining tasks */
    xTaskCreate( startupTask, "StartupTask", STARTUP_TASK_STACK_SIZE,
        context, STARTUP_TASK_HIGH_PRIORITY, &context->startupTaskHandle );

    /* Start the scheduler. */
    vTaskStartScheduler();

    return(0);
}

/*-----------------------------------------------------------
 * FreeRTOS critical error and debugging hooks
 *-----------------------------------------------------------*/
void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
    ( void ) pcFile;
    ( void ) ulLine;

    /* Disable interrupts so the tick interrupt stops executing, then sit in a loop
    so execution does not move past the line that failed the assertion. */
    taskDISABLE_INTERRUPTS();
    while (1);
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    while (1);
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* Run time allocation failure checking is performed if
    configUSE_MALLOC_FAILED_HOOK is defined.  This hook
    function is called if an allocation failure is detected. */
    taskDISABLE_INTERRUPTS();
    while (1);
}

/*-----------------------------------------------------------*/
