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

/* Standard library includes */
#include <string.h>
#include <limits.h>
#include <assert.h>
#include <stdbool.h>

/* ADI service includes */
#include <services/gpio/adi_gpio.h>
#include <services/int/adi_sec.h>
#include <services/tmr/adi_tmr.h>
#include <services/pwr/adi_pwr.h>

/* ADI processor includes */
#include <sru21569.h>

/* Kernel includes */
#include "FreeRTOS.h"
#include "semphr.h"

/* Simple driver includes */
#include "twi_simple.h"
#include "spi_simple.h"
#include "sport_simple.h"
#include "pcg_simple.h"
#include "flash.h"
#include "is25lp512.h"

/* Simple service includes */
#include "adau1962.h"
#include "adau1979.h"
#include "syslog.h"
#include "a2b_to_sport_cfg.h"

/* OSS service includes */
#include "umm_malloc.h"
#include "umm_malloc_heaps.h"

/* Application includes */
#include "context.h"
#include "clocks.h"
#include "init.h"
#include "util.h"
#include "codec_audio.h"
#include "spdif_audio.h"
#include "a2b_audio.h"
#include "route.h"
#include "clock_domain.h"
#include "ss_init.h"

/* This code can be placed in external memory */
#include "external_memory.h"

/***********************************************************************
 * System Clock Initialization
 **********************************************************************/
void system_clk_init(uint32_t *cclk)
{
    ADI_PWR_RESULT ePwrResult;
    uint32_t _cclk,sclk,sclk0,sclk1,dclk,oclk;

    /* Initialize ADI power service */
    ePwrResult = adi_pwr_Init(0, OSC_CLK);

    /* The adi_pwr_SetFreq() API is not available on the 21569.  Fortunately
     * the default values are OK.
     */

    /* Query primary clocks from CGU0 for confirmation */
    ePwrResult = adi_pwr_GetCoreClkFreq(0, &_cclk);
    ePwrResult = adi_pwr_GetSystemFreq(0, &sclk, &sclk0, &sclk1);
    ePwrResult = adi_pwr_GetDDRClkFreq(0, &dclk);
    ePwrResult = adi_pwr_GetOutClkFreq(0, &oclk);

    /* Store the core clock frequency */
    if (cclk) {
        *cclk = _cclk;
    }

    /* Make sure they match clocks.h */
    assert(_cclk == CCLK);
    assert(sclk == SYSCLK);
    assert(sclk0 == SCLK0);
}

/***********************************************************************
 * GPIO / Pin MUX / SRU Initialization
 **********************************************************************/

/* TWI2 GPIO FER bit positions */
#define TWI2_SCL_PORTA_FER   (1 << BITP_PORT_DATA_PX14)
#define TWI2_SDA_PORTA_FER   (1 << BITP_PORT_DATA_PX15)

/* TWI2 GPIO MUX bit positions (two bits per MUX entry */
#define TWI2_SCL_PORTA_MUX   (0 << (BITP_PORT_DATA_PX14 << 1))
#define TWI2_SDA_PORTA_MUX   (0 << (BITP_PORT_DATA_PX15 << 1))

/* SPI2 GPIO FER bit positions (one bit per FER entry) */
#define SPI2_CLK_PORTA_FER   (1 << BITP_PORT_DATA_PX4)
#define SPI2_MISO_PORTA_FER  (1 << BITP_PORT_DATA_PX0)
#define SPI2_MOSI_PORTA_FER  (1 << BITP_PORT_DATA_PX1)
#define SPI2_D2_PORTA_FER    (1 << BITP_PORT_DATA_PX2)
#define SPI2_D3_PORTA_FER    (1 << BITP_PORT_DATA_PX3)
#define SPI2_SEL_PORTA_FER   (1 << BITP_PORT_DATA_PX5)

/* SPI2 GPIO MUX bit positions (two bits per MUX entry) */
#define SPI2_CLK_PORTA_MUX   (0 << (BITP_PORT_DATA_PX4 << 1))
#define SPI2_MISO_PORTA_MUX  (0 << (BITP_PORT_DATA_PX0 << 1))
#define SPI2_MOSI_PORTA_MUX  (0 << (BITP_PORT_DATA_PX1 << 1))
#define SPI2_D2_PORTA_MUX    (0 << (BITP_PORT_DATA_PX2 << 1))
#define SPI2_D3_PORTA_MUX    (0 << (BITP_PORT_DATA_PX3 << 1))
#define SPI2_SEL_PORTA_MUX   (0 << (BITP_PORT_DATA_PX5 << 1))

/* UART0 GPIO FER bit positions */
#define UART0_TX_PORTA_FER   (1 << BITP_PORT_DATA_PX6)
#define UART0_RX_PORTA_FER   (1 << BITP_PORT_DATA_PX7)
#define UART0_RTS_PORTA_FER  (1 << BITP_PORT_DATA_PX8)
#define UART0_CTS_PORTA_FER  (1 << BITP_PORT_DATA_PX9)

/* UART0 GPIO MUX bit positions (two bits per MUX entry */
#define UART0_TX_PORTA_MUX   (1 << (BITP_PORT_DATA_PX6 << 1))
#define UART0_RX_PORTA_MUX   (1 << (BITP_PORT_DATA_PX7 << 1))
#define UART0_RTS_PORTA_MUX  (1 << (BITP_PORT_DATA_PX8 << 1))
#define UART0_CTS_PORTA_MUX  (1 << (BITP_PORT_DATA_PX9  << 1))

/* DAI IE Bit definitions (not in any ADI header files) */
#define BITP_PADS0_DAI0_IE_PB03   (2)
#define BITP_PADS0_DAI0_IE_PB04   (3)
#define BITP_PADS0_DAI0_IE_PB05   (4)

/*
 * WARNING: Order must match the GPIO_PIN_ID enum in gpio_pins.h!
 */
GPIO_CONFIG gpioPins[GPIO_PIN_MAX] = {
    { ADI_GPIO_PORT_B, ADI_GPIO_PIN_3,  ADI_GPIO_DIRECTION_INPUT,  0 },  // GPIO_PIN_SOMCRR_PB1
    { ADI_GPIO_PORT_B, ADI_GPIO_PIN_5,  ADI_GPIO_DIRECTION_INPUT,  0 },  // GPIO_PIN_SOMCRR_PB2
    { ADI_GPIO_PORT_C, ADI_GPIO_PIN_1,  ADI_GPIO_DIRECTION_OUTPUT, 0 },  // GPIO_PIN_SOMCRR_LED7
    { ADI_GPIO_PORT_C, ADI_GPIO_PIN_2,  ADI_GPIO_DIRECTION_OUTPUT, 0 },  // GPIO_PIN_SOMCRR_LED10
    { ADI_GPIO_PORT_C, ADI_GPIO_PIN_3,  ADI_GPIO_DIRECTION_OUTPUT, 0 },  // GPIO_PIN_SOMCRR_LED9
    { ADI_GPIO_PORT_C, ADI_GPIO_PIN_5,  ADI_GPIO_DIRECTION_INPUT,  0 },  // GPIO_PIN_SOMCRR_A2B1_IRQ
};

bool gpio_get_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;
    uint32_t value;

    pin = &pinConfig[pinId];

    result = adi_gpio_GetData(pin->port, &value);

    pin->state = value & pin->pinNum;

    return(pin->state);
}

void gpio_set_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId, bool state)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;

    pin = &pinConfig[pinId];

    if (state) {
        result = adi_gpio_Set(pin->port, pin->pinNum);
    } else {
        result = adi_gpio_Clear(pin->port, pin->pinNum);
    }

    pin->state = state;
}

void gpio_toggle_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;

    pin = &pinConfig[pinId];

    result = adi_gpio_Toggle(pin->port, pin->pinNum);

    gpio_get_pin(pinConfig, pinId);
}

void gpio_init_pins(GPIO_CONFIG *pinConfig, int numPins)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;
    int i;

    for (i = 0; i < GPIO_PIN_MAX; i++) {

        /* Select the pin */
        pin = &pinConfig[i];

        /* Set the initial pin state */
        if (pin->state) {
            result = adi_gpio_Set(pin->port, pin->pinNum);
        } else {
            result = adi_gpio_Clear(pin->port, pin->pinNum);
        }

        /* Set the direction */
        result = adi_gpio_SetDirection(pin->port, pin->pinNum, pin->dir);

        /* Set the flags */
        if (pin->flags & GPIO_FLAGS_PUE) {
            adi_pads_PortPullup(pin->port, pin->pinNum, true);
        }
    }
}

void gpio_init(void)
{
    static uint8_t gpioMemory[ADI_GPIO_CALLBACK_MEM_SIZE * 16];
    uint32_t numCallbacks;
    ADI_GPIO_RESULT result;

    /* Init the GPIO system service */
    result = adi_gpio_Init(gpioMemory, sizeof(gpioMemory), &numCallbacks);

    /* Configure TWI2 Alternate Function GPIO */
    *pREG_PORTA_FER |= (
        TWI2_SCL_PORTA_FER |
        TWI2_SDA_PORTA_FER
    );
    *pREG_PORTA_MUX |= (
        TWI2_SCL_PORTA_MUX |
        TWI2_SDA_PORTA_MUX
    );

    /* Configure SPI2 Alternate Function GPIO */
    *pREG_PORTA_FER |= (
        SPI2_CLK_PORTA_FER |
        SPI2_MISO_PORTA_FER |
        SPI2_MOSI_PORTA_FER |
        SPI2_D2_PORTA_FER |
        SPI2_D3_PORTA_FER |
        SPI2_SEL_PORTA_FER
    );
    *pREG_PORTA_MUX |= (
        SPI2_CLK_PORTA_MUX |
        SPI2_MISO_PORTA_MUX |
        SPI2_MOSI_PORTA_MUX |
        SPI2_D2_PORTA_MUX |
        SPI2_D3_PORTA_MUX |
        SPI2_SEL_PORTA_MUX
    );


    /* Configure UART0 Alternate Function GPIO */
    *pREG_PORTA_FER |= (
        UART0_TX_PORTA_FER |
        UART0_RX_PORTA_FER |
        UART0_RTS_PORTA_FER |
        UART0_CTS_PORTA_FER
    );
    *pREG_PORTA_MUX |= (
        UART0_TX_PORTA_MUX |
        UART0_RX_PORTA_MUX |
        UART0_RTS_PORTA_MUX |
        UART0_CTS_PORTA_MUX
    );

    /* Configure straight GPIO */
    gpio_init_pins(gpioPins, GPIO_PIN_MAX);

    /* PADS0 DAI0/1 Port Input Enable Control Register */
    *pREG_PADS0_DAI0_IE = BITM_PADS_DAI0_IE_VALUE;
    *pREG_PADS0_DAI1_IE = BITM_PADS_DAI1_IE_VALUE;

}

/***********************************************************************
 * libc heap initialization
 **********************************************************************/
/* See 'build/SHARC0-EV-21569-EZKIT.ldf' to modify system heap size */

/***********************************************************************
 * UMM_MALLOC heap initialization
 **********************************************************************/
#pragma section("seg_l1_block0_noinit_data", NO_INIT)
    static uint8_t umm_block0_heap[UMM_L1_BLOCK0_HEAP_SIZE];
#pragma section("seg_l1_block1_noinit_data", NO_INIT)
    static uint8_t umm_block1_heap[UMM_L1_BLOCK1_HEAP_SIZE];
#pragma section("seg_l1_block2_noinit_data", NO_INIT)
    static uint8_t umm_block2_heap[UMM_L1_BLOCK2_HEAP_SIZE];
#pragma section("seg_l1_block3_noinit_data", NO_INIT)
    static uint8_t umm_block3_heap[UMM_L1_BLOCK3_HEAP_SIZE];
#pragma section("seg_l2_noinit_data_cached", NO_INIT)
    static uint8_t umm_l2_cached_heap[UMM_L2_CACHED_HEAP_SIZE];
#pragma section("seg_l2_noinit_data_uncached", NO_INIT)
    static uint8_t umm_l2_uncached_heap[UMM_L2_UNCACHED_HEAP_SIZE];
#pragma section("seg_sdram_noinit_data", NO_INIT)
    static uint8_t umm_sdram_cached_heap[UMM_SDRAM_CACHED_HEAP_SIZE];

void umm_heap_init(void)
{
    /* Initialize the L1 Block 0 heap. */
    umm_init(UMM_L1_BLOCK0_HEAP, umm_block0_heap, UMM_L1_BLOCK0_HEAP_SIZE);

    /* Initialize the L1 Block 1 heap. */
    umm_init(UMM_L1_BLOCK1_HEAP, umm_block1_heap, UMM_L1_BLOCK1_HEAP_SIZE);

    /* Initialize the L1 Block 2 heap. */
    umm_init(UMM_L1_BLOCK2_HEAP, umm_block2_heap, UMM_L1_BLOCK2_HEAP_SIZE);

    /* Initialize the L1 Block 3 heap. */
    umm_init(UMM_L1_BLOCK3_HEAP, umm_block3_heap, UMM_L1_BLOCK3_HEAP_SIZE);

    /* Initialize the L2 cached heap. */
    umm_init(UMM_L2_CACHED_HEAP, umm_l2_cached_heap, UMM_L2_CACHED_HEAP_SIZE);

    /* Initialize the L2 uncached heap. */
    umm_init(UMM_L2_UNCACHED_HEAP, umm_l2_uncached_heap,
        UMM_L2_UNCACHED_HEAP_SIZE);

    /* Initialize the cached L3 SDRAM heap (default heap). */
    umm_init(UMM_SDRAM_CACHED_HEAP, umm_sdram_cached_heap,
        UMM_SDRAM_CACHED_HEAP_SIZE);
}

/***********************************************************************
 * SPI Flash initialization
 **********************************************************************/
void flash_init(APP_CONTEXT *context)
{
    SPI_SIMPLE_RESULT spiResult;

    /* Open a SPI handle to SPI2 */
    spiResult = spi_open(SPI2, &context->spi2Handle);

    /* Open a SPI2 device handle for the flash */
    spiResult = spi_openDevice(context->spi2Handle, &context->spiFlashHandle);

    /* Configure the flash device handle */
    spiResult = spi_setClock(context->spiFlashHandle, 5);
    spiResult = spi_setMode(context->spiFlashHandle, SPI_MODE_3);
    spiResult = spi_setFastMode(context->spiFlashHandle, true);
    spiResult = spi_setLsbFirst(context->spiFlashHandle, false);
    spiResult = spi_setSlaveSelect(context->spiFlashHandle, SPI_SSEL_1);

    /* Open the flash driver with the configured SPI device handle */
    context->flashHandle = is25lp_open(context->spiFlashHandle);
}

/***********************************************************************
 * CGU Timestamp init
 **********************************************************************/
void cgu_ts_init(void)
{
    /* Configure the CGU timestamp counter.  See clocks.h for more detail. */
    *pREG_CGU0_TSCTL =
        ( 1 << BITP_CGU_TSCTL_EN ) |
        ( CGU_TS_DIV << BITP_CGU_TSCTL_TSDIV );
}

/***********************************************************************
 * This function allocates audio buffers in L2 cached memory and
 * initializes a single SPORT using the simple SPORT driver.
 *
 * NOTE: OUT buffers are generally marked as not cached since outbound
 *       buffers are flushed in process_audio().  This keeps the driver
 *       from prematurely (and redundantly) flushing the buffer when the
 *       SPORT callback returns.
 **********************************************************************/
static sSPORT *single_sport_init(SPORT_SIMPLE_PORT sport,
    SPORT_SIMPLE_CONFIG *cfg, SPORT_SIMPLE_AUDIO_CALLBACK cb,
    void **pingPongPtrs, unsigned *pingPongLen, void *usrPtr,
    bool cached, SPORT_SIMPLE_RESULT *result)
{
    sSPORT *sportHandle;
    SPORT_SIMPLE_RESULT sportResult;
    uint32_t dataBufferSize;
    int i;

    /* Open a handle to the SPORT */
    sportResult = sport_open(sport, &sportHandle);
    if (sportResult != SPORT_SIMPLE_SUCCESS) {
        if (result) {
            *result = sportResult;
        }
        return(NULL);
    }

    /* Copy application callback info */
    cfg->callBack = cb;
    cfg->usrPtr = usrPtr;

    /* Allocate audio buffers if not already allocated */
    dataBufferSize = sport_buffer_size(cfg);
    for (i = 0; i < 2; i++) {
        if (!cfg->dataBuffers[i]) {
            cfg->dataBuffers[i] = umm_malloc_heap_aligned(
                UMM_L2_CACHED_HEAP, dataBufferSize, ADI_CACHE_LINE_LENGTH);
            memset(cfg->dataBuffers[i], 0, dataBufferSize);
        }
    }
    cfg->dataBuffersCached = cached;
    cfg->syncDMA = true;

    /* Configure the SPORT */
    sportResult = sport_configure(sportHandle, cfg);

    /* Save ping pong data pointers */
    if (pingPongPtrs) {
        pingPongPtrs[0] = cfg->dataBuffers[0];
        pingPongPtrs[1] = cfg->dataBuffers[1];
    }
    if (pingPongLen) {
        *pingPongLen = dataBufferSize;
    }
    if (result) {
        *result = sportResult;
    }

    return(sportHandle);
}

static void single_sport_deinit(sSPORT **sportHandle,
    void **pingPongPtrs, unsigned *pingPongLen)
{
    if (sportHandle && *sportHandle) {
        sport_close(sportHandle);
    }
    if (pingPongPtrs) {
        if (pingPongPtrs[0]) {
            umm_free_heap_aligned(UMM_L2_CACHED_HEAP, pingPongPtrs[0]);
            pingPongPtrs[0] = NULL;
        }
        if (pingPongPtrs[1]) {
            umm_free_heap_aligned(UMM_L2_CACHED_HEAP, pingPongPtrs[1]);
            pingPongPtrs[1] = NULL;
        }
    }
    if (pingPongLen) {
        *pingPongLen = 0;
    }
}

/***********************************************************************
 * Simple SPORT driver 8/16-ch digital audio settings:
 *
 * FS = Frame Sync, BCLK = Bit Clock
 *
 * Rising edge FS (pulse high)
 * Early FS (data MSb delayed 1 BCLK)
 * Assert FS and data on BCLK rising edge
 * Sample FS and data on BCLK falling edge
 * 32-bit
 *
 * AD24xx compatible register settings:
 *
 * 8 ch
 *    I2SGCFG: 0x42
 *     I2SCFG: 0xF7
 *
 * 16 ch
 *    I2SGCFG: 0x44
 *     I2SCFG: 0xF7
 **********************************************************************/
SPORT_SIMPLE_CONFIG cfg8ch = {
    .clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE,
    .fsDir = SPORT_SIMPLE_FS_DIR_SLAVE,
    .dataDir = SPORT_SIMPLE_DATA_DIR_UNKNOWN,
    .bitClkOptions = SPORT_SIMPLE_CLK_DEFAULT,
    .fsOptions = SPORT_SIMPLE_FS_OPTION_EARLY,
    .tdmSlots = SPORT_SIMPLE_TDM_8,
    .wordSize = SPORT_SIMPLE_WORD_SIZE_32BIT,
    .dataEnable = SPORT_SIMPLE_ENABLE_BOTH,
    .frames = SYSTEM_BLOCK_SIZE,
    .syncDMA = true
};

SPORT_SIMPLE_CONFIG cfg16ch = {
    .clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE,
    .fsDir = SPORT_SIMPLE_FS_DIR_SLAVE,
    .dataDir = SPORT_SIMPLE_DATA_DIR_UNKNOWN,
    .bitClkOptions = SPORT_SIMPLE_CLK_DEFAULT,
    .fsOptions = SPORT_SIMPLE_FS_OPTION_EARLY,
    .tdmSlots = SPORT_SIMPLE_TDM_16,
    .wordSize = SPORT_SIMPLE_WORD_SIZE_32BIT,
    .dataEnable = SPORT_SIMPLE_ENABLE_BOTH,
    .frames = SYSTEM_BLOCK_SIZE,
    .syncDMA = true
};

void disable_mclk(APP_CONTEXT *context)
{
    *pREG_PADS0_DAI1_IE &= ~(
        (1 << BITP_PADS0_DAI0_IE_PB05)
    );
}

void enable_mclk(APP_CONTEXT *context)
{
    *pREG_PADS0_DAI1_IE |= (
        (1 << BITP_PADS0_DAI0_IE_PB05)
    );
}

static void sru_config_mclk(APP_CONTEXT *context)
{
    /* 24.576Mhz MCLK in from DAC 512Fs BCLK on DAI1.05 */
    SRU2(LOW, DAI1_PBEN05_I);

    /* Cross-route DAI1.05 MCLK to DAI0.11 as an output */
    SRU(HIGH, DAI0_PBEN11_I);
    SRU(DAI0_CRS_PB05_O, DAI0_PB11_I);
}

static void pcg_init_dai1_tdm8_bclk(void)
{
    /* Configure static PCG C parameters */
    PCG_SIMPLE_CONFIG pcg = {
        .pcg = PCG_C,                   // PCG C
        .clk_src = PCG_SRC_DAI_PIN,     // Sourced from DAI
        .clk_in_dai_pin = 5,            // Sourced from DAI pin 5
        .lrclk_clocks_per_frame = 256,  // Not used
        .sync_to_fs = false
    };

    /* Configure a 12.288 MHz BCLK from 24.576 BCLK */
    pcg.bitclk_div = 2;
    pcg_open(&pcg);
    pcg_enable(pcg.pcg, true);
}

void mclk_init(APP_CONTEXT *context)
{
    sru_config_mclk(context);
    pcg_init_dai1_tdm8_bclk();
}

/***********************************************************************
 * ADAU1962 DAC / SPORT4 / SRU initialization (TDM16 clock slave)
 **********************************************************************/
#define ADAU1962_I2C_ADDR  (0x04)

static void sru_config_adau1962_master(void)
{
    /* Setup DAI Pin I/O */
    SRU2(HIGH, DAI1_PBEN01_I);      // ADAU1962 DAC data1 is an output
    SRU2(HIGH, DAI1_PBEN02_I);      // ADAU1962 DAC data2 is an output
    SRU2(LOW, DAI1_PBEN04_I);       // ADAU1962 FS is an input
    //SRU2(LOW, DAI1_PBEN05_I);       // ADAU1962 CLK is an input

    /* Route clocks */
    SRU2(DAI1_PB04_O, SPT4_AFS_I);   // route FS
    SRU2(DAI1_PB05_O, SPT4_ACLK_I);  // route BCLK

    /* Route to SPORT4A */
    SRU2(SPT4_AD0_O, DAI1_PB01_I);    // SPORT4A-D0 output to ADAU1962 data1 pin
    SRU2(SPT4_AD1_O, DAI1_PB02_I);    // SPORT4A-D1 output to ADAU1962 data2 pin
}

static void adau1962_sport_init(APP_CONTEXT *context)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;

    /* SPORT4A: DAC 16-ch data out */
    sportCfg = cfg16ch;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_TX;
    sportCfg.dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY;
    memcpy(sportCfg.dataBuffers, context->codecAudioOut, sizeof(sportCfg.dataBuffers));
    context->dacSportOutHandle = single_sport_init(
        SPORT4A, &sportCfg, dacAudioOut,
        context->codecAudioOut, &context->codecAudioOutLen, context, false, NULL
    );

    if (context->dacSportOutHandle) {
        sportResult = sport_start(context->dacSportOutHandle, true);
    }
}

static void adau1962_sport_deinit(APP_CONTEXT *context)
{
    if (context->dacSportOutHandle) {
        single_sport_deinit(
            &context->dacSportOutHandle, context->codecAudioOut, &context->codecAudioOutLen
        );
        clock_domain_clr_active(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_CODEC_OUT);
    }
}

void adau1962_init(APP_CONTEXT *context)
{
    /* Configure the DAI routing */
    sru_config_adau1962_master();

    /* Configure the SPORT */
    adau1962_sport_init(context);

    /* Initialize the DAC */
    init_adau1962(context->adau1962TwiHandle, ADAU1962_I2C_ADDR);
}

/***********************************************************************
 * ADAU1979 ADC / SPORT6 / SRU initialization (TDM8 clock slave)
 *
 * WARNING: The ADAU1979 does not have the drive strength to reliably
 *          drive data out with a TDM16 bit clock.
 **********************************************************************/
#define ADAU1979_I2C_ADDR  (0x11)

static void sru_config_adau1979_slave(void)
{
    /* Setup DAI Pin I/O */
    SRU2(LOW, DAI1_PBEN06_I);      // ADAU1979 ADC data1 is an input
    SRU2(LOW, DAI1_PBEN07_I);      // ADAU1979 ADC data2 is an input
    SRU2(HIGH, DAI1_PBEN12_I);     // ADAU1979 CLK is an output
    SRU2(HIGH, DAI1_PBEN20_I);     // ADAU1979 FS is an output

    /* Route audio clocks to ADAU1979 */
    SRU2(SPT6_AFS_O, DAI1_PB20_I);   // route SPORT4A FS to ADAU1979 FS
    SRU2(PCG0_CLKC_O, DAI1_PB12_I);  // route TDM8 BCLK to ADAU1979 BCLK

    /* Route to SPORT6A */
    SRU2(DAI1_PB20_O, SPT6_AFS_I);   // route ADAU1979 FS to SPORT6A frame sync
    SRU2(DAI1_PB12_O, SPT6_ACLK_I);  // route ADAU1979 BCLK to SPORT6A clock input
    SRU2(DAI1_PB06_O, SPT6_AD0_I);   // ADAU1979 SDATAOUT1 pin to SPORT6A data0
    SRU2(DAI1_PB07_O, SPT6_AD1_I);   // ADAU1979 SDATAOUT2 pin to SPORT6A data1
}

static void adau1979_sport_init(APP_CONTEXT *context)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;

    /* SPORT6A: ADC 8-ch data in */
    sportCfg = cfg8ch;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_RX;
    sportCfg.dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY;
    sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_MASTER;
    memcpy(sportCfg.dataBuffers, context->codecAudioIn, sizeof(sportCfg.dataBuffers));
    context->adcSportInHandle = single_sport_init(
        SPORT6A, &sportCfg, adcAudioIn,
        context->codecAudioIn, &context->codecAudioInLen, context, true, NULL
    );

    if (context->adcSportInHandle) {
        sportResult = sport_start(context->adcSportInHandle, true);
    }
}

static void adau1979_sport_deinit(APP_CONTEXT *context)
{
    if (context->adcSportInHandle) {
        single_sport_deinit(
            &context->adcSportInHandle, context->codecAudioIn, &context->codecAudioInLen
        );
        clock_domain_clr_active(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_CODEC_IN);
    }
}

void adau1979_init(APP_CONTEXT *context)
{
    /* Configure the DAI routing */
    sru_config_adau1979_slave();

    /* Configure the SPORT */
    adau1979_sport_init(context);

    /* Initialize the ADC */
    init_adau1979(context->adau1962TwiHandle, ADAU1979_I2C_ADDR);
}

/**************************************************************************
 * SPDIF Init
 *************************************************************************/
SPORT_SIMPLE_CONFIG cfgI2Sx1 = {
    .clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE,
    .fsDir = SPORT_SIMPLE_FS_DIR_MASTER,
    .dataDir = SPORT_SIMPLE_DATA_DIR_UNKNOWN,
    .bitClkOptions = SPORT_SIMPLE_CLK_FALLING,
    .fsOptions = SPORT_SIMPLE_FS_OPTION_INV | SPORT_SIMPLE_FS_OPTION_EARLY |
                 SPORT_SIMPLE_FS_OPTION_50,
    .tdmSlots = SPORT_SIMPLE_TDM_2,
    .wordSize = SPORT_SIMPLE_WORD_SIZE_32BIT,
    .dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY,
    .frames = SYSTEM_BLOCK_SIZE,
};

/* PCGA generates 12.288 MHz CLK from 24.576 MCLK/BCLK */
static void spdif_cfg_hfclk(void)
{
    /* Configure static PCG A parameters */
    PCG_SIMPLE_CONFIG pcg = {
        .pcg = PCG_A,                    // PCG A
        .clk_src = PCG_SRC_DAI_PIN,      // Sourced from DAI
        .clk_in_dai_pin = 11,            // Sourced from DAI pin 11
        .lrclk_clocks_per_frame = 256,   // Not used
        .sync_to_fs = false
    };

    /* Configure the PCG BCLK depending on the cfgI2Sx1 SPORT config */
    pcg.bitclk_div = 2;

    /* This sets everything up */
    pcg_open(&pcg);
    pcg_enable(pcg.pcg, true);
}

/* PCGB generates 3.072 MHz I2S BCLK from 24.576 MCLK/BCLK */
static void spdif_cfg_bclk(void)
{
    /* Configure static PCG B parameters */
    PCG_SIMPLE_CONFIG pcg = {
        .pcg = PCG_B,                    // PCG B
        .clk_src = PCG_SRC_DAI_PIN,      // Sourced from DAI
        .clk_in_dai_pin = 11,            // Sourced from DAI pin 11
        .lrclk_clocks_per_frame = 256,   // Not used
        .sync_to_fs = false
    };

    /* Configure the PCG BCLK depending on the cfgI2Sx1 SPORT config */
    pcg.bitclk_div =
        SYSTEM_MCLK_RATE / (cfgI2Sx1.wordSize * cfgI2Sx1.tdmSlots * SYSTEM_SAMPLE_RATE);
    assert(pcg.bitclk_div > 0);

    /* This sets everything up */
    pcg_open(&pcg);
    pcg_enable(pcg.pcg, true);
}

/*
 * The SPDIF HFCLK is derived from PCG0_CLKA_O
 * The SPDIF BCLK is derived from PCG0_CLKB_O
 *
 */
static void spdif_sru_config(void)
{
    // Assign SPDIF I/O pins
    SRU(HIGH, DAI0_PBEN10_I);       // SPDIF TX is an output
    SRU(LOW,  DAI0_PBEN09_I);       // SPDIF RX is an input

    // Connect I/O pins to SPDIF module
    SRU(DAI0_PB09_O, SPDIF0_RX_I);  // route SPDIF RX
    SRU(SPDIF0_TX_O, DAI0_PB10_I);  // route SPDIF TX

    // Connect 64Fs BCLK to SPORT2A/B
    SRU(PCG0_CLKB_O, SPT2_ACLK_I);     // route PCG 64fs BCLK signal to SPORT2A BCLK
    SRU(PCG0_CLKB_O, SPT2_BCLK_I);     // route PCG 64fs BCLK signal to SPORT2B BCLK

    // Connect SPDIF RX to SRC 0 "IP" side
    SRU(SPDIF0_RX_CLK_O, SRC0_CLK_IP_I);     // route SPDIF RX BCLK to SRC IP BCLK
    SRU(SPDIF0_RX_FS_O,  SRC0_FS_IP_I);      // route SPDIF RX FS to SRC IP FS
    SRU(SPDIF0_RX_DAT_O, SRC0_DAT_IP_I);     // route SPDIF RX Data to SRC IP Data

    // Connect SPORT2B to SRC 0 "OP" side
    SRU(PCG0_CLKB_O,    SRC0_CLK_OP_I);     // route PCG 64fs BCLK signal to SRC OP BCLK
    SRU(SPT2_BFS_O,     SRC0_FS_OP_I);      // route SPORT FS signal to SRC OP FS
    SRU(SRC0_DAT_OP_O,  SPT2_BD0_I);        // route SRC0 OP Data output to SPORT 2B data

    // Connect 256Fs HFCLK to SPDIF TX
    SRU(PCG0_CLKA_O, SPDIF0_TX_HFCLK_I); // route PCGA_CLK to SPDIF TX HFCLK

    // Connect SPORT2A to SPDIF TX
    SRU(PCG0_CLKB_O, SPDIF0_TX_CLK_I);    // route 64fs BCLK signal to SPDIF TX BCLK
    SRU(SPT2_AFS_O,  SPDIF0_TX_FS_I);     // route SPORT2A FS signal to SPDIF TX FS
    SRU(SPT2_AD0_O,  SPDIF0_TX_DAT_I);    // SPT2A AD0 output to SPDIF TX data pin
}

static void spdif_sport_init(APP_CONTEXT *context)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;

    /* SPORT2A: SPDIF data out */
    sportCfg = cfgI2Sx1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_TX;
    memcpy(sportCfg.dataBuffers, context->spdifAudioOut, sizeof(sportCfg.dataBuffers));
    context->spdifSportOutHandle = single_sport_init(
        SPORT2A, &sportCfg, spdifAudioOut,
        context->spdifAudioOut, &context->spdifAudioOutLen, context, false, NULL
    );
    if (context->spdifSportOutHandle) {
        sportResult = sport_start(context->spdifSportOutHandle, true);
    }

    /* SPORT2B: SPDIF data in */
    sportCfg = cfgI2Sx1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_RX;
    memcpy(sportCfg.dataBuffers, context->spdifAudioIn, sizeof(sportCfg.dataBuffers));
    context->spdifSportInHandle = single_sport_init(
        SPORT2B, &sportCfg, spdifAudioIn,
        context->spdifAudioIn, &context->spdifAudioInLen, context, true, NULL
    );
    if (context->spdifSportInHandle) {
        sportResult = sport_start(context->spdifSportInHandle, true);
    }
}

static void spdif_sport_deinit(APP_CONTEXT *context)
{
    if (context->spdifSportOutHandle) {
        single_sport_deinit(
            &context->spdifSportOutHandle, context->spdifAudioOut, &context->spdifAudioOutLen
        );
        clock_domain_clr_active(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_SPDIF_OUT);
    }
    if (context->spdifSportInHandle) {
        single_sport_deinit(
            &context->spdifSportInHandle, context->spdifAudioIn, &context->spdifAudioInLen
        );
        clock_domain_clr_active(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_SPDIF_IN);
    }
}

static void spdif_asrc_init(void)
{
    // Configure and enable SRC 0/1
    *pREG_ASRC0_CTL01 =
        BITM_ASRC_CTL01_EN0 |                // Enable SRC0
        (0x1 << BITP_ASRC_CTL01_SMODEIN0) |  // Input mode = I2S
        (0x1 << BITP_ASRC_CTL01_SMODEOUT0) | // Output mode = I2S
        0;

    // Configure and enable SPDIF RX
    *pREG_SPDIF0_RX_CTL =
        BITM_SPDIF_RX_CTL_EN |          // Enable the SPDIF RX
        BITM_SPDIF_RX_CTL_FASTLOCK |    // Enable SPDIF Fastlock (see HRM 32-15)
        BITM_SPDIF_RX_CTL_RSTRTAUDIO |
        0;

    // Configure SPDIF Transmitter in auto mode
    *pREG_SPDIF0_TX_CTL =
        (0x1 << BITP_SPDIF_TX_CTL_SMODEIN) |  // I2S Mode
        BITM_SPDIF_TX_CTL_AUTO |             // Standalone mode
        0;

    // Enable SPDIF transmitter
    *pREG_SPDIF0_TX_CTL |=
        BITM_SPDIF_TX_CTL_EN |         // Enable SPDIF TX
        0;
}

void spdif_init(APP_CONTEXT *context)
{
    /* Configure the DAI routing */
    spdif_sru_config();

    /* Initialize the SPDIF BCLK and HFCLK PCGs */
    spdif_cfg_bclk();
    spdif_cfg_hfclk();

    /* Initialize the SPDIF and ASRC modules */
    spdif_asrc_init();

    /* Initialize the SPORTs */
    spdif_sport_init(context);
}

/***********************************************************************
 * A2B Generic functions
 **********************************************************************/
#include "a2b_irq.h"
#include "a2b_to_sport_cfg.h"

/***********************************************************************
 * J10 / A2B (Main/Sub) / SPORT0-1 / SRU / IRQ initialization
 **********************************************************************/
void a2b_pint_init(APP_CONTEXT *context)
{
    ADI_GPIO_RESULT result;
    uint32_t pint2pins = 0x00000000;

    /* Don't wire in interrupts if no A2B */
    if (!context->a2bPresent) {
        return;
    }

    /* This code must match the values in a2b_irq.h */
    assert(A2B1_PINT_IRQ == ADI_GPIO_PIN_INTERRUPT_2);
    assert(A2B1_PINT_PIN == ADI_GPIO_INT_PIN_5);

    /*
     * A2B1 (J10): Port C.5 -> PINT2 Byte 0.5)
     */
    pint2pins |= ADI_GPIO_INT_PIN_5;

    /* Map Pins PC0-PC7 to PINT2 Byte 0 */
    result = adi_gpio_PinInterruptAssignment(
        ADI_GPIO_PIN_INTERRUPT_2,
        ADI_GPIO_PIN_ASSIGN_BYTE_0,
        ADI_GPIO_PIN_ASSIGN_PCL_PINT2
    );
    result = adi_gpio_SetPinIntEdgeSense(
        ADI_GPIO_PIN_INTERRUPT_2,
        pint2pins,
        ADI_GPIO_SENSE_RISING_EDGE
    );
    result = adi_gpio_RegisterCallback(
        ADI_GPIO_PIN_INTERRUPT_2,
        pint2pins,
        a2b_irq,
        context
    );
    result = adi_gpio_EnablePinInterruptMask(
        ADI_GPIO_PIN_INTERRUPT_2,
        pint2pins,
        true
    );
}

/**
 *
 * A2B Master Mode Configuration:
 *    - MCLK/BCLK to SPORT1B/A2B Transceiver
 *    - SPORT1A FS to SPORT1B/A2B Transceiver
 */
void sru_config_a2b_master(APP_CONTEXT *context)
{
    // Set up pins for J10 A2B
    SRU(HIGH,  DAI0_PBEN03_I);        // pin for A2B BCLK is an output (to A2B bus)
    SRU(HIGH,  DAI0_PBEN04_I);        // pin for A2B FS is an output (to A2B bus)
    if (context->a2bxcvr == A2B_TO_SPORT_CFG_XCVR_AD243x) {
        SRU(LOW,   DAI0_PBEN07_I);    // DTX0/SIO4 is always an input (from A2B bus)
        SRU(LOW,   DAI0_PBEN06_I);    // DTX1/SIO3 is always an input (from A2B bus)
        SRU(HIGH,  DAI0_PBEN01_I);    // DRX0/SIO0 is always an output (to A2B bus)
        SRU(HIGH,  DAI0_PBEN02_I);    // DRX1/SIO1 is always an output (to A2B bus)
        SRU(LOW,   DAI0_PBEN20_I);    // ADD_SIO7/DAI0_P20 is connected to SIO4 on the MINI
    } else {
        SRU(LOW,   DAI0_PBEN01_I);    // DTX0/SIO0 is always an input (from A2B bus)
        SRU(LOW,   DAI0_PBEN02_I);    // DTX1/SIO1 is always an input (from A2B bus)
        SRU(HIGH,  DAI0_PBEN05_I);    // DRX0/SIO2 is always an output (to A2B bus)
        SRU(HIGH,  DAI0_PBEN06_I);    // DRX1/SIO3 is always an output (to A2B bus)
    }

    // BCLK/MCLK to A2B and SPORTA/B CLK */
    SRU(DAI0_PB11_O, DAI0_PB03_I);     // route MCLK/BCLK to A2B
    SRU(DAI0_PB11_O, SPT1_ACLK_I);     // route MCLK/BCLK to SPORT1A
    SRU(DAI0_PB11_O, SPT1_BCLK_I);     // route MCLK/BCLK to SPORT1B

    // SPORT1A FS to A2B and SPORT1B FS */
    SRU(SPT1_AFS_O, DAI0_PB04_I);     // route SPORT1A FS to A2B
    SRU(SPT1_AFS_O, SPT1_BFS_I);      // route SPORT1A FS to SPORT1B

    // Connect A2B data signals to SPORT1
    if (context->a2bxcvr == A2B_TO_SPORT_CFG_XCVR_AD243x) {
        SRU(SPT1_AD0_O, DAI0_PB01_I); // route SPORT1A data TX primary to A2B DRX0
        SRU(SPT1_AD1_O, DAI0_PB02_I); // route SPORT1A data TX secondary to A2B DRX1
        SRU(DAI0_PB07_O, SPT1_BD0_I); // route A2B DTX0 to SPORT1B data RX primary
        SRU(DAI0_PB06_O, SPT1_BD1_I); // route A2B DTX1 to SPORT1B data RX secondary
    } else {
        SRU(SPT1_AD0_O, DAI0_PB05_I); // route SPORT1A data TX primary to A2B DRX0
        SRU(SPT1_AD1_O, DAI0_PB06_I); // route SPORT1A data TX secondary to A2B DRX1
        SRU(DAI0_PB01_O, SPT1_BD0_I); // route A2B DTX0 to SPORT1B data RX primary
        SRU(DAI0_PB02_O, SPT1_BD1_I); // route A2B DTX1 to SPORT1B data RX secondary
    }

    // Increase MCLK/BCLK and FS drive strength
    *pREG_PADS0_DAI0_0_DS &= ~(BITM_PADS_DAI0_0_DS_DAI3 | BITM_PADS_DAI0_0_DS_DAI4);
    *pREG_PADS0_DAI0_0_DS |= (2 << BITP_PADS_DAI0_0_DS_DAI3) | (2 << BITP_PADS_DAI0_0_DS_DAI4);
}

/**
 *
 * A2B Sub Node Configuration:
 *    - A2B BCLK to SPORT1B
 *    - A2B FS to SPORT1B
 *
 * WARNING: A2B Sub Node DAI routing is only generic for AD242x modules
 *          (with fixed TX/RX pins).  AD243x and AD244x modules require a
 *          discovery using a compatible configuration.
 *
 */
void sru_config_a2b_slave(APP_CONTEXT *context)
{
    // Set up pins for J10 A2B
    SRU(LOW,   DAI0_PBEN03_I);        // pin for A2B BCLK is an input (from A2B bus)
    SRU(LOW,   DAI0_PBEN04_I);        // pin for A2B FS is an input (from A2B bus)
    if (context->a2bxcvr == A2B_TO_SPORT_CFG_XCVR_AD243x) {
        SRU(LOW,   DAI0_PBEN07_I);        // DTX0/SIO4 is always an input (from A2B bus)
        SRU(LOW,   DAI0_PBEN06_I);        // DTX1/SIO3 is always an input (from A2B bus)
        SRU(HIGH,  DAI0_PBEN01_I);        // DRX0/SIO0 is always an output (to A2B bus)
        SRU(HIGH,  DAI0_PBEN02_I);        // DRX1/SIO1 is always an output (to A2B bus)
        SRU(LOW,   DAI0_PBEN20_I);        // ADD_SIO7/DAI0_P20 is connected to SIO4 on the MINI
    } else {
        SRU(LOW,   DAI0_PBEN01_I);        // DTX0/SIO0 is always an input (from A2B bus)
        SRU(LOW,   DAI0_PBEN02_I);        // DTX1/SIO1 is always an input (from A2B bus)
        SRU(HIGH,  DAI0_PBEN05_I);        // DRX0/SIO2 is always an output (to A2B bus)
        SRU(HIGH,  DAI0_PBEN06_I);        // DRX1/SIO3 is always an output (to A2B bus)
    }

    // BCLK/MCLK to SPORT1A/B, SPORT0A/B CLK */
    SRU(DAI0_PB03_O, SPT1_ACLK_I);     // route MCLK/BCLK to SPORT1A
    SRU(DAI0_PB03_O, SPT1_BCLK_I);     // route MCLK/BCLK to SPORT1B

    // SPORT1A FS to SPORT1B FS */
    SRU(DAI0_PB04_O, SPT1_AFS_I);      // route FS to SPORT1A
    SRU(DAI0_PB04_O, SPT1_BFS_I);      // route FS to SPORT1B

    // Connect A2B data signals to SPORT1
    if (context->a2bxcvr == A2B_TO_SPORT_CFG_XCVR_AD243x) {
        SRU(SPT1_AD0_O, DAI0_PB01_I);     // route SPORT1A data TX primary to A2B DRX0
        SRU(SPT1_AD1_O, DAI0_PB02_I);     // route SPORT1A data TX secondary to A2B DRX1
        SRU(DAI0_PB07_O, SPT1_BD0_I);     // route A2B DTX0 to SPORT1B data RX primary
        SRU(DAI0_PB06_O, SPT1_BD1_I);     // route A2B DTX1 to SPORT1B data RX secondary
    } else {
        SRU(SPT1_AD0_O, DAI0_PB05_I);     // route SPORT1A data TX primary to A2B DRX0
        SRU(SPT1_AD1_O, DAI0_PB06_I);     // route SPORT1A data TX secondary to A2B DRX0
        SRU(DAI0_PB01_O, SPT1_BD0_I);     // route A2B DTX0 to SPORT1B data RX primary
        SRU(DAI0_PB02_O, SPT1_BD1_I);     // route A2B DTX1 to SPORT1B data RX secondary
    }
}

#define AD242X_PRODUCT             0x03u

A2B_TO_SPORT_CFG_XCVR a2b_product(sTWI *twi, uint8_t twiAddr)
{
    TWI_SIMPLE_RESULT result;
    A2B_TO_SPORT_CFG_XCVR xcvr;
    uint8_t wBuf[1];
    uint8_t rBuf[1];

    wBuf[0] = AD242X_PRODUCT;

    result = twi_writeRead(twi, twiAddr, wBuf, sizeof(wBuf),
        rBuf, sizeof(rBuf));

    if (result != TWI_SIMPLE_SUCCESS) {
        return(A2B_TO_SPORT_CFG_XCVR_UNKNOWN);
    }

    switch (rBuf[0] & 0xF0) {
        case 0x10:
            xcvr = A2B_TO_SPORT_CFG_XCVR_AD241x;
            break;
        case 0x20:
            xcvr = A2B_TO_SPORT_CFG_XCVR_AD242x;
            break;
        case 0x30:
            xcvr = A2B_TO_SPORT_CFG_XCVR_AD243x;
            break;
        default:
            xcvr = A2B_TO_SPORT_CFG_XCVR_UNKNOWN;
            break;
    }

    return(xcvr);
}

#define AD242X_CONTROL             0x12u
#define AD242X_CONTROL_SOFTRST     0x04u
#define AD242X_CONTROL_MSTR        0x80u

#define A2B_I2C_ADDR_0x68          0x68
#define A2B_I2C_ADDR_0x6A          0x6A

/* Probe for A2B transceiver on J10 */
bool a2b_probe(APP_CONTEXT *context)
{
    TWI_SIMPLE_RESULT result;

    /* Probe for module at I2C addr 0x68 or 0x6A */
    result = twi_write(context->a2bTwiHandle, A2B_I2C_ADDR_0x68, NULL, 0);
    if (result == TWI_SIMPLE_SUCCESS) {
        context->cfg.a2bI2CAddr = A2B_I2C_ADDR_0x68;
    } else {
        result = twi_write(context->a2bTwiHandle, A2B_I2C_ADDR_0x6A, NULL, 0);
        if (result == TWI_SIMPLE_SUCCESS) {
            context->cfg.a2bI2CAddr = A2B_I2C_ADDR_0x6A;
        }
    }

    /* If not found, reset to defaults */
    if (result != TWI_SIMPLE_SUCCESS) {
        context->a2bxcvr == A2B_TO_SPORT_CFG_XCVR_UNKNOWN;
        context->cfg.a2bI2CAddr = DEFAULT_A2B_I2C_ADDR;
        syslog_print("A2B (J10): Not detected");
        return(false);
    }

    /* Probe transceiver type */
    context->a2bxcvr = a2b_product(context->a2bTwiHandle, context->cfg.a2bI2CAddr);

    if (context->a2bxcvr == A2B_TO_SPORT_CFG_XCVR_UNKNOWN) {
        syslog_print("A2B (J10): Unknown");
    } else {
        syslog_printf("A2B (J10): I2C Addr: 0x%02X, Type: AD24%ux\n",
            context->cfg.a2bI2CAddr,
            (unsigned)context->a2bxcvr);
    }

    return(context->a2bxcvr != A2B_TO_SPORT_CFG_XCVR_UNKNOWN);
}

/* Soft reset a single transceiver */
bool a2b_restart(APP_CONTEXT *context)
{
    TWI_SIMPLE_RESULT result;
    uint8_t wBuf[2];

    wBuf[0] = AD242X_CONTROL;
    wBuf[1] = AD242X_CONTROL_SOFTRST;
    if (context->a2bmode != A2B_BUS_MODE_SUB) {
        wBuf[1] |= AD242X_CONTROL_MSTR;
    }

    result = twi_write(context->a2bTwiHandle, context->cfg.a2bI2CAddr,
        wBuf, sizeof(wBuf));

    delay(10);

    return(result == TWI_SIMPLE_SUCCESS);
}

void a2b_reset(APP_CONTEXT *context)
{
    a2b_restart(context);
}

bool a2b_sport_init(APP_CONTEXT *context,
    bool master, CLOCK_DOMAIN clockDomain, uint8_t I2SGCFG, uint8_t I2SCFG,
    bool verbose)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;
    bool sportCfgOk;
    bool rxtx;

    /* Calculate the SPORT1/0 TX configuration */
    memset(&sportCfg, 0, sizeof(sportCfg));
    rxtx = false;
    sportCfgOk = a2b_to_sport_cfg(master, rxtx, I2SGCFG, I2SCFG, &sportCfg, verbose, context->a2bxcvr);
    if (!sportCfgOk) {
        goto abort;
    }
    sportCfg.frames = SYSTEM_BLOCK_SIZE;
    sportCfg.fs = SYSTEM_SAMPLE_RATE;
    sportCfg.clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE;

    /* Configure SPORT1 Tx */
    if (master) {
        sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_MASTER;
    } else {
        sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    }
    memcpy(sportCfg.dataBuffers, context->a2bAudioOut, sizeof(sportCfg.dataBuffers));
    context->a2bSportOutHandle = single_sport_init(
        SPORT1A, &sportCfg, a2bAudioOut,
        context->a2bAudioOut, &context->a2bAudioOutLen, context, false, &sportResult
    );
    if (sportResult == SPORT_SIMPLE_SUCCESS) {
        context->a2bOutChannels = context->a2bAudioOutLen / (sportCfg.frames * (sportCfg.wordSize/8));
        clock_domain_set(context, clockDomain, CLOCK_DOMAIN_BITM_A2B_OUT);
        sportResult = sport_start(context->a2bSportOutHandle, true);
    } else {
        if (context->a2bSportOutHandle) {
            sport_close(&context->a2bSportOutHandle);
        }
        context->a2bOutChannels = 0;
        clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_OUT);
    }

    /* Calculate the SPORT1/0 RX configuration */
    memset(&sportCfg, 0, sizeof(sportCfg));
    rxtx = true;
    sportCfgOk = a2b_to_sport_cfg(master, rxtx, I2SGCFG, I2SCFG, &sportCfg, verbose, context->a2bxcvr);
    if (!sportCfgOk) {
        goto abort;
    }
    sportCfg.clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE;
    sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    sportCfg.frames = SYSTEM_BLOCK_SIZE;
    sportCfg.fs = SYSTEM_SAMPLE_RATE;

    /* Configure SPORT1 Rx */
    memcpy(sportCfg.dataBuffers, context->a2bAudioIn, sizeof(sportCfg.dataBuffers));
    context->a2bSportInHandle = single_sport_init(
        SPORT1B, &sportCfg, a2bAudioIn,
        context->a2bAudioIn, &context->a2bAudioInLen, context, true, &sportResult
    );
    if (sportResult == SPORT_SIMPLE_SUCCESS) {
        context->a2bInChannels = context->a2bAudioInLen / (sportCfg.frames * (sportCfg.wordSize/8));
        clock_domain_set(context, clockDomain, CLOCK_DOMAIN_BITM_A2B_IN);
        sportResult = sport_start(context->a2bSportInHandle, true);
    } else {
        if (context->a2bSportInHandle) {
            sport_close(&context->a2bSportInHandle);
        }
        context->a2bInChannels = 0;
        clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_IN);
    }

abort:
    return(sportCfgOk);
}

bool a2b_sport_deinit(APP_CONTEXT *context, CLOCK_DOMAIN cd)
{
    single_sport_deinit(
        &context->a2bSportOutHandle, context->a2bAudioOut, &context->a2bAudioOutLen
    );
    clock_domain_clr_active(context, cd, CLOCK_DOMAIN_BITM_A2B_OUT);
    single_sport_deinit(
        &context->a2bSportInHandle, context->a2bAudioIn, &context->a2bAudioInLen
    );
    clock_domain_clr_active(context, cd, CLOCK_DOMAIN_BITM_A2B_IN);
    return(true);
}

bool a2b_master_init(APP_CONTEXT *context)
{
    bool ok;

    /* Don't enable if no A2B */
    if (!context->a2bPresent) {
        return(false);
    }

    sru_config_a2b_master(context);

    ok = a2b_sport_init(context, true, CLOCK_DOMAIN_SYSTEM,
        SYSTEM_I2SGCFG, SYSTEM_I2SCFG, false);
    if (ok) {
        context->a2bmode = A2B_BUS_MODE_MAIN;
        context->a2bSlaveActive = false;
    }

    return(ok);
}

static void a2b_disconnect_slave_clocks(void)
{
    *pREG_PADS0_DAI0_IE &= ~(
        BITP_PADS0_DAI0_IE_PB03 | BITP_PADS0_DAI0_IE_PB04
    );
}

static void a2b_connect_slave_clocks(void)
{
    *pREG_PADS0_DAI0_IE |= (
        BITP_PADS0_DAI0_IE_PB03 | BITP_PADS0_DAI0_IE_PB04
    );
}

bool a2b_init_slave(APP_CONTEXT *context)
{
    sru_config_a2b_slave(context);

    context->a2bmode = A2B_BUS_MODE_SUB;

    /*
     * Disconnect A2B from all clock domains.  IN and OUT will be re-attached
     * to the A2B domain during discovery when/if the TX and RX serializers
     * are enabled.
     */
    clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_IN);
    clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_OUT);

    return(true);
}

bool a2b_set_mode(APP_CONTEXT *context, A2B_BUS_MODE mode)
{
    CLOCK_DOMAIN cd;

    if (mode == context->a2bmode) {
        return(true);
    }

    if (context->a2bmode == A2B_BUS_MODE_SUB) {
        cd = CLOCK_DOMAIN_A2B;
    } else {
        cd = CLOCK_DOMAIN_SYSTEM;
    }

    a2b_sport_deinit(context, cd);

    if (mode == A2B_BUS_MODE_SUB) {
        a2b_init_slave(context);
    } else {
        adau1962_sport_deinit(context);
        adau1979_sport_deinit(context);
        spdif_sport_deinit(context);
        disable_mclk(context);
        adau1962_sport_init(context);
        adau1979_sport_init(context);
        spdif_sport_init(context);
        a2b_master_init(context);
        enable_mclk(context);
    }

    a2b_restart(context);

    return(true);
}

bool a2b_sport_start(APP_CONTEXT *context, uint8_t I2SGCFG, uint8_t I2SCFG)
{
    bool ok;
    ok = a2b_sport_init(context, false, CLOCK_DOMAIN_A2B,
        I2SGCFG, I2SCFG, true);
    a2b_connect_slave_clocks();
    return(ok);
}

bool a2b_sport_stop(APP_CONTEXT *context)
{
    bool ok;
    ok = a2b_sport_deinit(context, CLOCK_DOMAIN_A2B);
    a2b_disconnect_slave_clocks();
    return(ok);
}

/***********************************************************************
 * Audio routing
 **********************************************************************/
void audio_routing_init(APP_CONTEXT *context)
{
    context->routingTable = calloc(MAX_AUDIO_ROUTES, sizeof(ROUTE_INFO));
}

/**********************************************************************
 * SoM Carrier HW probe
 **********************************************************************/
int somcrr_hw_version(APP_CONTEXT *context)
{
    SPI_SIMPLE_RESULT spiResult;
    sSPIPeriph *spiFlash;
    sSPI *spi;
    int version = SOMCRR_REV_A;
    uint8_t rx[4];
    uint8_t tx[4] = { 0x9F, 0x00, 0x00, 0x00};
    bool value;

    /*
     * The carrier board rev can be detected by the polarity of the
     * OCTAL_SPI_CS_EN signal.  This signal is active HIGH on Rev A
     * boards and active LOW on Rev D boards.
     *
     * Temporarily disable the SoM QSPI (U2) by deasserting SPI2FLASH_CS_EN on
     * the SoM.  Then set OCTAL_SPI_CS_EN LOW and probe the OSPI (U40)
     * via SPI2 to detect Rev D boards.
     *
     * SPI CLK is set to 10MHz
     *
     * Default to Rev A
     */

    /* Configure SPI2 */
    spiResult = spi_open(SPI2, &spi);
    spiResult = spi_openDevice(spi, &spiFlash);
    spiResult = spi_setClock(spiFlash, 10);
    spiResult = spi_setMode(spiFlash, SPI_MODE_3);
    spiResult = spi_setFastMode(spiFlash, true);
    spiResult = spi_setLsbFirst(spiFlash, false);
    spiResult = spi_setSlaveSelect(spiFlash, SPI_SSEL_1);


    /* Disable SoM QSPI Flash */
    ss_set(context, SS_PIN_ID_nSPI2FLASH_CS_EN, 1);

    /* Save SOMCRR OSPI enable value */
    ss_get(context, SS_PIN_ID_OCTAL_SPI_CS_EN, &value);

    /* Set carrier board OCTAL_SPI_CS_EN low and probe (Rev D) */
    ss_set(context, SS_PIN_ID_OCTAL_SPI_CS_EN, 0);
    spiResult = spi_xfer(spiFlash, 4, rx, tx);
    if ((rx[1] == 0xC2) && (rx[2] == 0x85)) {
        version = SOMCRR_REV_D;
    }

    /* Restore SOMCRR OSPI enable value */
    ss_set(context, SS_PIN_ID_OCTAL_SPI_CS_EN, value);

    /* Close SPI2 */
    spiResult = spi_closeDevice(&spiFlash);
    spiResult = spi_close(&spi);

    /* Enable SoM QSPI Flash */
    ss_set(context, SS_PIN_ID_nSPI2FLASH_CS_EN, 0);

    /* Set new SS default values for Rev D where polarity changed */
    if (version == SOMCRR_REV_D) {
        ss_set(context, SS_PIN_ID_EEPROM_EN, 0);
        ss_set(context, SS_PIN_ID_PUSHBUTTON_EN, 0);
        ss_set(context, SS_PIN_ID_OCTAL_SPI_CS_EN, 1);
    }

    return(version);
}
