/**
 * Copyright (c) 2022 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */
#ifndef _sharc0_context_h
#define _sharc0_context_h

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "uart_simple.h"
#include "twi_simple.h"
#include "spi_simple.h"
#include "sport_simple.h"
#include "flash.h"
#include "shell.h"
#include "a2b_to_sport_cfg.h"
#include "clock_domain_defs.h"
#include "spiffs.h"
#include "route.h"

/* Misc defines */
#define UNUSED(expr) do { (void)(expr); } while (0)

/* SoM Carrier Versions */
#define SOMCRR_REV_A   100
#define SOMCRR_REV_D   400

/*
 * WARNING: Do not change SYSTEM_AUDIO_TYPE from int32_t
 *
 */
#define SYSTEM_MCLK_RATE               (24576000)
#define SYSTEM_SAMPLE_RATE             (48000)
#define SYSTEM_BLOCK_SIZE              (64)
#define SYSTEM_AUDIO_TYPE              int32_t
#define SYSTEM_MAX_CHANNELS            (32)

#define VU_MAX_CHANNELS                (64)

#define ADC_AUDIO_CHANNELS             (4)
#define ADC_DMA_CHANNELS               (8)
#define DAC_AUDIO_CHANNELS             (12)
#define DAC_DMA_CHANNELS               (16)

#define SPDIF_AUDIO_CHANNELS           (2)
#define SPDIF_DMA_CHANNELS             (2)

#define A2B_AUDIO_CHANNELS             (32)
#define A2B_DMA_CHANNELS               (32)

/*
 * The A2B I2C addresses are latched at power up and cannot be changed
 * at runtime. Use the 'a2b' command to set the I2C address at runtime to
 * match the HW.
 *
 * By default AD242x boards have address 0x68 and AD243x boards have
 * address 0x6A
 */
#define DEFAULT_A2B_I2C_ADDR        (0x68)

#define SPIFFS_VOL_NAME             "sf:"

typedef enum A2B_BUS_MODE {
    A2B_BUS_MODE_UNKNOWN = 0,
    A2B_BUS_MODE_MAIN,
    A2B_BUS_MODE_SUB
} A2B_BUS_MODE;

/*
 * FS = Frame Sync, BCLK = Bit Clock
 *
 * TDM 16 x 32-bit
 * Rising edge FS (pulse high)
 * Early FS (data MSb delayed 1 BCLK)
 * Assert FS and data on BCLK rising edge
 * Sample FS and data on BCLK falling edge
 *
 */
#define SYSTEM_I2SGCFG                 (0x44)
#define SYSTEM_I2SCFG                  (0xF7)

/* Audio routing */
#define MAX_AUDIO_ROUTES               (16)

typedef struct APP_CFG {
    unsigned a2bI2CAddr;
} APP_CFG;

/*
 * The main application context.  Used as a container to carry a
 * variety of useful pointers, handles, etc., between various
 * modules and subsystems.
 */
typedef struct _APP_CONTEXT {

    /* SoM Carrier Version */
    int SoMCRRVersion;

    /* Core clock frequency */
    uint32_t cclk;

    /* Device handles */
    sUART *stdioHandle;
    sSPI *spi2Handle;
    sSPIPeriph *spiFlashHandle;
    FLASH_INFO *flashHandle;
    sTWI *twi0Handle;
    sTWI *twi1Handle;
    sTWI *twi2Handle;
    sTWI *si5356Handle;
    sTWI *softSwitchHandle;
    sTWI *adau1962TwiHandle;
    sTWI *adau1979TwiHandle;
    sTWI *a2bTwiHandle;
    sSPORT *dacSportOutHandle;
    sSPORT *adcSportInHandle;
    sSPORT *spdifSportOutHandle;
    sSPORT *spdifSportInHandle;
    sSPORT *a2bSportOutHandle;
    sSPORT *a2bSportInHandle;
    spiffs *spiffsHandle;

    /* Shell context */
    SHELL_CONTEXT shell;

    /* Task handles (used in 'stacks' command) */
    TaskHandle_t houseKeepingTaskHandle;
    TaskHandle_t pushButtonTaskHandle;
    TaskHandle_t startupTaskHandle;
    TaskHandle_t idleTaskHandle;
    TaskHandle_t a2bSlaveTaskHandle;
    TaskHandle_t a2bIrqTaskHandle;
    TaskHandle_t vuTaskHandle;

    /* A2B XML init items */
    void *a2bInitSequence;
    uint32_t a2bIinitLength;

    /* Audio ping/pong buffer pointers */
    void *codecAudioIn[2];
    void *codecAudioOut[2];
    void *spdifAudioIn[2];
    void *spdifAudioOut[2];
    void *a2bAudioIn[2];
    void *a2bAudioOut[2];

    /* Audio ping/pong buffer lengths */
    unsigned codecAudioInLen;
    unsigned codecAudioOutLen;
    unsigned spdifAudioInLen;
    unsigned spdifAudioOutLen;
    unsigned a2bAudioInLen;
    unsigned a2bAudioOutLen;

    /* Audio routing table */
    ROUTE_INFO *routingTable;

    /* APP config */
    APP_CFG cfg;

    /* Current time in mS */
    uint64_t now;

    /* A2B mode */
    A2B_BUS_MODE a2bmode;
    A2B_TO_SPORT_CFG_XCVR a2bxcvr;
    bool a2bPresent;
    bool a2bSlaveActive;
    bool discoverCmdStatus;
    bool a2bIrqDisable;
    unsigned a2bOutChannels;
    unsigned a2bInChannels;

    /* Clock domain management */
    uint32_t clockDomainMask[CLOCK_DOMAIN_MAX];
    uint32_t clockDomainActive[CLOCK_DOMAIN_MAX];
} APP_CONTEXT;

/*
 * Make the application context global for convenience
 */
extern APP_CONTEXT mainAppContext;

#endif
