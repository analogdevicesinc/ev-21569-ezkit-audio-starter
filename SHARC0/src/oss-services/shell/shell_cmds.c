/**
 * Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/*
 * Place code/data by default in external memory
 * This code has been modified by Analog Devices, Inc.
 */
#include "external_memory.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>

#include "FreeRTOS.h"
#include "task.h"

#include "context.h"
#include "syslog.h"
#include "twi_simple.h"
#include "shell.h"
#include "shell_printf.h"
#include "term.h"
#include "xmodem.h"
#include "util.h"
#include "clock_domain.h"

#ifdef printf
#undef printf
#endif

#ifdef vprintf
#undef vprintf
#endif

#define printf(...) shell_printf(ctx, __VA_ARGS__)
#define vprintf(x,y) shell_vprintf(ctx, x, y)

/***********************************************************************
 * Main application context
 **********************************************************************/
static APP_CONTEXT *context = &mainAppContext;

/***********************************************************************
 * Misc helper functions
 **********************************************************************/
static void dumpBytes(SHELL_CONTEXT *ctx, unsigned char *rdata, unsigned addr, unsigned rlen)
{
    unsigned i;
    for (i = 0; i < rlen; i++) {
        if ((i % 16) == 0) {
            if (i) {
                printf("\n");
            }
            printf("%08x: ", addr + i);
        }
        printf("%02x ", rdata[i]);
    }
    printf("\n");
}

static unsigned str2bytes(char *str, unsigned char **bytes)
{
    char *delim = " ,";
    char *token = NULL;
    unsigned wlen = 0;
    unsigned allocLen = 0;
    unsigned char *wdata = NULL;

    wlen = 0; allocLen = 0;
    token = strtok(str, delim);
    while (token) {
        if ((wlen + 1) > allocLen) {
            allocLen += 256;
            wdata = SHELL_REALLOC(wdata, allocLen * sizeof(*wdata));
        }
        wdata[wlen] = strtoul(token, NULL, 0);
        wlen++;
        token = strtok(NULL, delim);
    }

    if (bytes) {
        *bytes = wdata;
    }

    return(wlen);
}

enum {
    INVALID_FS = -1,
    SPIFFS_FS = 0,
    SD_FS,
    EMMC_FS
};

static int fsVolOK(SHELL_CONTEXT *ctx, APP_CONTEXT *context, char *fs)
{
    if (strcmp(fs, SPIFFS_VOL_NAME) == 0) {
        if (context->spiffsHandle) {
            return SPIFFS_FS;
        } else {
            printf("SPIFFS has not been initialized!\n");
            return INVALID_FS;
        }
    }
#if defined(SDCARD_VOL_NAME) && !defined(SDCARD_USE_EMMC)
    if (strcmp(fs, SDCARD_VOL_NAME) == 0) {
        if (context->sdcardHandle) {
            return SD_FS;
        } else {
            printf("SDCARD has not been initialized!\n");
            return INVALID_FS;
        }
    }
#endif
#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
    if (strcmp(fs, EMMC_VOL_NAME) == 0) {
        if (context->sdcardHandle) {
            return EMMC_FS;
        } else {
            printf("EMMC has not been initialized!\n");
            return INVALID_FS;
        }
    }
#endif
    printf("Invalid drive name specified.\n");
    return INVALID_FS;
}

/***********************************************************************
 * XMODEM helper functions
 **********************************************************************/
typedef struct XMODEM_STATE {
    SHELL_CONTEXT *ctx;
} XMODEM_STATE;

static void shell_xmodem_putchar(int c, void *usr)
{
    XMODEM_STATE *x = (XMODEM_STATE *)usr;
    SHELL_CONTEXT *ctx = x->ctx;
    TERM_STATE *t = &ctx->t;

    term_putch(t, c);
}

static int shell_xmodem_getchar(int timeout, void *usr)
{
    XMODEM_STATE *x = (XMODEM_STATE *)usr;
    SHELL_CONTEXT *ctx = x->ctx;
    TERM_STATE *t = &ctx->t;
    int c;

    c = term_getch(t, timeout);

    return(c);
}

typedef struct FLASH_WRITE_STATE {
    XMODEM_STATE xmodem;
    const FLASH_INFO *flash;
    unsigned addr;
    unsigned maxAddr;
    unsigned eraseBlockSize;
} FLASH_WRITE_STATE;

void flashDataWrite(void *usr, void *data, int size)
{
   FLASH_WRITE_STATE *state = (FLASH_WRITE_STATE *)usr;
   int err;

   if (size > 0) {
      if ((state->addr % state->eraseBlockSize) == 0) {
         err = flash_erase(state->flash, state->addr, state->eraseBlockSize);
         if (err != FLASH_OK) {
            return;
         }
      }
      if ((state->addr + size) <= state->maxAddr) {
         err = flash_program(state->flash, state->addr, (const unsigned char *)data, size);
         if (err != FLASH_OK) {
            return;
         }
         state->addr += size;
      }
   }
}

typedef struct FILE_XFER_STATE {
    XMODEM_STATE xmodem;
    FILE *f;
    void *data;
    int size;
} FILE_XFER_STATE;

void fileDataWrite(void *usr, void *data, int size)
{
    FILE_XFER_STATE *state = (FILE_XFER_STATE *)usr;
    size_t wsize;

    /*
     * Need to double-buffer the data in order to strip off the
     * trailing packet bytes at the end of an xmodem transfer.
     */
    if (state->data == NULL) {
        state->data = SHELL_MALLOC(1024);
        memcpy(state->data, data, size);
        state->size = size;
    } else {
        if (data) {
            wsize = fwrite(state->data, 1, state->size, state->f);
            memcpy(state->data, data, size);
            state->size = size;
        } else {
            uint8_t *buf = (uint8_t *)state->data;
            while (state->size && buf[state->size-1] == '\x1A') {
               state->size--;
            }
            wsize = fwrite(state->data, 1, state->size, state->f);
            if (state->data) {
                SHELL_FREE(state->data);
                state->data = NULL;
            }
        }
    }
}

void fileDataRead(void *usr, void *data, int size)
{
    FILE_XFER_STATE *state = (FILE_XFER_STATE *)usr;
    size_t rsize;

    if (size > 0) {
        rsize = fread(data, 1, size, state->f);
    }
}

int confirmDanger(SHELL_CONTEXT *ctx, char *warnStr)
{
    char c;

    printf( "%s\n", warnStr );
    printf( "Are you sure you want to continue? [y/n]" );

    c = term_getch( &ctx->t, TERM_INPUT_WAIT );
    printf( "%c\n", isprint( c ) ? c : ' ' );

    if( tolower(c) == 'y' ) {
        return(1);
    }

    return(0);
}

/***********************************************************************
 * CMD: dump
 **********************************************************************/
const char shell_help_dump[] = "[addr] <len>\n"
    "  addr - Starting address to dump\n"
    "  len - Number of bytes to dump (default 1)\n";
const char shell_help_summary_dump[] = "Hex dump of flash contents";

#include "flash.h"

void shell_dump(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    uintptr_t addr = 0;
    unsigned len = 1;
    uint8_t *buf;
    int ok;

    if (argc < 2) {
        printf("Invalid arguments\n");
        return;
    }

    addr = strtoul(argv[1], NULL, 0);
    if (argc > 2) {
        len = strtoul(argv[2], NULL, 0);
    }

    buf = SHELL_MALLOC(len);
    if (buf) {
        ok = flash_read(context->flashHandle, addr, buf, len);
        if (ok == FLASH_OK) {
            dumpBytes(ctx, buf, addr, len);
        }
        SHELL_FREE(buf);
    }
}

/***********************************************************************
 * CMD: fdump
 **********************************************************************/
const char shell_help_fdump[] =
  "[file] <start> <size>\n"
  "  file - File to dump\n"
  "  start - Start offset in bytes (default: 0)\n"
  "  size - Size in bytes (default: full file)\n";
const char shell_help_summary_fdump[] = "Dumps the contents of a file in hex";

#define DUMP_SIZE 512

void shell_fdump(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    FILE *f;
    size_t start = 0;
    size_t size = SIZE_MAX;
    uint8_t *buf = NULL;
    size_t rlen;
    int c;

    if( argc < 2 ) {
        printf("No file given\n");
        return;
    }

    if (argc > 2) {
        start = (size_t)strtoul(argv[2], NULL, 0);
    }

    if (argc > 3) {
        size = (size_t)strtoul(argv[3], NULL, 0);
    }

    f = fopen(argv[1], "r" );
    if (f) {
        buf = SHELL_MALLOC(DUMP_SIZE);
        fseek(f, start, SEEK_SET);
        do {
            rlen = (size < DUMP_SIZE) ? size : DUMP_SIZE;
            rlen = fread(buf, sizeof(*buf), rlen, f);
            if (rlen) {
                dumpBytes(ctx, buf, start, rlen);
                size -= rlen;
                start += rlen;
                c = term_getch(&ctx->t, TERM_INPUT_DONT_WAIT);
            }
        } while (size && rlen && (c < 0));
        if (buf) {
            SHELL_FREE(buf);
        }
        fclose(f);
    } else {
        printf("Unable to open '%s'\n", argv[1]);
    }
}

/***********************************************************************
 * CMD: recv
 **********************************************************************/
const char shell_help_recv[] = "<file>\n"
    "  Transfer and save to file\n";
const char shell_help_summary_recv[] = "Receive a file via XMODEM";

void shell_recv( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    FILE_XFER_STATE fileState = { 0 };
    long size;

    if( argc != 2 ) {
        printf( "Usage: recv <file>\n" );
        return;
    }

    fileState.xmodem.ctx = ctx;

    fileState.f = fopen( argv[ 1 ], "wb");
    if( fileState.f == NULL) {
        printf( "unable to open file %s\n", argv[ 1 ] );
        return;
    }
    printf( "Prepare your terminal for XMODEM send ... " );
    term_set_mode(&ctx->t, TERM_MODE_COOKED, 0);
    size = XmodemReceiveCrc(fileDataWrite, &fileState, INT_MAX,
        shell_xmodem_getchar, shell_xmodem_putchar);
    term_set_mode(&ctx->t, TERM_MODE_COOKED, 1);
    if (size < 0) {
        printf( "XMODEM Error: %ld\n", size);
    } else {
        printf( "received and saved as %s\n", argv[ 1 ] );
    }
    fclose( fileState.f );
}

/***********************************************************************
 * CMD: send
 **********************************************************************/
const char shell_help_send[] = "<file1> [<file2> ...]\n";
const char shell_help_summary_send[] = "Send files via YMODEM.";

#include "uart_stdio.h"

typedef struct YMODEM_STATE {
    FILE_XFER_STATE state;
    const char *fname;
    size_t fsize;
} YMODEM_STATE;

static void ymodem_hdr(void *usr, void *xmodemBuffer, int xmodemSize)
{
    YMODEM_STATE *y = (YMODEM_STATE *)usr;
    snprintf(xmodemBuffer, xmodemSize, "%s%c%u", y->fname, 0, (unsigned)y->fsize);
}

static void ymodem_end(void *xs, void *xmodemBuffer, int xmodemSize)
{
}

void shell_send(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    size_t size;
    FILE *fp = NULL;
    int ret = -1;
    int i;

    YMODEM_STATE y = {
        .state.xmodem.ctx = ctx,
    };

    if (argc < 2) {
        printf("Usage: %s <file1> [<file2> ...]\n", argv[0]);
        return;
    }

    printf ("Prepare your terminal for YMODEM receive...\n");
    term_set_mode(&ctx->t, TERM_MODE_COOKED, 0);
    for (i = 1; i < argc; i++) {
        fp = fopen( argv[i], "rb");
        if (fp) {
            fseek(fp, 0, SEEK_END);
            size = ftell(fp);
            fseek(fp, 0, SEEK_SET);
            y.fname = argv[i]; y.fsize = size; y.state.f = fp;
            ret = XmodemTransmit(ymodem_hdr, &y, 128, 0, 1,
                shell_xmodem_getchar, shell_xmodem_putchar);
            if (ret >= 0) {
                ret = XmodemTransmit(fileDataRead, &y, y.fsize, 1, 0,
                    shell_xmodem_getchar, shell_xmodem_putchar);
            }
            fclose(fp);
            if (ret < 0) {
                break;
            }
        }
    }
    if (ret >= 0) {
        ret = XmodemTransmit(ymodem_end, &y, 128, 0, 1,
            shell_xmodem_getchar, shell_xmodem_putchar);
    }
    term_set_mode(&ctx->t, TERM_MODE_COOKED, 1);
    if (ret < 0) {
        printf( "YMODEM Error: %ld\n", ret);
    }

}

/**********************************************************************
 * CMD: i2c
 **********************************************************************/
const char shell_help_i2c[] = "<i2c_port> <i2c_addr> <mem_addr> <wdata> <length> [addr_len]\n"
  "  i2c_port - I2C port to probe\n"
  "  i2c_addr - I2C device address\n"
  "  mem_addr - Starting memory address\n"
  "  wdata - Comma separated string of bytes to write (i.e. \"1,0x02,3\")\n"
  "          Empty quotes for read only.\n"
  "  length - Number of bytes to read.  Zero for write only.\n"
  "  addr_len - Number of address bytes (default 1)\n";
const char shell_help_summary_i2c[] = "Executes an I2C write/read transaction";

void shell_i2c(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    sTWI *twiHandle;
    TWI_SIMPLE_PORT twiPort;
    uint16_t  reg_addr;
    uint8_t  i2c_addr;
    long int  length;
    uint16_t  addrLength;
    uint8_t *twiWrBuffer = NULL;
    uint8_t *wdata = NULL;
    uint8_t twiWrLen;
    uint8_t *twiRdBuffer = NULL;
    TWI_SIMPLE_RESULT result;

    if (argc < 5) {
        printf( "Invalid arguments. Type help [<command>] for usage.\n" );
        return;
    }

    twiPort = (TWI_SIMPLE_PORT)strtol(argv[1], NULL, 0);
    twiHandle = NULL;

    if (twiPort >= TWI_END) {
        printf("Invalid I2C port!\n");
        return;
    }

    i2c_addr = strtol(argv[2], NULL, 0);
    reg_addr = strtol(argv[3], NULL, 0);
    twiWrLen = str2bytes(argv[4], &wdata);

    length = strtol(argv[5], NULL, 0);
    if ((length < 0) || (length > UINT16_MAX)) {
        printf("Invalid read length!\n");
        return;
    }

    addrLength = (reg_addr > 255) ? 2 : 1;
    if (argc > 6) {
        addrLength = strtol(argv[6], NULL, 0);
        if ((addrLength < 1) || (addrLength > 2)) {
            printf("Invalid address length!\n");
            return;
        }
    }

    /* Allocate a buffer to read into */
    if (length) {
        twiRdBuffer = SHELL_MALLOC(length);
    }

    /* Allocate a write buffer */
    twiWrBuffer = SHELL_MALLOC(addrLength + twiWrLen);

    /* See if requested TWI port is already open globally */
    if ((twiPort == TWI0) && context->twi0Handle) {
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI1) && context->twi1Handle) {
        twiHandle = context->twi1Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }

    if (twiHandle != NULL) {

        /* Print header if reading */
        if (length > 0) {
            printf ( "I2C Device (0x%02x): addr 0x%04x, bytes %d (0x%02x)\n",
                i2c_addr, reg_addr, length, length);
        }

        /* Do write/read of peripheral at the specified address */
        if (result == TWI_SIMPLE_SUCCESS) {
            if (addrLength == 2) {
                twiWrBuffer[0] = (reg_addr >> 8) & 0xFF;
                twiWrBuffer[1] = (reg_addr >> 0) & 0xFF;
            } else {
                twiWrBuffer[0] = reg_addr;
            }
            memcpy(twiWrBuffer + addrLength, wdata, twiWrLen);
            twiWrLen += addrLength;
            result = twi_writeRead(twiHandle, i2c_addr, twiWrBuffer, twiWrLen, twiRdBuffer, length);
            if (result == TWI_SIMPLE_SUCCESS) {
                if (length > 0) {
                    dumpBytes(ctx, twiRdBuffer, reg_addr, length);
                }
            } else {
                printf("twi twi_writeRead() error %d\n", result);
            }
        }
    } else {
        printf("I2C port %d is not configured in this project!\n", twiPort);
    }

    /* Free the write buffers */
    if (wdata) {
        SHELL_FREE(wdata);
    }
    if (twiWrBuffer) {
        SHELL_FREE(twiWrBuffer);
    }

    /* Free the read buffer */
    if (twiRdBuffer) {
        SHELL_FREE(twiRdBuffer);
    }
}

/***********************************************************************
 * CMD: i2c_probe
 **********************************************************************/
const char shell_help_i2c_probe[] = "<i2c_port>\n"
  "  i2c_port - I2C port to probe\n";
const char shell_help_summary_i2c_probe[] = "Probe an I2C port for active devices";

void shell_i2c_probe(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    sTWI *twiHandle;
    TWI_SIMPLE_RESULT result;
    TWI_SIMPLE_PORT twiPort;
    int i;

    if (argc != 2) {
        printf( "Invalid arguments. Type help [<command>] for usage.\n" );
        return;
    }

    twiPort = (TWI_SIMPLE_PORT)strtol(argv[1], NULL, 0);
    twiHandle = NULL;

    if (twiPort >= TWI_END) {
        printf("Invalid I2C port!\n");
        return;
    }

    /* See if requested TWI port is already open globally */
    if ((twiPort == TWI0) && context->twi0Handle) {
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI1) && context->twi1Handle) {
        twiHandle = context->twi1Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }

    if (twiHandle != NULL) {
        if (result == TWI_SIMPLE_SUCCESS) {
            printf ( "Probing I2C port %d:\n", twiPort);
            for (i = 0; i < 128; i++) {
                result = twi_write(twiHandle, i, NULL, 0);
                if (result == TWI_SIMPLE_SUCCESS) {
                    printf(" Found device 0x%02x\n", i);
                }
            }
        }
    } else {
        printf("I2C port %d is not configured in this project!\n", twiPort);
    }
}

/***********************************************************************
 * CMD: syslog
 **********************************************************************/
const char shell_help_syslog[] = "\n";
const char shell_help_summary_syslog[] = "Show the live system log";

#include "syslog.h"

#define MAX_TS_LINE  32
#define MAX_LOG_LINE 256

void shell_syslog(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    char *line;
    char *lbuf;
    char *ts;
    int c;

    ts = SHELL_MALLOC(MAX_TS_LINE);
    lbuf = SHELL_MALLOC(MAX_LOG_LINE);

    c = 0;
    do {
        line = syslog_next(ts, MAX_TS_LINE, lbuf, MAX_LOG_LINE);
        if (line) {
            printf("%s %s\n", ts, line);
        }
        c = term_getch(&ctx->t, TERM_INPUT_DONT_WAIT);
#ifdef FREE_RTOS
        if ((line == NULL) && (c < 0)) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
#endif
    } while (c < 0);

    if (ts) {
        SHELL_FREE(ts);
    }
    if (lbuf) {
        SHELL_FREE(lbuf);
    }
}

/***********************************************************************
 * CMD: drive
 **********************************************************************/
#include "fs_devman.h"

const char shell_help_drive[] = "<device> <action>\n"
  "  device - The device to take action on\n"
  "  action - The action to take on the device\n"
  " Valid actions\n"
  "  default - Sets the requested device as the default file system\n"
  " No arguments\n"
  "  Show all available devices\n";
const char shell_help_summary_drive[] = "Shows/supports filesystem device information";

void shell_drive(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    char *device;
    char *action;
    char *dirName;
    unsigned int dirIdx;
    const char *pcDeviceDefault;

    /* Local Inits */
    pcDeviceDefault = NULL;

    if((argc == 2) || (argc > 3))
    {
       printf("Invalid arguments. Type help [<command>] for usage.\n");
       return;
    }

    /* Get the default device */
    fs_devman_get_default(&pcDeviceDefault);

    if(argc == 1)
    {
        printf("Available devices are:\n");
        for(dirIdx = 0u; dirIdx < fs_devman_get_num_devices(); dirIdx++)
        {
            if(fs_devman_get_deviceName(dirIdx, (const char **)&dirName) == FS_DEVMAN_OK)
            {
                printf("%s %s\n",
                    dirName, (pcDeviceDefault == dirName) ? "(Default)" : "");
            }
        }
    }
    else
    {
        /* Get the device and action and verify the arguments */
        device = argv[1];
        action = argv[2];

        if(fs_devman_is_device_valid(device) == true)
        {
            if(strcmp(action, "default") == 0)
            {
                if(fs_devman_set_default(device) == FS_DEVMAN_OK)
                {
                   printf("Succesfully set %s to default drive!\n", device);
                }
                else
                {
                   printf("Could not set %s to default drive!\n", device);
                }
            }
            else
            {
                printf("Invalid action. Type help [<command>] for usage.\n");
            }
        }
    }
}

/***********************************************************************
 * CMD: ls
 **********************************************************************/
const char shell_help_ls[] = "<device>\n";
const char shell_help_summary_ls[] = "Shows a device directory listing";

static void shell_ls_helper( SHELL_CONTEXT *ctx, const char *crtname, int recursive, int *phasdirs )
{
  void *d;
  uint32_t total = 0;
  FS_DEVMAN_DIRENT *ent;
  int ndirs = 0;
  unsigned year ,month, day, hour, min, sec;

  if( ( d = fs_devman_opendir( crtname ) ) != NULL )
  {
    total = 0;
    printf( "%s", crtname );
    while( ( ent = fs_devman_readdir( d ) ) != NULL )
    {
      printf("\n");
      if( ent->flags & FS_DEVMAN_DIRENT_FLAG_DIR )
      {
        printf( "%12s ", "<DIR>" );
        ndirs = ndirs + 1;
        if( phasdirs )
          *phasdirs = 1;
      }
      else
      {
        printf( "%12u ", ( unsigned )ent->fsize );
        total = total + ent->fsize;
      }
      if (ent->fdate) {
          year = ((ent->fdate >> 9) & 0x7F) + 1980;
          month = (ent->fdate >> 5) & 0xF;
          day = (ent->fdate >> 0) & 0x1F;
          printf("%04u-%02u-%02u ", year, month, day);
    }
      if (ent->ftime) {
          hour = (ent->ftime >> 11) & 0x1F;
          min = (ent->ftime >> 5) & 0x3F;
          sec = ((ent->ftime >> 0) & 0x1F) * 2;
          printf("%02u:%02u:%02u ", hour, min, sec);
      }
      printf( " %s", ent->fname );
    }
    fs_devman_closedir( d );
    printf("\n");
  }
}

void shell_ls(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    FS_DEVMAN_RESULT result;
    int phasdirs;
    char *device;

    if (argc == 1) {
        result = fs_devman_get_default((const char **)&device);
        if (result != FS_DEVMAN_OK) {
            return;
        }
    } else {
        device = argv[1];
    }

    shell_ls_helper(ctx, device, 0, &phasdirs );
}

/***********************************************************************
 * CMD: format
 **********************************************************************/
const char shell_help_format[] = "["
    SPIFFS_VOL_NAME
#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
    " | " EMMC_VOL_NAME
#endif
    "]\n";
const char shell_help_summary_format[] = "Formats an internal flash filesystem";

#include "fs_devman.h"

#include "spiffs_fs.h"
#include "fs_dev_spiffs.h"
#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
#include "ff.h"
#endif

void shell_format(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    FS_DEVMAN_RESULT fsdResult;
    FS_DEVMAN_DEVICE *device;
    const char *ddname;
    int fs = INVALID_FS;
    bool dd = false;

    fs = fsVolOK(ctx, context, (argc > 1) ? argv[1] : "");
    if (fs == INVALID_FS) {
        return;
    }

    printf("Be patient, this may take a while.\n");
    printf("Formatting...\n");

    if (fs == SPIFFS_FS) {
        if (fs_devman_is_device_valid(SPIFFS_VOL_NAME)) {
            fsdResult = fs_devman_get_default(&ddname);
            if (fsdResult == FS_DEVMAN_OK) {
                dd = (strcmp(ddname, SPIFFS_VOL_NAME) == 0);
            }
            fs_devman_unregister(SPIFFS_VOL_NAME);
        }
        spiffs_format(context->spiffsHandle);
        device = fs_dev_spiffs_device();
        fsdResult = fs_devman_register(SPIFFS_VOL_NAME, device, context->spiffsHandle);
        if (dd) {
            fsdResult = fs_devman_set_default(SPIFFS_VOL_NAME);
        }
    }
#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
    if (fs == EMMC_FS) {
        FRESULT res;
        MKFS_PARM opt = {
            .fmt = FM_FAT32
        };
        res = f_mkfs(EMMC_VOL_NAME, &opt, NULL, 512*16);
        if (res == FR_OK) {
            context->sdRemount = true;
        }
    }
#endif
    if (fs == SD_FS) {
        printf("Not supported.\n");
    }

    printf("Done.\n");
}

/***********************************************************************
 * CMD: discover
 **********************************************************************/
#include "init.h"
#include "a2b_xml.h"
#include "adi_a2b_cmdlist.h"
#include "a2b_irq.h"

const char shell_help_discover[] = "<a2b.xml> <verbose> <i2c_port> <i2c_addr>\n"
  "  a2b.xml  - A SigmaStudio A2B XML config export file\n"
  "             default 'a2b.xml'\n"
  "  verbose  - Print out results to 0:none, 1:stdout, 2:syslog\n"
  "             default: 1\n"
  "  i2c_port - Set the AD242x transceiver I2C port.\n"
  "             default: 2 (TWI2)\n"
  "  i2c_addr - Set the AD242x transceiver I2C address\n"
  "             default: 0x68\n";
const char shell_help_summary_discover[] = "Discovers an A2B network";

static ADI_A2B_CMDLIST_RESULT shell_discover_twi_read(
    void *twiHandle, uint8_t address,
    void *in, uint16_t inLen, void *usr)
{
    TWI_SIMPLE_RESULT twiResult;
    twiResult = twi_read(twiHandle, address, in, inLen);
    return (twiResult == TWI_SIMPLE_SUCCESS ?
        ADI_A2B_CMDLIST_SUCCESS : ADI_A2B_CMDLIST_A2B_I2C_READ_ERROR);
    }

static ADI_A2B_CMDLIST_RESULT shell_discover_twi_write(
    void *twiHandle, uint8_t address,
    void *out, uint16_t outLen, void *usr)
{
    TWI_SIMPLE_RESULT twiResult;
    twiResult = twi_write(twiHandle, address, out, outLen);
    return (twiResult == TWI_SIMPLE_SUCCESS ?
        ADI_A2B_CMDLIST_SUCCESS : ADI_A2B_CMDLIST_A2B_I2C_WRITE_ERROR);
}

static ADI_A2B_CMDLIST_RESULT shell_discover_twi_write_write(
    void *twiHandle, uint8_t address,
    void *out, uint16_t outLen, void *out2, uint16_t out2Len, void *usr)
{
    TWI_SIMPLE_RESULT twiResult;
    twiResult = twi_writeWrite(twiHandle, address, out, outLen, out2, out2Len);
    return (twiResult == TWI_SIMPLE_SUCCESS ?
        ADI_A2B_CMDLIST_SUCCESS : ADI_A2B_CMDLIST_A2B_I2C_WRITE_ERROR);
}

static ADI_A2B_CMDLIST_RESULT shell_discover_twi_write_read(
    void *twiHandle, uint8_t address,
    void *out, uint16_t outLen, void *in, uint16_t inLen, void *usr)
{
    TWI_SIMPLE_RESULT twiResult;
    twiResult = twi_writeRead(twiHandle, address, out, outLen, in, inLen);
    return (twiResult == TWI_SIMPLE_SUCCESS ?
        ADI_A2B_CMDLIST_SUCCESS : ADI_A2B_CMDLIST_A2B_I2C_WRITE_ERROR);
}

static void shell_discover_delay(uint32_t mS, void *usr)
{
    vTaskDelay(pdMS_TO_TICKS(mS));
}

static uint32_t shell_discover_get_time(void *usr)
{
    return(xTaskGetTickCount() * (1000 / configTICK_RATE_HZ));
}

static void *shell_discover_get_buffer(uint16_t size, void *usr)
{
    return(SHELL_MALLOC(size));
}

static void shell_discover_free_buffer(void *buffer, void *usr)
{
    SHELL_FREE(buffer);
}

static void shell_discover_log (
    bool newLine, void *usr, const char *fmt, va_list va)
{
    static char *logline = NULL;
    char *str = NULL;
    char *l;
    va_list _va;
    int len;

    /* Flush if newline or done */
    if (newLine || (fmt == NULL)) {
        if (logline) {
            syslog_print(logline);
            SHELL_FREE(logline);
            logline = NULL;
        }
    }

    /* Make new string */
    if (fmt) {
        _va = va;
        len = vsnprintf(NULL, 0, fmt, _va);
        _va = va;
        str = SHELL_MALLOC(len + 1);
        vsnprintf(str, len + 1, fmt, _va);
    }

    /* Concat */
    if (str) {
        len = logline ? strlen(logline) : 0;
        len += strlen(str) + 1;
        l = SHELL_REALLOC(logline, len);
        if (logline == NULL) {
            *l = '\0';
        }
        logline = l;
        strcat(logline, str);
    }

    /* Free */
    if (str) {
        SHELL_FREE(str);
    }
}


typedef int (*PF)(SHELL_CONTEXT *ctx, const char *restrict format, ...);

int shell_syslog_vprintf(SHELL_CONTEXT *ctx, const char *restrict format, ...)
{
    va_list va;
    va_start(va, format);
    syslog_vprintf((char *)format, va);
    va_end(va);
    return(0);
}

void shell_discover(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    const char *fileName = "a2b.xml";
    ADI_A2B_CMDLIST_RESULT cmdListResult;
    void *a2bInitSequence;
    uint32_t a2bIinitLength;
    int verbose;
    sTWI *twiHandle;
    TWI_SIMPLE_RESULT result;
    TWI_SIMPLE_PORT twiPort;
    uint8_t ad2425I2CAddr;
    A2B_CMD_TYPE a2bCmdType;
    ADI_A2B_CMDLIST *list;
    ADI_A2B_CMDLIST_EXECUTE_INFO execInfo;
    ADI_A2B_CMDLIST_SCAN_INFO scanInfo;
    ADI_A2B_CMDLIST_OVERRIDE_INFO overrideInfo;
    PF pf;

    ADI_A2B_CMDLIST_CFG cfg = {
        .twiRead = shell_discover_twi_read,
        .twiWrite = shell_discover_twi_write,
        .twiWriteRead = shell_discover_twi_write_read,
        .twiWriteWrite = shell_discover_twi_write_write,
        .delay = shell_discover_delay,
        .getTime = shell_discover_get_time,
        .getBuffer = shell_discover_get_buffer,
        .freeBuffer = shell_discover_free_buffer,
        .log = shell_discover_log,
        .usr = ctx
    };

    /* Determine file name */
    if (argc >= 2) {
        fileName = (const char *)argv[1];
    } else {
        fileName = "a2b.xml";
    }

    /* Determine verbosity */
    if (argc >= 3) {
        verbose = strtol(argv[2], NULL, 0);
    } else {
        verbose = 1;
    }
    if (verbose == 1) {
        pf = shell_printf;
    } else if (verbose == 2) {
        pf = shell_syslog_vprintf;
    } else {
        pf = NULL;
    }

    /* Determine TWI port */
    if (argc >= 4) {
        twiPort = (TWI_SIMPLE_PORT)strtol(argv[3], NULL, 0);
    } else {
        twiPort = TWI2;
    }
    if (twiPort >= TWI_END) {
        if (pf) {
            pf(ctx, "Invalid I2C port!\n");
        }
        return;
    }

    /* See if requested TWI port is already open globally */
    twiHandle = NULL;
    if ((twiPort == TWI0) && context->twi0Handle) {
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI1) && context->twi1Handle) {
        twiHandle = context->twi1Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }
    if (twiHandle == NULL) {
        if (pf) {
            pf(ctx, "I2C port %d is not configured in this project!\n", twiPort);
        }
        return;
    }

    /* Save the TWI handle to use */
    cfg.handle = (void *)twiHandle;

    /* Determine AD2425 I2C address */
    if (argc >= 5) {
        ad2425I2CAddr = strtol(argv[4], NULL, 0);
    } else {
        ad2425I2CAddr = context->cfg.a2bI2CAddr;
    }

    /* Load the A2B network init */
    a2bIinitLength = a2b_xml_load(fileName, &a2bInitSequence, &a2bCmdType);

    /* If successful, play out the binary init sequence */
    if (a2bIinitLength && a2bInitSequence) {

        /* Open a command list instance */
        cmdListResult = adi_a2b_cmdlist_open(&list, &cfg);

        /* Set the command list */
        cmdListResult =  adi_a2b_cmdlist_set(
            list, ad2425I2CAddr, a2bInitSequence, a2bIinitLength, a2bCmdType
        );

        /* Clear the overrides */
        memset(&overrideInfo, 0, sizeof(overrideInfo));

        /* Override default SigmaStudio address */
        overrideInfo.masterAddr_override = true;
        overrideInfo.masterAddr = ad2425I2CAddr;

        /* Confirm master I2S/TDM settings and override if they don't match */
        cmdListResult = adi_a2b_cmdlist_scan(
            list, &scanInfo
        );
        if (scanInfo.I2SGCFG_valid && (scanInfo.I2SGCFG != SYSTEM_I2SGCFG)) {
            if (pf) {
                pf(ctx, "WARNING: I2SGCFG mismatch (expected %02x, got %02x)\n",
                    SYSTEM_I2SGCFG, scanInfo.I2SGCFG);
                pf(ctx, "         Overriding...\n");
            }
            overrideInfo.I2SGCFG_override = true;
            overrideInfo.I2SGCFG = SYSTEM_I2SGCFG;
        }
        if (scanInfo.I2SCFG_valid && (scanInfo.I2SCFG != SYSTEM_I2SCFG)) {
            if (pf) {
                pf(ctx, "WARNING: I2SCFG mismatch (expected %02x, got %02x)\n",
                    SYSTEM_I2SCFG, scanInfo.I2SCFG);
                pf(ctx, "         Overriding...\n");
            }
            overrideInfo.I2SCFG_override = true;
            overrideInfo.I2SCFG = SYSTEM_I2SCFG;
        }

        /* Process any overrides */
        cmdListResult = adi_a2b_cmdlist_override(list, &overrideInfo);

        /* Disable A2B IRQ processing */
        a2b_irq_disable(context, A2B_BUS_NUM_1);

        /* Run the command list */
        cmdListResult = adi_a2b_cmdlist_execute(list, &execInfo);

        /* Reenable A2B IRQ processing */
        a2b_irq_enable(context, A2B_BUS_NUM_1);

        if (pf) {
            pf(ctx, "A2B config lines processed: %lu\n", execInfo.linesProcessed);
            pf(ctx, "A2B discovery result: %s\n", execInfo.resultStr);
            pf(ctx, "A2B nodes discovered: %d\n", execInfo.nodesDiscovered);
        }

        /* Close the command list */
        cmdListResult = adi_a2b_cmdlist_close(&list);

        /* Free the network config */
        a2b_xml_free(a2bInitSequence, a2bIinitLength, a2bCmdType);

    } else {
        if (pf) {
            pf(ctx, "Error loading '%s' A2B init XML file\n", fileName);
        }
    }

}

/***********************************************************************
 * CMD: df
 **********************************************************************/
const char shell_help_df[] = "["
    SPIFFS_VOL_NAME
#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
    " | " EMMC_VOL_NAME
#elif defined(SDCARD_VOL_NAME)
    " | " SDCARD_VOL_NAME
#endif
    "]\n";
const char shell_help_summary_df[] = "Shows internal filesystem disk full status";

#if defined(EMMC_VOL_NAME) && defined(SDCARD_USE_EMMC)
#define DF_SD_EMMC_VOL_NAME  EMMC_VOL_NAME
#elif defined(SDCARD_VOL_NAME)
#define DF_SD_EMMC_VOL_NAME  SDCARD_VOL_NAME
#endif

#include "spiffs.h"
#ifdef DF_SD_EMMC_VOL_NAME
#include "ff.h"
#endif

void shell_df( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    int fs = INVALID_FS;
    int sf = 0;
    int emmc = 0;
    int sd = 0;

    if (argc > 1) {
        fs = fsVolOK(ctx, context, (argc > 1) ? argv[1] : SPIFFS_VOL_NAME);
        if (fs == INVALID_FS) {
            return;
        }
        sf = (fs == SPIFFS_FS); sd = (fs == SD_FS); emmc = (fs == EMMC_FS);
    } else {
        sf = 1; sd = 1; emmc = 1;
    }

    printf("%-10s %10s %10s %10s %5s\n", "Filesystem", "Size", "Used", "Available", "Use %");

    if (sf) {
        s32_t serr; u32_t ssize; u32_t sused;
        serr = SPIFFS_info(context->spiffsHandle, &ssize, &sused);
        if (serr == SPIFFS_OK) {
          printf("%-10s %10u %10u %10u %5u\n", SPIFFS_VOL_NAME,
            (unsigned)ssize, (unsigned)sused, (unsigned)(ssize - sused),
            (unsigned)((100 * sused) / ssize));
        }
    }
#ifdef DF_SD_EMMC_VOL_NAME
    if (sd || emmc) {
        FRESULT res;
        FATFS *fs;
        DWORD fre_clust, fre_blk, tot_blk, used_blk;
        res = f_getfree(DF_SD_EMMC_VOL_NAME, &fre_clust, &fs);
        tot_blk = ((fs->n_fatent - 2) * fs->csize)/2;
        fre_blk = (fre_clust * fs->csize)/2;
        used_blk = tot_blk - fre_blk;
        if (res == FR_OK) {
            printf("%-10s %10u %10u %10u %5u\n", DF_SD_EMMC_VOL_NAME,
                (unsigned)tot_blk, (unsigned)used_blk, (unsigned)fre_blk,
                (unsigned)((100 * used_blk) / tot_blk));
        }
    }
#endif
}

/***********************************************************************
 * CMD: rm/del
 **********************************************************************/
const char shell_help_rm[] = "<file1> [<file2> ...]\n";
const char shell_help_summary_rm[] = "Removes a file";

#include <stdio.h>

void shell_rm( SHELL_CONTEXT *ctx, int argc, char **argv )
{
  int i;

  if (argc < 2) {
    printf( "Usage: rm <file1> [<file2> ...]\n" );
    return;
  }

  for (i = 1; i < argc; i++) {
    if (remove(argv[i]) != 0) {
      printf("Unable to remove '%s'\n", argv[i]);
    }
  }
}

/***********************************************************************
 * CMD: cat
 **********************************************************************/
const char shell_help_cat[] = "\n";
const char shell_help_summary_cat[] = "Print file on standard output";

void shell_cat( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    FILE *handle;
    unsigned i;
    unsigned char c;

    if( argc < 2 ) {
        printf( "Usage: cat <filename1> [<filename2> ...]\n" );
        return;
    }
    for( i = 1; i < argc; i ++ ) {
        if( ( handle = fopen( argv[ i ], "r" ) ) > 0 )
        {
            while (fread(&c, sizeof(c), 1, handle) > 0) {
                printf("%c", c);
            }
            fclose(handle);
        } else {
            printf( "Unable to open '%s'\n", argv[ i ] );
        }
    }
}

/***********************************************************************
 * CMD: copy/cp
 **********************************************************************/
const char shell_help_cp[] = "<src> <dst>\n";
const char shell_help_summary_cp[] = "Copy source file <src> to <dst>";

#define SHELL_COPY_BUFSIZE    256

void shell_cp( SHELL_CONTEXT *ctx, int argc, char **argv )
{
   FILE *fps = NULL, *fpd = NULL;
   void *buf = NULL;
   size_t datalen, datawrote, total = 0;

   if( argc != 3 ) {
      printf( "Usage: cp <source> <destination>\n" );
      return;
   }

   if( ( fps = fopen( argv[ 1 ], "r" ) ) == NULL ) {
      printf( "Unable to open %s for reading\n", argv[ 1 ] );
   } else {
      if( ( fpd = fopen( argv[ 2 ], "w" ) ) == NULL ) {
         printf( "Unable to open %s for writing\n", argv[ 2 ] );
      } else {
         if( ( buf = SHELL_MALLOC( SHELL_COPY_BUFSIZE ) ) == NULL ) {
            printf( "Not enough memory\n" );
         } else {
            while( 1 ) {
               datalen = fread( buf, 1, SHELL_COPY_BUFSIZE, fps );
               datawrote = fwrite( buf, 1, datalen, fpd );
               if( datawrote < datalen ) {
                  printf( "Copy error (no space left on target?)\n" );
                  break;
               }
               total += datalen;
               if( datalen < SHELL_COPY_BUFSIZE ) {
                  break;
               }
            }
            fflush( fpd );
            printf( "%u bytes copied\n", ( unsigned int )total );
         }
      }
   }

   if( fps ) {
      fclose( fps );
   }
   if( fpd ) {
      fclose( fpd );
   }
   if( buf ) {
      SHELL_FREE( buf );
   }
}


/***********************************************************************
 * CMD: run
 **********************************************************************/
const char shell_help_run[] = "<file1> [<file2> ...]\n";
const char shell_help_summary_run[] = "Runs a command file";

void shell_run( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    FILE *f = NULL;
    char *cmd = NULL;
    char *ok = NULL;
    int i;

    if (argc < 2) {
        printf( "Usage: run <file1> [<file2> ...]\n" );
        return;
    }

    cmd = SHELL_MALLOC(SHELL_MAX_LINE_LEN);

    for (i = 1; i < argc; i++) {
        f = fopen(argv[i], "r");
        if (f) {
            ok = NULL;
            do {
                ok = fgets(cmd, SHELL_MAX_LINE_LEN, f);
                if (ok == cmd) {
                    if (cmd[0] == ';' || cmd[0] == '#') {
                        continue;
                    }
                    shell_exec(ctx, cmd);
                }
            } while (ok);
            fclose(f);
        } else {
            if (ctx->interactive) {
                printf("Failed to open '%s'\n", argv[i]);
            } else {
                syslog_printf("Failed to open '%s'\n", argv[i]);
            }
        }
    }

    SHELL_FREE(cmd);
}

/***********************************************************************
 * CMD: stacks
 **********************************************************************/
const char shell_help_stacks[] = "\n";
const char shell_help_summary_stacks[] = "Report task stack usage";

static void shell_print_task_stack(SHELL_CONTEXT *ctx, TaskHandle_t task)
{
    if (task) {
        printf(" %s: %u\n",
            pcTaskGetName(task), (unsigned)uxTaskGetStackHighWaterMark(task)
        );
    }
}

void shell_stacks( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    APP_CONTEXT *context = &mainAppContext;

    printf("High Water Marks are in 32-bit words (zero is bad).\n");
    printf("Task Stask High Water Marks:\n");

    shell_print_task_stack(ctx, context->startupTaskHandle);
    shell_print_task_stack(ctx, context->houseKeepingTaskHandle);
    shell_print_task_stack(ctx, context->pushButtonTaskHandle);
    shell_print_task_stack(ctx, context->a2bSlaveTaskHandle);
    shell_print_task_stack(ctx, context->a2bIrqTaskHandle);
    shell_print_task_stack(ctx, context->vuTaskHandle);
}

/***********************************************************************
 * CMD: cpu
 **********************************************************************/
const char shell_help_cpu[] = "\n";
const char shell_help_summary_cpu[] = "Report cpu usage";

#include "cpu_load.h"

void shell_cpu( SHELL_CONTEXT *ctx, int argc, char **argv )
{
    uint32_t percentCpuLoad, maxCpuLoad;

    percentCpuLoad = cpuLoadGetLoad(&maxCpuLoad, true);
    printf("SHARC0 CPU Load: %u%% (%u%% peak)\n",
        (unsigned)percentCpuLoad, (unsigned)maxCpuLoad);
}

/***********************************************************************
 * CMD: fsck
 **********************************************************************/
const char shell_help_fsck[] = "["SPIFFS_VOL_NAME"]\n";
const char shell_help_summary_fsck[] = "Check the internal filesystem";

#include "spiffs.h"

void shell_fsck(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    int fs = INVALID_FS;
    s32_t ok;

    fs = fsVolOK(ctx, context, (argc > 1) ? argv[1] : SPIFFS_VOL_NAME);
    if (fs == INVALID_FS) {
        return;
    }

    printf("Be patient, this may take a while.\n");
    printf("Checking...\n");

    if (fs == SPIFFS_FS) {
        ok = SPIFFS_check(context->spiffsHandle);
        if (ok == SPIFFS_OK) {
            printf(SPIFFS_VOL_NAME " OK\n");
        } else {
            printf(SPIFFS_VOL_NAME " CORRUPT: %d\n", (int)ok);
        }
    }

    if ((fs == EMMC_FS) || (fs == SD_FS)) {
        printf("Not supported.\n");
    }
}

/***********************************************************************
 * CMD: update
 **********************************************************************/
#include "flash_map.h"

const char shell_help_update[] = "\n";
const char shell_help_summary_update[] = "Updates the firmware via xmodem";

void shell_update(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    char *warn = "Updating the firmware is DANGEROUS - DO NOT REMOVE POWER!";
    FLASH_WRITE_STATE state;
    const FLASH_INFO *flash;
    long size;

    /* Confirm action */
    if (confirmDanger(ctx, warn) == 0) {
        return;
    }

    /* Get a handle to the system flash */
    flash = context->flashHandle;
    if (flash == NULL) {
        printf("Flash not initialized!\n");
        return;
    }

    /* Configure the update */
    state.flash = flash;
    state.addr = APP_OFFSET;
    state.maxAddr = APP_OFFSET + APP_SIZE;
    state.eraseBlockSize = ERASE_BLOCK_SIZE;
    state.xmodem.ctx = ctx;

    /* Start the update */
    printf( "Start XMODEM transfer now... ");
    term_set_mode(&ctx->t, TERM_MODE_COOKED, 0);
    size = XmodemReceiveCrc(flashDataWrite, &state, INT_MAX,
        shell_xmodem_getchar, shell_xmodem_putchar);
    term_set_mode(&ctx->t, TERM_MODE_COOKED, 1);

    /* Wait a bit */
    delay(100);

    /* Display results and erase any remaining space */
    if (size < 0) {
       printf( "XMODEM Error: %ld\n", size);
    } else {
       printf("Received %ld bytes.\n", size);
    }

    printf("Done.\n");
}

/***********************************************************************
 * CMD: meminfo
 **********************************************************************/
const char shell_help_meminfo[] = "\n";
const char shell_help_summary_meminfo[] = "Displays UMM_MALLOC heap statistics";

#include "umm_malloc_cfg.h"
#include "umm_malloc_heaps.h"
const static char *heapNames[] = UMM_HEAP_NAMES;

void shell_meminfo(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    int i;
    int ok;

    /* UMM Malloc */
    UMM_HEAP_INFO ummHeapInfo;
    for (i = 0; i < UMM_NUM_HEAPS; i++) {
        printf("Heap %s Info:\n", heapNames[i]);
        ok = umm_integrity_check((umm_heap_t)i);
        if (ok) {
            umm_info((umm_heap_t)i, &ummHeapInfo, NULL, 0);
            printf("  Entries: Total  %8i, Allocated %8i, Free %8i\n",
                ummHeapInfo.totalEntries,
                ummHeapInfo.usedEntries,
                ummHeapInfo.freeEntries
            );
            printf("   Blocks: Total  %8i, Allocated %8i, Free %8i\n",
                ummHeapInfo.totalBlocks,
                ummHeapInfo.usedBlocks,
                ummHeapInfo.freeBlocks
            );
            printf("   Contig: Blocks %8i,     Bytes %8i\n",
                ummHeapInfo.maxFreeContiguousBlocks,
                ummHeapInfo.maxFreeContiguousBlocks * umm_block_size()
            );
        }
        printf("  Heap Integrity: %s\n", ok ? "OK" : "Corrupt");
    }

    /* FreeRTOS */
    HeapStats_t rtosHeapStats;
    vPortGetHeapStats(&rtosHeapStats);

    printf("FreeRTOS Info:\n");
    printf("     Free: Blocks %8u,   Current %8u, Min  %8u\n",
        (unsigned)rtosHeapStats.xNumberOfFreeBlocks,
        (unsigned)rtosHeapStats.xAvailableHeapSpaceInBytes,
        (unsigned)rtosHeapStats.xMinimumEverFreeBytesRemaining
    );
}

/***********************************************************************
 * CMD: cmdlist
 **********************************************************************/
const char shell_help_cmdlist[] = "<cmdlist.xml> <verbose> <i2c_port>\n"
  "  cmdlist.xml  - A SigmaStudio XML command list file\n"
  "                 default 'cmdlist.xml'\n"
  "  verbose  - Print out results to 0:none, 1:stdout, 2:syslog\n"
  "             default: 1\n"
  "  i2c_port - Set the AD242x transceiver I2C port.\n"
  "             default: 2 (TWI2)\n";
const char shell_help_summary_cmdlist[] = "Plays a SigmaStudio XML command list";

void shell_cmdlist(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    const char *fileName = "cmdlist.xml";
    ADI_A2B_CMDLIST_RESULT cmdListResult;
    void *a2bInitSequence;
    uint32_t a2bIinitLength;
    int verbose;
    sTWI *twiHandle;
    TWI_SIMPLE_RESULT result;
    TWI_SIMPLE_PORT twiPort;
    A2B_CMD_TYPE a2bCmdType;
    ADI_A2B_CMDLIST *list;
    PF pf;

    ADI_A2B_CMDLIST_CFG cfg = {
        .twiRead = shell_discover_twi_read,
        .twiWrite = shell_discover_twi_write,
        .twiWriteRead = shell_discover_twi_write_read,
        .twiWriteWrite = shell_discover_twi_write_write,
        .delay = shell_discover_delay,
        .getTime = shell_discover_get_time,
        .getBuffer = shell_discover_get_buffer,
        .freeBuffer = shell_discover_free_buffer,
        .log = shell_discover_log,
        .usr = context
    };

    /* Determine file name */
    if (argc >= 2) {
        fileName = (const char *)argv[1];
    }

    /* Determine verbosity */
    if (argc >= 3) {
        verbose = strtol(argv[2], NULL, 0);
    } else {
        verbose = 1;
    }
    if (verbose == 1) {
        pf = shell_printf;
    } else if (verbose == 2) {
        pf = shell_syslog_vprintf;
    } else {
        pf = NULL;
    }

    /* Determine TWI port */
    if (argc >= 4) {
        twiPort = (TWI_SIMPLE_PORT)strtol(argv[3], NULL, 0);
    } else {
        twiPort = TWI2;
    }
    if (twiPort >= TWI_END) {
        if (pf) {
            pf(ctx, "Invalid I2C port!\n");
        }
        return;
    }

    /* See if requested TWI port is already open globally */
    twiHandle = NULL;
    if ((twiPort == TWI0) && context->twi0Handle) {
        twiHandle = context->twi0Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI1) && context->twi1Handle) {
        twiHandle = context->twi1Handle;
        result = TWI_SIMPLE_SUCCESS;
    } else if ((twiPort == TWI2) && context->twi2Handle) {
        twiHandle = context->twi2Handle;
        result = TWI_SIMPLE_SUCCESS;
    }
    if (twiHandle == NULL) {
        if (pf) {
            pf(ctx, "I2C port %d is not configured in this project!\n", twiPort);
        }
        return;
    }

    /* Save the TWI handle to use */
    cfg.handle = (void *)twiHandle;

    /* Load the command list */
    a2bIinitLength = a2b_xml_load(fileName, &a2bInitSequence, &a2bCmdType);

    /* If successful, play out the binary init sequence */
    if (a2bIinitLength && a2bInitSequence) {

        /* Open a command list instance */
        cmdListResult = adi_a2b_cmdlist_open(&list, &cfg);

        /* Set the command list */
        cmdListResult =  adi_a2b_cmdlist_set(
            list, 0, a2bInitSequence, a2bIinitLength, a2bCmdType
        );

        /* Run the command list */
        cmdListResult = adi_a2b_cmdlist_play(list);
        if (cmdListResult != ADI_A2B_CMDLIST_SUCCESS) {
            if (pf) {
                pf(ctx, "Error processing command list\n");
            }
        }

        /* Close the command list */
        adi_a2b_cmdlist_close(&list);

        /* Free the network config */
        a2b_xml_free(a2bInitSequence, a2bIinitLength, a2bCmdType);

    } else {
        if (pf) {
            pf(ctx, "Error loading '%s' A2B init XML file\n", fileName);
        }
    }

}

/***********************************************************************
 * CMD: resize
 **********************************************************************/
const char shell_help_resize[] = "[columns [lines]]\n";
const char shell_help_summary_resize[] = "Resize/Sync terminal window";

#include "term.h"

void shell_resize(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    uint32_t now;
    unsigned cols, lines;

    /* Resync screen size */
    term_sync_size(&ctx->t);
    now = rtosTimeMs();
    while (rtosTimeMs() < (now + 100)) {
        term_getch(&ctx->t, TERM_INPUT_DONT_WAIT);
    }

    /* Get latest size */
    cols = term_get_cols(&ctx->t);
    lines = term_get_lines(&ctx->t);

    /* Set terminal window size */
    if (argc > 1) {
        cols = atoi(argv[1]);
        if (argc > 2) {
            lines = atoi(argv[2]);
        }
        term_set_size(&ctx->t, cols, lines);
    }

    /* Show latest size */
    if (argc == 1) {
        printf("Size: %u x %u\n", cols, lines);
        return;
    }
}

/***********************************************************************
 * CMD: test
 **********************************************************************/
const char shell_help_test[] = "\n";
const char shell_help_summary_test[] = "Test command";

void shell_test(SHELL_CONTEXT *ctx, int argc, char **argv )
{
}

/***********************************************************************
 * CMD: vu
 **********************************************************************/
const char shell_help_vu[] = "[domain a2b|system]\n";
const char shell_help_summary_vu[] = "Show VU meters";

#include <math.h>
#include "vu_audio.h"

#define VU_UPDATE_DELAY_MS    75

void shell_vu(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    unsigned idx, channels;
    SYSTEM_AUDIO_TYPE vu[VU_MAX_CHANNELS];
    SYSTEM_AUDIO_TYPE value;
    unsigned vu_pix[VU_MAX_CHANNELS];
    double db;
    unsigned lines, cols;
    unsigned vu_lines, offset;
    unsigned l;
    int c;
    char *screen[2] = { NULL, NULL };
    char p;
    unsigned screenIdx = 0;
    unsigned screenSize;
    unsigned screenPix;
    bool split_screen;
    unsigned ch;
    unsigned line, col;

    char *RESET_MODE = "\x1B[0m";
    char *BLACK = "\x1B[30;40m";

    /* Black background with block char, 1 space between channels */
    char *px = "\xDC";
    char *RED = "\x1B[31;40m";
    char *YELLOW = "\x1B[33;40m";
    char *GREEN = "\x1B[32;40m";

    if (argc > 1) {
        if (strcmp(argv[1], "domain") == 0) {
            if (argc > 2) {
                if (strcmp(argv[2], "a2b") == 0) {
                    clock_domain_set(context, CLOCK_DOMAIN_A2B, CLOCK_DOMAIN_BITM_VU_IN);
                } else if (strcmp(argv[2], "system") == 0) {
                    clock_domain_set(context, CLOCK_DOMAIN_SYSTEM, CLOCK_DOMAIN_BITM_VU_IN);
                } else {
                    printf("Bad domain\n");
                }
                return;
            } else {
                printf("No domain\n");
            }
            return;
        }
    }

    channels = SYSTEM_MAX_CHANNELS;
    if (argc > 1) {
        channels = strtol(argv[1], NULL, 0);
        if ((channels < 1) || (channels > VU_MAX_CHANNELS)) {
            printf("Bad channels\n");
            return;
        }
    }
    channels = getVU(context, NULL, channels);

    char *color = NULL;
    char *old_color = NULL;

    /* Get screen info */
    lines = term_get_lines(&ctx->t);
    cols = term_get_cols(&ctx->t);

    /* Make screen buffers */
    screenSize = lines * cols;
    screen[0] = SHELL_MALLOC(screenSize);
    memset(screen[0], 'B', screenSize);
    screen[1] = SHELL_MALLOC(screenSize);
    memset(screen[1], 'B', screenSize);

    /* Calculate some values */
    if (channels > 32) {
        split_screen = true;
        vu_lines = lines / 2 - 2;
    } else {
        split_screen = false;
        vu_lines = lines;
    }

    /* Clear the screen */
    term_clrscr(&ctx->t);

    do {
        /* Get VU linear values */
        getVU(context, vu, channels);

        /* Convert to db then to screen height */
        for (idx = 0; idx < channels; idx++) {
            value = vu[idx];
            if (value > 0) {
                db = 20.0 * log10((double)value / 2147483647.0);
                /* Scale for VU meter -70dB = 0, 0dB = 'lines' */
                db = ((double)vu_lines / 70.0) * db + (double)vu_lines;
                /* Round up to show full scale */
                db += 0.5;
                /* Always show something if a signal is present */
                if ((db < 1.0) && value) {
                    db = 1.0;
                }
            } else {
                db = 0.0;
            }
            vu_pix[idx] = (unsigned)db;
            if (vu_pix[idx] > vu_lines) {
                vu_pix[idx] = vu_lines;
            }
            if (vu_pix[idx] < 1) {
                vu_pix[idx] = 1;
            }
        }

        /* Render up to first 32 channels into screen buffer */
        ch = (split_screen) ? 32 : channels;
        offset = (cols - ch * 2) / 2;
        for (idx = 0; idx < ch; idx++) {
            for (l = 0; l < vu_lines; l++) {
                if (l < vu_pix[idx]) {
                    if (l > ((vu_lines * 2) / 3)) {
                        p = 'R';
                    } else if (l > (vu_lines / 3)) {
                        p = 'Y';
                    } else {
                        p = 'G';
                    }
                } else {
                    p = 'B';
                }
                line = (lines - 1) - l;
                col = offset + idx * 2;
                screenPix = line * cols + col;
                screen[screenIdx][screenPix] = p;
            }
        }

        /* Render next 32 channels into screen buffer */
        if (split_screen) {
            ch = channels - 32;
            offset = (cols - ch * 2) / 2;
            for (idx = 0; idx < ch; idx++) {
                for (l = 0; l < vu_lines; l++) {
                    if (l < vu_pix[idx+32]) {
                        if (l > ((vu_lines * 2) / 3)) {
                            p = 'R';
                        } else if (l > (vu_lines / 3)) {
                            p = 'Y';
                        } else {
                            p = 'G';
                        }
                    } else {
                        p = 'B';
                    }
                    line = ((lines / 2) - 1) - l;
                    col = offset + idx * 2;
                    screenPix = line * cols + col;
                    screen[screenIdx][screenPix] = p;
                }
            }
        }

        /* Draw only differences */
        for (line = 0; line < lines; line++) {
            for (col = 0; col < cols; col++) {
                screenPix = line * cols + col;
                if (screen[0][screenPix] != screen[1][screenPix]) {
                    p = screen[screenIdx][screenPix];
                    switch (p) {
                        case 'R':
                            color = RED;
                            break;
                        case 'Y':
                            color = YELLOW;
                            break;
                        case 'G':
                            color = GREEN;
                            break;
                        case 'B':
                            color = BLACK;
                            break;
                        default:
                            color = BLACK;
                            break;
                    }
                    term_gotoxy(&ctx->t, col + 1, line + 1);
                    if (color != old_color) {
                        term_putstr(&ctx->t, color, strlen(color));
                        color = old_color;
                    }
                    term_putstr(&ctx->t, px, 1);
                }
            }
        }

        /* Toggle screens */
        screenIdx = (screenIdx + 1) & 1;

        /* Park the cursor */
        term_gotoxy(&ctx->t, cols, lines);

        /* Check for character */
        c = term_getch(&ctx->t, TERM_INPUT_DONT_WAIT);
        if (c < 0) {
            vTaskDelay(pdMS_TO_TICKS(VU_UPDATE_DELAY_MS));
        }

    } while (c < 0);

    /* Reset the character mode */
    term_putstr(&ctx->t, RESET_MODE, strlen(RESET_MODE));

    /* Clear the screen */
    term_clrscr(&ctx->t);

    /* Go back home */
    term_gotoxy(&ctx->t, 0, 0);

    /* Free screen buffers */
    if (screen[0]) {
        SHELL_FREE(screen[0]);
    }
    if (screen[1]) {
        SHELL_FREE(screen[1]);
    }
}

/***********************************************************************
 * CMD: adc
 **********************************************************************/
const char shell_help_adc[] = "[enable|disable]\n";
const char shell_help_summary_adc[] = "Enable or disable the carrier board ADC inputs";

#include "ss_init.h"

void shell_adc(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    bool jack;
    bool ok;

    /* No ADC switch on Rev D SOMCRR boards */
    if (context->SoMCRRVersion == SOMCRR_REV_D) {
        printf("No ADC switch on Rev D SOMCRR boards\n");
        return;
    }

    if (argc > 1) {
        if (strcmp(argv[1], "enable") == 0) {
            ss_set(context, SS_PIN_ID_AUDIO_JACK_SEL, 1);
        } else if (strcmp(argv[1], "disable") == 0) {
            ss_set(context, SS_PIN_ID_AUDIO_JACK_SEL, 0);
        } else {
            printf("Invalid selection\n");
        }
    } else {
        ok = ss_get(context, SS_PIN_ID_AUDIO_JACK_SEL, &jack);
        if (ok) {
            printf("ADC Inputs %s\n", jack ? "enabled" : "disabled");
        } else {
            printf("Error!\n");
        }
    }
}

/***********************************************************************
 * CMD: reset
 **********************************************************************/
#include "init.h"

const char shell_help_reset[] = "\n";
const char shell_help_summary_reset[] = "Resets the system";

void shell_reset(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    printf("Not implemented.\n");
}

/***********************************************************************
 * CMD: route
 **********************************************************************/
const char shell_help_route[] =
    "[ <idx> <src> <src offset> <dst> <dst offset> <channels> [attenuation] [mix|set] ]\n"
    "  idx         - Routing index\n"
    "  src         - Source stream\n"
    "  src offset  - Source stream offset\n"
    "  dst         - Destination stream\n"
    "  dst offset  - Destination stream offset\n"
    "  channels    - Number of channels\n"
    "  attenuation - Source attenuation in dB (0dB default)\n"
    "  mix         - Mix or set source into destination (set default)\n"
    " Valid Streams\n"
    "  a2b        - A2B Audio\n"
    "  codec      - Analog TRS line in/out\n"
    "  spdif      - Optical SPDIF in/out\n"
    "  vu         - VU Meter sink\n"
    "  off        - Turn off the stream\n"
    " No arguments\n"
    "  Show routing table\n"
    " Single 'clear' argument\n"
    "  Clear routing table\n";
const char shell_help_summary_route[] = "Configures the audio routing table";

#include "route.h"

static char *stream2str(int streamID)
{
    char *str = "NONE";

    switch (streamID) {
        case STREAM_ID_UNKNOWN:
            str = "NONE";
            break;
        case STREAM_ID_CODEC_IN:
            str = "CODEC_IN";
            break;
        case STREAM_ID_CODEC_OUT:
            str = "CODEC_OUT";
            break;
        case STREAM_ID_SPDIF_IN:
            str = "SPDIF_IN";
            break;
        case STREAM_ID_SPDIF_OUT:
            str = "SPDIF_OUT";
            break;
        case STREAM_ID_A2B_IN:
            str = "A2B_IN";
            break;
        case STREAM_ID_A2B_OUT:
            str = "A2B_OUT";
            break;
        case STREAM_ID_VU_IN:
            str = "VU_IN";
            break;
        default:
            str = "UNKNOWN";
            break;
    }

    return(str);
}

STREAM_ID str2stream(char *stream, bool src)
{
    if (strcmp(stream, "codec") == 0) {
        return(src ? STREAM_ID_CODEC_IN : STREAM_ID_CODEC_OUT);
    } else if (strcmp(stream, "spdif") == 0) {
        return(src ? STREAM_ID_SPDIF_IN : STREAM_ID_SPDIF_OUT);
    } else if (strcmp(stream, "a2b") == 0) {
        return(src ? STREAM_ID_A2B_IN : STREAM_ID_A2B_OUT);
    } else if (strcmp(stream, "vu") == 0) {
        return(src ? STREAM_ID_MAX : STREAM_ID_VU_IN);
    } else if (strcmp(stream, "off") == 0) {
        return(STREAM_ID_UNKNOWN);
    }

    return(STREAM_ID_MAX);
}

void shell_route(SHELL_CONTEXT *ctx, int argc, char **argv)
{
    ROUTE_INFO *route;
    unsigned idx, srcOffset, sinkOffset, channels, attenuation, mix;
    STREAM_ID srcID, sinkID;
    unsigned i;

    if (argc == 1) {
        printf("Audio Routing\n");
        for (i = 0; i < MAX_AUDIO_ROUTES; i++) {
            route = context->routingTable + i;
            printf(" [%02d]: %s[%u] -> %s[%u], CHANNELS: %u, %s%udB, %s\n",
                i,
                stream2str(route->srcID), route->srcOffset,
                stream2str(route->sinkID), route->sinkOffset,
                route->channels,
                route->attenuation == 0 ? "" : "-",
                (unsigned)route->attenuation, route->mix ? "mix" : "set"
            );
        }
        return;
    } else if (argc == 2) {
        if (strcmp(argv[1], "clear") == 0) {
            for (i = 0; i < MAX_AUDIO_ROUTES; i++) {
                route = context->routingTable + i;
                /* Configure the route atomically in a critical section */
                taskENTER_CRITICAL();
                route->srcID = STREAM_ID_UNKNOWN;
                route->srcOffset = 0;
                route->sinkID = STREAM_ID_UNKNOWN;
                route->sinkOffset = 0;
                route->channels = 0;
                route->attenuation = 0;
                route->mix = 0;
                taskEXIT_CRITICAL();
            }
        }
    }

    /* Confirm a valid route index */
    idx = atoi(argv[1]);
    if (idx > MAX_AUDIO_ROUTES) {
        printf("Invalid idx\n");
        return;
    }
    route = context->routingTable + idx;
    srcID = route->srcID;
    srcOffset = route->srcOffset;
    sinkID = route->sinkID;
    sinkOffset = route->sinkOffset;
    channels = route->channels;
    mix = route->mix;

    /* Gather the source info */
    if (argc >= 3) {
        srcID = str2stream(argv[2], true);
        if (srcID == STREAM_ID_MAX) {
            printf("Invalid src\n");
            return;
        }
    }
    if (argc >= 4) {
        srcOffset = atoi(argv[3]);
    }

    /* Gather the sink info */
    if (argc >= 5) {
        sinkID = str2stream(argv[4], false);
        if (sinkID == STREAM_ID_MAX) {
            printf("Invalid sink\n");
            return;
        }
    }
    if (argc >= 6) {
        sinkOffset = atoi(argv[5]);
    }

    /* Get the number of channels */
    if (argc >= 7) {
        channels = atoi(argv[6]);
        if (channels > SYSTEM_MAX_CHANNELS) {
            printf("Invalid channels\n");
            return;
        }
    }

    /* Get the attenuation */
    if (argc >= 8) {
        attenuation = abs(atoi(argv[7]));
        if (attenuation > 120) {
            attenuation = 120;
        }
    } else {
        attenuation = 0;
    }

    /* Get the 'mix' arg */
    if (argc >= 9) {
        if (strcmp(argv[8], "mix") == 0) {
            mix = 1;
        } else {
            mix = 0;
        }
    } else {
        mix = 0;
    }

    /* Configure the route atomically in a critical section */
    taskENTER_CRITICAL();
    route->srcID = srcID;
    route->srcOffset = srcOffset;
    route->sinkID = sinkID;
    route->sinkOffset = sinkOffset;
    route->channels = channels;
    route->attenuation = attenuation;
    route->mix = mix;
    taskEXIT_CRITICAL();
}

/***********************************************************************
 * CMD: cmp (file compare)
 **********************************************************************/
const char shell_help_cmp[] = "<file1> <file2>\n";
const char shell_help_summary_cmp[] = "Compare <file1> to <file2>";

#define SHELL_CMP_PROGRESS_DOT  10000

void shell_cmp( SHELL_CONTEXT *ctx, int argc, char **argv )
{
   FILE *fp1 = NULL, *fp2 = NULL;
   unsigned long bytecount = 0;
   unsigned int printcount = 0;
   unsigned char byte1, byte2;
   bool match = false;
   bool eof1 = false;
   bool eof2 = false;

   if( argc != 3 ) {
      printf( "Usage: cmp <file1> <file2>\n" );
      return;
   }

   if( ( fp1 = fopen( argv[ 1 ], "r" ) ) == NULL ) {
      printf( "Unable to open %s\n", argv[ 1 ] );
   } else {
        if( ( fp2 = fopen( argv[ 2 ], "r" ) ) == NULL ) {
             printf( "Unable to open %s\n", argv[ 2 ] );
        } else {
            /* file compare */
            while (1) {
                byte1 = fgetc(fp1);
                if (feof(fp1)) {
                    eof1 = true;
                }
                byte2 = fgetc(fp2);
                if (feof(fp2)) {
                    eof2 = true;
                }
                if (byte1 == byte2) {
                    match = true;
                    bytecount++;
                    printcount++;
                    if (printcount > SHELL_CMP_PROGRESS_DOT) {
                        printf(".");  /* only updates screen when buffer fills */
                        printcount = 0;
                    }
                } else {
                    fseek(fp1, -1, SEEK_CUR);
                    match = false;
                    break;
                }
                if (eof1 || eof2) {
                    break;
                }
            }
            if (match) {
                printf("\nFiles match (%ld bytes)\n", bytecount);
            } else {
                printf("\nFiles don't match, first mismatch at byte: %ld (0x%02X : 0x%02X)\n", ftell(fp1), byte1, byte2);
            }
            if (eof1) {
                printf("EOF: %s\n", argv[1]);
            }
            if (eof2) {
                printf("EOF: %s\n", argv[2]);
            }
        }
    }

    if( fp1 ) {
        fclose( fp1 );
    }
    if( fp2 ) {
        fclose( fp2 );
    }
}

/***********************************************************************
 * CMD: a2b
 **********************************************************************/
const char shell_help_a2b[] = "[cmd]\n"
  "  mode [main|sub] - Bus mode 'main node' or 'sub node'\n"
  "                    (default 'main')\n"
  "  No arguments, show current bus mode\n";

const char shell_help_summary_a2b[] = "Set A2B Parameters";

void shell_a2b(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    A2B_BUS_MODE mode;
    bool ok;

    if (argc == 1) {
        if (context->a2bPresent) {
            printf("A2B (J10): I2C Addr: 0x%02X, Type: AD24%ux, Mode: %s\n",
                context->cfg.a2bI2CAddr,
                (unsigned)context->a2bxcvr,
                context->a2bmode == A2B_BUS_MODE_MAIN ? "Main" : "Sub");
        } else {
            printf("A2B: Not present\n");
        }
        return;
    }

    if (argc >= 2) {
        if (strcmp(argv[1], "mode") == 0) {
            mode = A2B_BUS_MODE_MAIN;
            if (argc >= 3) {
                if (strcmp(argv[2], "sub") == 0) {
                    mode = A2B_BUS_MODE_SUB;
                } else {
                    mode = A2B_BUS_MODE_MAIN;
                }
            }
            printf("Setting A2B (J10) mode to %s\n",
                mode == A2B_BUS_MODE_MAIN ? "main" : "sub");
            ok = a2b_set_mode(context, mode);
            if (!ok) {
                printf("Error setting A2B mode!\n");
            }
        } else {
            printf("Invalid subcommand\n");
        }
    }
}

/***********************************************************************
 * CMD: delay
 **********************************************************************/
const char shell_help_delay[] = "<ms>\n";
const char shell_help_summary_delay[] = "Delay";

#include "util.h"

void shell_delay(SHELL_CONTEXT *ctx, int argc, char **argv )
{
    unsigned delaymS = 0;

    if (argc > 1) {
        delaymS = strtoul(argv[1], NULL, 0);
    }

    if (delaymS > 0) {
        delay(delaymS);
    }
}

