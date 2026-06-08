/*
 * Copyright (c) 2001-2019, Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause OR Arm’s non-OSI source license
 */

/*
 * Ambiq Apollo510 modification notes
 * ----------------------------------
 * This is ARM's CryptoCell-312 runtime ChaCha low-level driver, patched for
 * the Apollo510 (Cortex-M55) which places a write-back/write-allocate data
 * cache (DCACHE) in front of the buffers the CC312 symmetric DMA engine reads
 * from and writes to. Two constraints must be honored, identical to the ones
 * already applied to the AES driver (am_hal_cc312_aes.c::am_hal_cc312_aes_process):
 *
 *   1. Any buffer transferred to/from the engine by DMA must be aligned to the
 *      DCACHE line size (AM_HAL_CC312_DMA_ALIGNMENT == 32 bytes), and its
 *      backing allocation padded to a multiple of that size. Cache maintenance
 *      operates on whole cache lines; if a DMA buffer shares a cache line with
 *      unrelated data, the clean/invalidate below will corrupt that neighbor.
 *
 *   2. Before the DMA is kicked, the input buffer must be CLEANED (so pending
 *      CPU writes reach memory for the engine to read) and the output buffer
 *      must be CLEANED+INVALIDATED (so no stale dirty line later evicts on top
 *      of the engine's result). After the DMA completes, the output buffer must
 *      be INVALIDATED so the CPU re-reads the engine's result from memory
 *      rather than a stale cached copy. A dummy DLLI read flush is performed
 *      first to guarantee the engine's last write has drained to memory.
 *
 * Only the internal-DMA ChaCha path (ProcessChacha) needs this. SetDataBuffersInfo
 * (driver_common.c) is shared by all engines and only records buffer addresses,
 * and the chacha_driver_ext_dma.c path delegates DMA and coherency to the host
 * application, so neither is modified here.
 */

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include "cc_pal_mutex.h"
#include "cc_pal_abort.h"
#include "chacha_driver.h"
#include "driver_defs.h"
#include "cc_hal.h"
#include "cc_hal_plat.h"
#include "cc_sram_map.h"
#include "cc_regs.h"
#include "dx_crys_kernel.h"
#include "cc_util_pm.h"

/*
 * Apollo510 DCACHE / CC312 DMA coherency helpers.
 *
 * These are provided by the Ambiq HAL (am_hal_cc312.c). They are declared here
 * (rather than pulling in the full am_mcu_apollo.h) to keep this ARM runtime
 * source self-contained and minimally coupled to the HAL headers. The same
 * helpers back the AES driver's DMA path.
 */
#ifndef AM_HAL_CC312_DMA_ALIGNMENT
#define AM_HAL_CC312_DMA_ALIGNMENT  32U  /* Apollo510 DCACHE line size. */
#endif

extern void     am_hal_cc312_cache_clean_invalidate_region(const void *addr, uint32_t length);
extern void     am_hal_cc312_cache_invalidate_region(const void *addr, uint32_t length);
extern uint32_t am_hal_cc312_flush_dummy_dlli(uint32_t src_addr, uint32_t length);


extern CC_PalMutex CCSymCryptoMutex;

/* chacha mode, poly1305 disabled, 256 bit key, 20 rounds, 64 bit iv, do not reset the block counter (overwritten by the context) */
#define CHACHA_CONTROL_REG_VAL        (1 << DX_CHACHA_CONTROL_REG_INIT_FROM_HOST_BIT_SHIFT)
#define CHACHA_CONTROL_REG_USE_IV_96  (1 << DX_CHACHA_CONTROL_REG_USE_IV_96BIT_BIT_SHIFT)

/******************************************************************************
*               PRIVATE FUNCTIONS
******************************************************************************/

static drvError_t InitChacha(ChachaContext_t *chachaCtx)
{
    uint32_t irrVal = 0;

    /* verify user context pointer */
    if ( chachaCtx == NULL ) {
        return CHACHA_DRV_INVALID_USER_CONTEXT_POINTER_ERROR;
    }

    /* verify chacha valid input addr type */
    if ( (chachaCtx->inputDataAddrType != SRAM_ADDR) &&
         (chachaCtx->inputDataAddrType != DLLI_ADDR) ) {
        return CHACHA_DRV_ILLEGAL_INPUT_ADDR_MEM_ERROR;
    }

    /* verify chacha valid output addr type */
    if ( (chachaCtx->outputDataAddrType != SRAM_ADDR) &&
         (chachaCtx->outputDataAddrType != DLLI_ADDR) ) {
        return CHACHA_DRV_ILLEGAL_OUTPUT_ADDR_MEM_ERROR;
    }

    /* make sure sym engines are ready to use */
    CC_HAL_WAIT_ON_CRYPTO_BUSY();

    /* clear all interrupts before starting the engine */
    CC_HalClearInterruptBit(0xFFFFFFFFUL);

    /* mask dma interrupts which are not required */
    irrVal = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, HOST_IMR));
    CC_REG_FLD_SET(HOST_RGF, HOST_IMR, SRAM_TO_DIN_MASK, irrVal, 1);
    CC_REG_FLD_SET(HOST_RGF, HOST_IMR, DOUT_TO_SRAM_MASK, irrVal, 1);
    CC_REG_FLD_SET(HOST_RGF, HOST_IMR, MEM_TO_DIN_MASK, irrVal, 1);
    CC_REG_FLD_SET(HOST_RGF, HOST_IMR, DOUT_TO_MEM_MASK, irrVal, 1);
    CC_REG_FLD_SET(HOST_RGF, HOST_IMR, SYM_DMA_COMPLETED_MASK, irrVal, 0);
    CC_HalMaskInterrupt(irrVal);

    /* configure DIN-CHACHA-DOUT */
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CRYPTO_CTL) ,CONFIG_DIN_CHACHA_DOUT_VAL);

    return CHACHA_DRV_OK;
}

static drvError_t LoadChachaState(ChachaContext_t *chachaCtx)
{
        /* verify user context pointer */
        if (chachaCtx == NULL) {
                return CHACHA_DRV_INVALID_USER_CONTEXT_POINTER_ERROR;
        }

        /* write the initial counter value according to mode */
        if (chachaCtx->nonceSize == NONCE_SIZE_64) {
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_MSB), chachaCtx->blockCounterMsb);
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_0), chachaCtx->nonceBuf[0]);
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_1), chachaCtx->nonceBuf[1]);
        }
        else if (chachaCtx->nonceSize == NONCE_SIZE_96) {
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_MSB), chachaCtx->nonceBuf[0]);
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_0), chachaCtx->nonceBuf[1]);
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_1), chachaCtx->nonceBuf[2]);
        }
        else {
                return CHACHA_DRV_ILLEGAL_NONCE_SIZE_ERROR;
        }
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_LSB), chachaCtx->blockCounterLsb);

        return CHACHA_DRV_OK;
}


static drvError_t StoreChachaState(ChachaContext_t *chachaCtx)
{
        /* verify user context pointer */
        if (chachaCtx == NULL) {
                return CHACHA_DRV_INVALID_USER_CONTEXT_POINTER_ERROR;
        }

        /* read the initial counter value according to mode */
        if (chachaCtx->nonceSize == NONCE_SIZE_64) {
                chachaCtx->blockCounterMsb = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_MSB));
                chachaCtx->nonceBuf[0] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_0));
                chachaCtx->nonceBuf[1] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_1));
        }
        else if (chachaCtx->nonceSize == NONCE_SIZE_96) {
                chachaCtx->nonceBuf[0] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_MSB));
                chachaCtx->nonceBuf[1] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_0));
                chachaCtx->nonceBuf[2] = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_IV_1));
        }
        else {
                return CHACHA_DRV_ILLEGAL_NONCE_SIZE_ERROR;
        }
        chachaCtx->blockCounterLsb = CC_HAL_READ_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_BLOCK_CNT_LSB));

        return CHACHA_DRV_OK;
}

static drvError_t LoadChachaKey(ChachaContext_t *chachaCtx)
{
        int enrtyNum = 0;

        /* verify user context pointer */
        if (chachaCtx == NULL) {
                return CHACHA_DRV_INVALID_USER_CONTEXT_POINTER_ERROR;
        }

        for (enrtyNum = 0; enrtyNum < CHACHA_256_BIT_KEY_SIZE_WORDS; ++enrtyNum) {
                CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_KEY0) + (sizeof(uint32_t) * enrtyNum), chachaCtx->keyBuf[enrtyNum]);
        }

        return CHACHA_DRV_OK;
}

/******************************************************************************
*               PUBLIC FUNCTIONS
******************************************************************************/
drvError_t ProcessChacha(ChachaContext_t *chachaCtx, CCBuffInfo_t *pInputBuffInfo, CCBuffInfo_t *pOutputBuffInfo, uint32_t inDataSize)
{
    uint32_t irrVal = 0;
    uint32_t chachaCtrl = CHACHA_CONTROL_REG_VAL;
    drvError_t drvRc = CHACHA_DRV_OK;
    uint32_t regVal = 0;
    uint32_t inputDataAddr, outputDataAddr;

    /* check input parameters */
    if ( (pInputBuffInfo == NULL) || (pOutputBuffInfo == NULL)) {
         return CHACHA_DRV_INVALID_USER_DATA_BUFF_POINTER_ERROR;
    }

    /* verify user context pointer */
    if ( chachaCtx == NULL ) {
        return CHACHA_DRV_INVALID_USER_CONTEXT_POINTER_ERROR;
    }
    if (((chachaCtx->inputDataAddrType == SRAM_ADDR) && (inDataSize >= CC_SRAM_MAX_SIZE)) ||
        ((chachaCtx->inputDataAddrType == DLLI_ADDR) && (inDataSize >= DLLI_MAX_BUFF_SIZE))) {
        return CHACHA_DRV_ILLEGAL_MEM_SIZE_ERROR;
    }

    /* lock mutex for more chacha hw operation */
    drvRc = CC_PalMutexLock(&CCSymCryptoMutex, CC_INFINITE);
    if (drvRc != 0) {
        CC_PalAbort("Fail to acquire mutex\n");
    }

    /* increase CC counter at the beginning of each operation */
    drvRc = CC_IS_WAKE;
    if (drvRc != 0) {
        CC_PalAbort("Fail to increase PM counter\n");
    }

    /* enable clock */
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_CLK_ENABLE) ,SET_CLOCK_ENABLE);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DMA_CLK_ENABLE) ,SET_CLOCK_ENABLE);

    drvRc = InitChacha(chachaCtx);
    if (drvRc != CHACHA_DRV_OK) {
            goto ProcessExit;
    }

    /* write the initial counter value */
    drvRc = LoadChachaState(chachaCtx);
    if (drvRc != CHACHA_DRV_OK) {
            goto ProcessExit;
    }

    /* load key */
    drvRc = LoadChachaKey(chachaCtx);
    if (drvRc != CHACHA_DRV_OK) {
            goto ProcessExit;
    }

    /* configure the CHACHA mode */
    if (chachaCtx->nonceSize == NONCE_SIZE_96) {
        chachaCtrl |= CHACHA_CONTROL_REG_USE_IV_96;
    }

    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_CONTROL_REG), chachaCtrl);

    inputDataAddr = pInputBuffInfo->dataBuffAddr;
    outputDataAddr = pOutputBuffInfo->dataBuffAddr;

    /* configure the HW with the correct data buffer attributes (secure/non-secure) */
    CC_REG_FLD_SET(HOST_RGF, AHBM_HNONSEC, AHB_READ_HNONSEC, regVal, pInputBuffInfo->dataBuffNs);
    CC_REG_FLD_SET(HOST_RGF, AHBM_HNONSEC, AHB_WRITE_HNONSEC, regVal, pOutputBuffInfo->dataBuffNs);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, AHBM_HNONSEC) ,regVal);

    /* configure destination address and size */
    /* and set dout bit in irr */
    if (chachaCtx->outputDataAddrType == DLLI_ADDR) {
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DST_LLI_WORD0) ,outputDataAddr);
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DST_LLI_WORD1) ,inDataSize);
    } else {
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, SRAM_DEST_ADDR) ,outputDataAddr);
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DOUT_SRAM_BYTES_LEN) ,inDataSize);
    }

    /*
     * Apollo510 DCACHE coherency (concern #2: clean & invalidate before DMA).
     *
     * Done after the destination is programmed but before the source write
     * below kicks the transfer, matching am_hal_cc312_aes_process(). Only DLLI
     * (system-memory) buffers live behind the CPU DCACHE; SRAM_ADDR buffers are
     * in CC-internal SRAM and must not be touched by CPU cache maintenance.
     *
     * The buffers MUST be 32-byte (AM_HAL_CC312_DMA_ALIGNMENT) aligned and
     * padded (concern #1) - the cache helpers round the length up to a whole
     * cache line, so an unaligned/short buffer would clobber its neighbors.
     */
    if (chachaCtx->inputDataAddrType == DLLI_ADDR) {
        am_hal_cc312_cache_clean_invalidate_region((const void *)(uintptr_t)inputDataAddr, inDataSize);
    }
    if (chachaCtx->outputDataAddrType == DLLI_ADDR) {
        am_hal_cc312_cache_clean_invalidate_region((const void *)(uintptr_t)outputDataAddr, inDataSize);
    }

    /* configure source address and size */
    if (chachaCtx->inputDataAddrType == DLLI_ADDR) {
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, SRC_LLI_WORD0) ,inputDataAddr);
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, SRC_LLI_WORD1) ,inDataSize);
    } else {
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, SRAM_SRC_ADDR) ,inputDataAddr);
        CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DIN_SRAM_BYTES_LEN) ,inDataSize);
    }

    /* set dma completion bit in irr */
    CC_REG_FLD_SET(HOST_RGF, HOST_IRR, SYM_DMA_COMPLETED, irrVal, 1);
    drvRc = CC_HalWaitInterrupt(irrVal);
    if (drvRc != CHACHA_DRV_OK) {
        goto ProcessExit;
    }
    /* get machine state */
    drvRc = StoreChachaState(chachaCtx);
    if (drvRc != CHACHA_DRV_OK) {
            goto ProcessExit;
    }

    /*
     * Apollo510 DCACHE coherency (concern #2: invalidate output after DMA).
     *
     * Flush the engine's last DLLI write to memory with a dummy DLLI read, then
     * invalidate the output region so the CPU re-reads the engine's result
     * instead of a stale cached copy. Mirrors am_hal_cc312_aes_process().
     */
    if (chachaCtx->outputDataAddrType == DLLI_ADDR) {
        if (am_hal_cc312_flush_dummy_dlli(outputDataAddr, inDataSize) != 0U) {
            drvRc = CHACHA_DRV_ILLEGAL_OUTPUT_ADDR_MEM_ERROR;
            goto ProcessExit;
        }
        am_hal_cc312_cache_invalidate_region((const void *)(uintptr_t)outputDataAddr, inDataSize);
    }

    ProcessExit:
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, CHACHA_CLK_ENABLE) ,SET_CLOCK_DISABLE);
    CC_HAL_WRITE_REGISTER(CC_REG_OFFSET(HOST_RGF, DMA_CLK_ENABLE) ,SET_CLOCK_DISABLE);

    /* decrease CC counter at the end of each operation */
    if (CC_IS_IDLE != 0) {
        CC_PalAbort("Fail to decrease PM counter\n");
    }

    /* release mutex */
    if (CC_PalMutexUnlock(&CCSymCryptoMutex) != 0) {
        CC_PalAbort("Fail to release mutex\n");
    }

    return drvRc;
}


