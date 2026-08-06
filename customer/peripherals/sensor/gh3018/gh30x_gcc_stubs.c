/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * GCC-only stubs for Goodix sensor algorithm functions.
 *
 * Goodix only ships the gh3018 HR / SPO2 / HBA / NADT algorithm libraries as
 * armclang-built .lib (Default_KEIL5_M33_hard-fp_fshort.lib +
 * cortexM33_armcland-mdk531_*.lib) which can't link with arm-none-eabi-gcc.
 *
 * For GCC builds we skip those .lib in the SConscripts and provide these
 * weak stubs so the LCPU links. PPG / heart-rate / SPO2 features will be
 * non-functional in GCC builds — use the Keil toolchain for production.
 */
/* Toolchain-gating is handled in SConscript (SrcRemove for non-GCC),
   so this file is only compiled by arm-none-eabi-gcc. */

#include "gh30x_example_common.h"
#include "gh30x_algo_hook.h"

/* memory init/deinit */
/* int32_t, not GS32: GCC's int32_t is `long int` while GS32 is `int`. */
int32_t goodix_mem_init(void *mem_addr, int32_t size) { (void)mem_addr; (void)size; return 0; }
void goodix_mem_deinit(void) { }

/* HBA (heart-rate algorithm) */
goodix_hba_ret goodix_hba_version(GU8 version[100]) { (void)version; return GX_ALGO_HBA_SUCCESS; }
goodix_hba_ret goodix_hba_init(goodix_hba_config const *cfg) { (void)cfg; return GX_ALGO_HBA_SUCCESS; }
goodix_hba_ret goodix_hba_deinit(void) { return GX_ALGO_HBA_SUCCESS; }
goodix_hba_ret goodix_hba_update(goodix_hba_input_rawdata *raw, goodix_hba_result *res)
{
    (void)raw; (void)res;
    return GX_ALGO_HBA_SUCCESS;
}

/* SPO2 */
goodix_spo2_ret goodix_spo2_version(GU8 version[100]) { (void)version; return (goodix_spo2_ret)0; }
goodix_spo2_ret goodix_spo2_init(goodix_spo2_config *cfg) { (void)cfg; return (goodix_spo2_ret)0; }
goodix_spo2_ret goodix_spo2_calc(goodix_spo2_input_rawdata *raw, goodix_spo2_result *res)
{
    (void)raw; (void)res;
    return (goodix_spo2_ret)0;
}
goodix_spo2_ret goodix_spo2_deinit(void) { return (goodix_spo2_ret)0; }

/* HRV */
GU8 *goodix_hrv_version(void) { static GU8 v[1] = {0}; return v; }
goodix_hrv_ret goodix_hrv_init(goodix_hrv_config *config) { (void)config; return (goodix_hrv_ret)0; }
goodix_hrv_ret goodix_hrv_calc(goodix_hrv_input_rawdata *in, goodix_hrv_result *out)
{
    (void)in; (void)out;
    return (goodix_hrv_ret)0;
}
goodix_hrv_ret goodix_hrv_deinit(void) { return (goodix_hrv_ret)0; }

/* NADT (no-algorithm-detect) */
void NADT_ProcInit(void) { }
void NADT_ProcDeInit(void) { }
GBOOL NADT_Proc(GS32 lPacketInfoArr[], GU8 lResult[])
{
    (void)lPacketInfoArr; (void)lResult;
    return 0;
}
void NADT_Control(GS32 lOptTye, GS32 lConfigValue[]) { (void)lOptTye; (void)lConfigValue; }
GS8 *NADT_GetVersion(void) { static GS8 v[1] = {0}; return v; }

/* Misc helpers normally in the .lib */
GCHAR *Gh30xHBDVersionGet(void) { static GCHAR v[1] = {0}; return v; }
void Gh30xRawdata24BitTo32Bit(GU8 *puchRawdataFifo, GU16 usSamplePointNum, GU16 usSamplePointNumMax)
{
    (void)puchRawdataFifo; (void)usSamplePointNum; (void)usSamplePointNumMax;
}
