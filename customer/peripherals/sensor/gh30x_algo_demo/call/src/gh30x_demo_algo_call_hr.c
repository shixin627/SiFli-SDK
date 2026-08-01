/**
 * @copyright (c) 2003 - 2024, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh30x_demo_algo_call_hr.c
 *
 * @brief   gh30x algorithm hr interface
 *
 * @version ref gh30x_example_common.h
 *
 */

#include <stdio.h>
#include "gh30x_demo_algo_call.h"
#include "gh30x_demo_algorithm_calc.h"
#include "gh30x_demo_algo_config.h"
#include "goodix_hba.h"

#if (__USE_GOODIX_HR_ALGORITHM__)

/* Sleep-context knobs (scene/mode/senseless cadence + per-frame sleep_flg) are
   set via the HBD_* APIs in gh30x_example_hook.c, driven by
   hr_service_set_sleep_active() on the sleep edge. Their legacy gh3011_algo_*
   hook readers are dead code (no callers); THIS file is the live algo path, so
   the knobs must be consumed here. gsthbdCfg requires
   __HBD_ALGORITHM_EXTERNANL_CONFIG_ENABLE__=1 (gh30x_example_config.h). */
extern goodix_hba_config gsthbdCfg;
extern GU8 HBD_HbaGetSleepFlag(void);

goodix_hba_ret goodix_hba_init_func(GU32 fs)
{
    goodix_hba_config stHbCfg;
    goodix_hba_ret stAlgoRet = GX_ALGO_HBA_SUCCESS;

    GCHAR uchHrVersion[100] = {0};
    goodix_hba_version((uint8_t *)uchHrVersion);

    GH30X_ALGO_LOG_PARAM("hba algorithm version : %s\r\n", uchHrVersion);

    memset(&stHbCfg, 0, sizeof(goodix_hba_config));
    /* Each bg_hr burst cold-starts the algo through here, so reading the
       external config at init is enough for the next burst to pick up a sleep
       edge. gsthbdCfg's static initializer is the awake trio
       (DYNAMIC/DEFAULT/0,0) — behaviour is unchanged until hr_service flips it. */
    stHbCfg.mode = gsthbdCfg.mode;
    stHbCfg.scence = gsthbdCfg.scence;
    stHbCfg.senseless_mode_step = gsthbdCfg.senseless_mode_step;
    stHbCfg.senseless_mode_duration = gsthbdCfg.senseless_mode_duration;
    stHbCfg.fs = fs;
    stHbCfg.valid_channel_num = 1;
    /* 2026-08-01: was 0 for this field's ENTIRE git history (vendor import default),
       but goodix_hba.h:99 documents the range as "默认30s,最大120s,最短30s" -- 0 is
       BELOW the vendor's own minimum. ADR 0016 concluded "this .lib does not implement
       confidence" from a measurement taken with back_track_len = 0; since backtrack is
       the retrospective window a confidence/SNR statistic would be computed over, that
       conclusion was drawn under an out-of-spec config and has never been re-tested at
       a legal value. 30 is the documented floor. Safe for the burst length: the old
       "longer window starved the burst" note was against a 40 s awake burst; both awake
       and sleep bursts are 180 s now (BG_HR_BURST_MS_*). If the awake burst is ever
       shortened back toward 40 s, re-check this. */
    stHbCfg.back_track_len = 30;
    stHbCfg.hba_latest_output_time = 0;
    stHbCfg.hba_earliest_output_time = 0;
    /* Still-HR stability tune (2026-06-05): withhold low-confidence HR so the algo
       stops emitting the 100<->40 garbage jumps seen while sitting still. The FIELD
       is official (goodix_hba.h:102) but ADR 0016 measured this .lib IGNORING it --
       valid_score stayed 0 and values below the threshold were emitted anyway. Left in
       place because the back_track_len fix above may be what revives it; if the bench
       re-test still shows qmax=0, this line is dead weight and the gate has to be built
       from our own history instead (ADR 0016 path B). */
    stHbCfg.hba_lowerest_confidence = 30;

    GH30X_ALGO_LOG_PARAM("[%s]:params = %d,%d,%d,%d,%d,%d,%d,\r\n", __FUNCTION__,
                         stHbCfg.mode,
                         stHbCfg.scence,
                         stHbCfg.fs,
                         stHbCfg.valid_channel_num,
                         stHbCfg.back_track_len,
                         stHbCfg.hba_latest_output_time,
                         stHbCfg.hba_earliest_output_time);

    stAlgoRet = goodix_hba_init(&stHbCfg);

    return stAlgoRet;
}

goodix_hba_ret goodix_hba_deinit_func(void)
{
    goodix_hba_deinit();
    return GX_ALGO_HBA_SUCCESS;
}

GS8 GH30xHrAlgoInit(const STGh30xFrameInfo *const pstFrameInfo)
{
    GS8 chRet = GH30X_RET_GENERIC_ERROR;

    GH30X_ALGO_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (goodix_hba_init_func(pstFrameInfo->pstFunctionInfo->usOutputDataRate) == GX_ALGO_HBA_SUCCESS)
    {
        chRet = GH30X_RET_OK;
        GH30X_ALGO_LOG_PARAM("[%s]GH30X_HbaInit success!!\r\n", __FUNCTION__);
    }
    else
    {
        GH30X_ALGO_LOG_PARAM("hba init error!\r\n");
    }
    pstFrameInfo->pstAlgoResult->uchUpdateFlag = 0;
    pstFrameInfo->pstAlgoResult->snResult[0] = 0;

    return chRet;
}

GS8 GH30xHrAlgoDeinit(const STGh30xFrameInfo *const pstFrameInfo)
{
    GS8 chRet = GH30X_RET_GENERIC_ERROR;
    GH30X_ALGO_LOG_PARAM("%s\r\n", __FUNCTION__);

    if (goodix_hba_deinit_func() == GX_ALGO_HBA_SUCCESS)
    {
        chRet = GH30X_RET_OK;
    }
    else
    {
        GH30X_ALGO_LOG_PARAM("hba deinit error!\r\n");
    }
    return chRet;
}

// call algo 2-level interface

GS8 GH30xHrAlgoExe(const STGh30xFrameInfo *const pstFrameInfo)
{
    GS8 chAlgoRet = 0;
    if (0 == pstFrameInfo)
    {
        return GH30X_RET_GENERIC_ERROR;
    }

    goodix_hba_input_rawdata stRawdata = {0};
    goodix_hba_result stResult = {0};

    stRawdata.frameid = GH30X_GET_BYTE0_FROM_DWORD(*(pstFrameInfo->punFrameCnt));
    stRawdata.ppg_rawdata[0] = (pstFrameInfo->punRawdata[0] & 0x00FFFFFF);
    stRawdata.enable_flg[0] = 1;
    stRawdata.acc_x = pstFrameInfo->pusGsensordata[0];
    stRawdata.acc_y = pstFrameInfo->pusGsensordata[1];
    stRawdata.acc_z = pstFrameInfo->pusGsensordata[2];
    stRawdata.sleep_flg = HBD_HbaGetSleepFlag(); /* read every frame; set by HBD_HbaSleepFlagConfig */

#if 0
    GH30X_ALGO_LOG_PARAM("[%s]:Rawdata = %d,0x%X,%d\r\n", __FUNCTION__,
                                                                            stRawdata.frameid,
                                                                            stRawdata.ppg_rawdata[0],
                                                                            stRawdata.enable_flg[0]
                                                                            );
#endif

    /* call algorithm, update result */
    if (goodix_hba_update(&stRawdata, &stResult) == GX_ALGO_HBA_SUCCESS)
    {
        // GH30X_ALGO_LOG_PARAM("[GH30xHrAlgoExe] hba_out_flag:%d \r\n", stResult.hba_out_flag);
        if (stResult.hba_out_flag == 1)
        {
            pstFrameInfo->pstAlgoResult->uchUpdateFlag = (GU8)stResult.hba_out_flag;
            pstFrameInfo->pstAlgoResult->snResult[0] = (GS32)stResult.hba_out;
            pstFrameInfo->pstAlgoResult->snResult[1] = (GS32)stResult.valid_score;
            pstFrameInfo->pstAlgoResult->snResult[2] = GH30X_ALGORITHM_GF32_2_GS32(stResult.hba_snr);
            pstFrameInfo->pstAlgoResult->snResult[3] = (GS32)stResult.valid_level;
            pstFrameInfo->pstAlgoResult->snResult[4] = (GS32)stResult.hba_acc_info;
            pstFrameInfo->pstAlgoResult->snResult[5] = (GS32)stResult.hba_acc_scence;

            GH30X_ALGO_LOG_PARAM("[%s]hr = %d,UpdateFlag = %d\r\n", __FUNCTION__,
                                 (int)pstFrameInfo->pstAlgoResult->snResult[0], (int)pstFrameInfo->pstAlgoResult->uchUpdateFlag);
            extern void gh3018_set_hr(uint32_t hr);
            gh3018_set_hr(pstFrameInfo->pstAlgoResult->snResult[0]);
            extern void gh3018_set_hr_quality(uint32_t valid_score, uint32_t valid_level,
                                              uint32_t confi_x100, uint32_t snr_x100);
            {
                GF32 c = stResult.hba_confi;  if (c < 0) c = 0;
                GF32 s = stResult.hba_snr;    if (s < 0) s = 0;
                gh3018_set_hr_quality((uint32_t)stResult.valid_score,
                                      (uint32_t)stResult.valid_level,
                                      (uint32_t)(c * 100.0f), (uint32_t)(s * 100.0f));
            }
            pstFrameInfo->pstAlgoResult->usResultBit = 0x3F;
            pstFrameInfo->pstAlgoResult->uchResultNum = GH30x_BitCount(pstFrameInfo->pstAlgoResult->usResultBit);
#if __GH30X_HR_OUTPUT_VALUE_STRATEGY_EN__
            Gh30xHrOutputValueStrategyPro(pstFrameInfo->pstAlgoResult, pstFrameInfo->punFrameCnt[0]);
#endif
            GH30X_HrAlgorithmResultReport(pstFrameInfo->pstAlgoResult, pstFrameInfo->punFrameCnt[0]);

            pstFrameInfo->pstAlgoResult->uchUpdateFlag = (GU8)stResult.hba_out_flag;
            pstFrameInfo->pstAlgoResult->snResult[0] = (GS32)stResult.hba_out;
        }
    }
    else
    {
        chAlgoRet = GH30X_RET_RESOURCE_ERROR;
        GH30X_ALGO_LOG_PARAM("no enough ram for algorithm error!\r\n");
        GH30X_ALGO_LOG_PARAM("please feedback to GOODIX!\r\n");
    }
    return chAlgoRet;
}

#endif /* __USE_GOODIX_HR_ALGORITHM__ */
