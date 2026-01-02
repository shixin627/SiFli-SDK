#ifndef __GH3011_ALGO_HOOK_H__
#define __GH3011_ALGO_HOOK_H__

#include "gh30x_example_common.h"
#if (__HBD_DOUBLE_CORE_ENABLE__ == 0)
#include "goodix_mem.h"
#include "goodix_type.h"
#include "goodix_hba.h"
#include "goodix_spo2.h"
#include "goodix_sys_hrv.h"
#else //(__HBD_DOUBLE_CORE_ENABLE__ == 0)
#define PPG_CHANNEL_NUM		4
#if (__HBD_HB_ALGORITHM_ENABLE__)
//hba algo typedef
typedef enum {
	HBA_SCENES_DEFAULT = 0,				// 默认：由算法内部识别处理

	HBA_SCENES_DAILY_LIFE = 1,			// 日常生活
	HBA_SCENES_RUNNING_INSIDE = 2,		// 室内跑步机
	HBA_SCENES_WALKING_INSIDE = 3,		// 室内步行
	HBA_SCENES_STAIRS = 4,				// 上下楼梯

	HBA_SCENES_RUNNING_OUTSIDE = 5,		// 室外跑步
	HBA_SCENES_WALKING_OUTSIDE = 6,		// 室外步行

	HBA_SCENES_STILL_REST = 7,			// 静息
	HBA_SCENES_REST = 8,				// 休息
	HBA_SCENES_STILLRADON = 9,			// 憋气

	HBA_SCENES_BIKING_INSIDE = 10,		//室内自行车
	HBA_SCENES_BIKING_OUTSIDE = 11,		//室外自行车
	HBA_SCENES_BIKING_MOUNTAIN= 12,		//室外自行车越野
	HBA_SCENES_RUNNING_HIGH_HR = 13,	//高心率跑步

	HBA_SCENES_RUNNING_TREADMILL_CCOMBINE= 14,		// 跑步机组合跑

	HBA_SCENES_HIGH_INTENSITY_COMBINE = 15,		// 高强度运动组合
	HBA_SCENES_TRADITIONAL_STRENGTH_COMBINE = 16,		// 传统力量训练组合
	HBA_SCENES_STEP_TEST = 17,		    // 台阶测试

	HBA_SCENES_BALL_SPORTS = 18,		// 球类运动
	HBA_SCENES_AEROBICS = 19,		    // 健身操

    HBA_SCENES_SLEEP = 20,              // 睡眠场景
    HBA_SCENES_JUMP = 21,               //手腕跳绳（LS ROMA）
    HBA_SCENES_CORDLESS_JUMP = 22,	    // 万机无绳跳绳测试
	HBA_SCENES_SWIMMING = 23,           // 游泳场景
    HBA_SCENES_SIZE = 24,               // 场景数目
}hba_scenes_e;

typedef enum {
    HBA_TEST_DYNAMIC = 0,		// 默认：动态测试
    HBA_TEST_DOT = 1,			// 点测
    HBA_TEST_SENSELESS = 2,		// 无感模式
}hba_test_mode;

typedef struct {
    hba_test_mode mode;                             // 测试模式
	hba_scenes_e scence;							// 场景
	GU32 fs;									// 采样率
	GS32 valid_channel_num;						// 有效通道数
	// 用于外部控制算法倾向性的参数
    GU32 back_track_len;                       // 回溯的时长,默认30s,最大时长限制为120s，最短时长限制为30s
	GS32 hba_latest_output_time;					// 最晚出值时间
	GS32 hba_earliest_output_time;				// 最早出值时间
	GU32 hba_lowerest_confidence;				// 最低出值置信度
	GU32 hba_out_step_second;					// 出值间隔
	GU32 hba_convergence_rate;					// 追踪过程中的收敛速率
    GU32 senseless_mode_step;                   // 无感间隔时间秒数，为0表示未知
    GU32 senseless_mode_duration;               // 无感持续时间秒数，为0表示未知
}goodix_hba_config;

typedef struct
{
	GU32 frameid;								// 帧序号
	GS32 ppg_rawdata[3 * PPG_CHANNEL_NUM];		// PPG原始数据，依次绿光1234-红外1234-红光1234
	GS32 cur_adj_flg[3 * PPG_CHANNEL_NUM];		// 电流调光标志位
	GS32 gain_adj_flg[3 * PPG_CHANNEL_NUM];		// 增益调光标志位
	GS32 enable_flg[3 * PPG_CHANNEL_NUM];		// 通道使能标志位

	GS32 acc_x;									// 加速度计x轴
	GS32 acc_y;									// 加速度计y轴
	GS32 acc_z;									// 加速度计z轴

    GU32 sleep_flg;                             //睡眠flg
}goodix_hba_input_rawdata;

typedef struct
{
	GU32 hba_out_flag;				// 出值标记：为 1 有效
	GS32 hba_out;					// 心率值
	GF32 hba_confi;				// 置信度
	GF32 hba_snr;					// 信噪比   : 滤波去除干扰后，频谱主峰能量处于总能量
    GU32 valid_level;			    // 置信等级 : 0 -> 1 -> 2 ，越大越可靠
    GU32 valid_score;				// 置信分数 : 0->100 越大越可靠
    GU32 hba_acc_info;              // 运动状态 : 0-静息，小运动；1-步行-中运动；2-跑步-大运动；
    hba_scenes_e hba_acc_scence;            // 运动场景 : 参考 hba_scenes_e 枚举类型
	// 每一路的心率值，用于调试
	GU32 hba_out_flag_channel[PPG_CHANNEL_NUM];				// 出值标记：为 1 有效
	GU32 hba_out_channel[PPG_CHANNEL_NUM];					// 心率值
	GF32 hba_confi_channel[PPG_CHANNEL_NUM];				// 置信度
	GF32 hba_snr_channel[PPG_CHANNEL_NUM];					// 信噪比
	GF32 *p_hba_td_filter[PPG_CHANNEL_NUM];				// 时域滤波结果指针数组
	GF32 *p_hba_fd_filter[PPG_CHANNEL_NUM + 1];			// 频域(维纳)滤波结果指针数组，4路加1路最优
}goodix_hba_result;

/**
* @brief HBA 算法函数返回状态值
*/
typedef enum
{
	GX_ALGO_HBA_SUCCESS = 0x00000000,			/**< 成功     */
	GX_ALGO_HBA_RWONG_INPUT = 0x00000001,		/**< 输入数据格式不合法     */
	GX_ALGO_HBA_NO_MEMORY = 0x00008000,			/**< 内存空间不够     */
}goodix_hba_ret;
#endif

#if(__HBD_SPO2_ALGORITHM_ENABLE__)
#define CHIP_PPG_CHL_NUM		                4  //芯片支持的通道数

typedef struct {
	//raw 配置信息
	GU32 valid_chl_num;			// 有效通道数
	GU32 raw_fs;			    // 原始采样率
	
	GS32 cali_coef_a;							//校准参数2次项
	GS32 cali_coef_b;							//校准参数1次项
	GS32 cali_coef_c;							//校准参数常数项
	//hb使能标志
	GU32 hb_en_flg;					//hb使能标志
	//佩戴状态
	GU32 wear_mode;					//佩戴状态 0:手环  1：手指  2：耳机
	// acc move thr
	GU32 acc_thr_max;
	GU32 acc_thr_min;
	GU32 acc_thr_scale;
	GU32 acc_thr_num;
	
	//CTR设置
	GU32 ctr_en_flg;
	GU32 ctr_red_thr;
}goodix_spo2_config;


typedef struct
{
	GU32 frameid;								// 帧序号
	GU8 bit_num;                                //数据位数
	GS32 ppg_rawdata[3 * CHIP_PPG_CHL_NUM];		// PPG原始数据，依次绿光1234-红外1234-红光1234
	GS8 cur_adj_flg[3 * CHIP_PPG_CHL_NUM];		// 电流调光标志位， 依次绿光1234-红外1234-红光1234
	GS8 gain_adj_flg[3 * CHIP_PPG_CHL_NUM];		// 增益调光标志位， 依次绿光1234-红外1234-红光1234
	GS8 enable_flg[3 * CHIP_PPG_CHL_NUM];		    // 通道使能标志位， 依次绿光1234-红外1234-红光1234

	GS32 ch_agc_drv0[3 * CHIP_PPG_CHL_NUM];		//  驱动电流0-依次绿光1234-红外1234-红光1234
	GS32 ch_agc_drv1[3 * CHIP_PPG_CHL_NUM];		//  驱动电流1-依次绿光1234-红外1234-红光1234
	GS8 ch_agc_gain[3 * CHIP_PPG_CHL_NUM];	    //  gain值1-依次绿光1234-红外1234-红光1234
	//ch_agc_drv0   ch_agc_drv1  ch_agc_gain
	//GS8 valid_ch_num;                            //实际使用通道数

	GS32 acc_x;				//加速度计x轴
	GS32 acc_y;				//加速度计y轴
	GS32 acc_z;				//加速度计z轴
	GS32 wear_on_flag;		//佩戴检测标志
}goodix_spo2_input_rawdata;

typedef struct
{
	//GF32 spo2[CHIP_PPG_CHL_NUM];				//血氧值
	GS32 spo2[CHIP_PPG_CHL_NUM];				//血氧值*10000
	GS32 valid_level[CHIP_PPG_CHL_NUM];		//置信等级
	GS32 confi_coeff[CHIP_PPG_CHL_NUM];		//置信度
	GS32 WeightR[CHIP_PPG_CHL_NUM];		//置信度

	//HB
	GS32 hb_mean[CHIP_PPG_CHL_NUM];
	GS32 hb_confi_lvl[CHIP_PPG_CHL_NUM];

	GS32 invalidFlg[CHIP_PPG_CHL_NUM];
	GS32 piIR[CHIP_PPG_CHL_NUM];
	GS32 piRed[CHIP_PPG_CHL_NUM];
	GS32 snrIR[CHIP_PPG_CHL_NUM];
	GS32 snrRed[CHIP_PPG_CHL_NUM];
	GS32 r_val[CHIP_PPG_CHL_NUM];				//r值（*10000）
	GS32 calc_flg[CHIP_PPG_CHL_NUM];

	//**********************四通道输出最终值***************************//
	GS32 final_spo2;
	GS32 final_valid_level;
	GS32 final_confi_coeff;
	GS32 final_WeightR;

	GS32 final_hb_mean;
	GS32 final_hb_confi_lvl;

	GS32 final_invalidFlg;
	GS32 final_piIR;
	GS32 final_piRed;
	GS32 final_snrIR;
	GS32 final_snrRed;
	GS32 final_r_val;
	GS32 final_calc_flg;
}goodix_spo2_result;


/**
* @brief SPO2 算法函数返回状态值
*/
typedef enum
{
	GX_ALGO_SPO2_SUCCESS = 0x00000000,			/**< 成功     */
	GX_ALGO_SPO2_RWONG_INPUT = 0x00000001,		/**< 输入数据格式不合法     */
	GX_ALGO_SPO2_NO_MEMORY = 0x00000002,        /**< 内存空间不够     */
	GX_ALGO_SPO2_FRAME_UNCOMPLETE = 0x00000003, /**< 处于降采样平均过程中*/
	GX_ALGO_SPO2_WIN_UNCOMPLETE = 0x00000004,   /**< 未到完整滑窗帧位置（整数s）     */
	GX_ALGO_SPO2_UNEXPECTED = 0x00000005,      /*不合设计逻辑异常 */
}goodix_spo2_ret;

#endif

#if(__HBD_HRV_ALGORITHM_ENABLE__)
#define HRV_CHNL_MAX_NUM 6
typedef struct
{
	GS32 ppg_green[HRV_CHNL_MAX_NUM];
	GS32 green_cur_adj_flag[HRV_CHNL_MAX_NUM];
	GS32 green_gain_adj_flag[HRV_CHNL_MAX_NUM];
	GS32 acc_x;
	GS32 acc_y;
	GS32 acc_z;
	GS32 frame_id;
	GS32 hr;
}ST_HRV_INPUT_INFO;


/* 返回错误码 */
#define GX_HRV_ALGO_OK					((GU32)0x00000000) // 成功返回，或无出值刷新
#define GX_HRV_ALGO_FAILED				((GU32)0x10000001) // 失败返回
#define GX_HRV_ALGO_NULL_PTR			((GU32)0x10000002) // 外部传入的指针为空
#define GX_HRV_ALGO_INVALID_PARAM		((GU32)0x10000003) // 参数范围无效
#define GX_HRV_ALGO_OUT_OF_MEM			((GU32)0x10000004) // 内存分配失败
#define GX_HRV_ALGO_DEINIT_ABORT		((GU32)0x10000005) // 释放失败
#define GX_HRV_ALGO_UPDATE              ((GU32)0x10000006) // 有出值刷新
#define PPG_CHANNEL_NUM		            4
#define RRI_NUM                         4
#define ACC_THR_NUM                     4

typedef struct {
    GS32 need_ipl;                                   // 是否需要插值到1KHz
    GS32 fs;                                         // 采样率
    GS32 acc_thr[ACC_THR_NUM];                       // acc 相关阈值，默认值为：20/10/3/3
}goodix_hrv_config;
typedef struct {
    GU8  bit_num;
    GS32 ppg_rawdata[PPG_CHANNEL_NUM];                 // PPG原始信号，最多四通道
    GS32 rawdata_cur_adj_flag[PPG_CHANNEL_NUM];        // 电流调光标志位
    GS32 rawdata_gain_adj_flag[PPG_CHANNEL_NUM];       // 增益调光标志位
    GS32 acc_x;                                      // 加速计 x 轴
    GS32 acc_y;                                      // 加速计 y 轴
    GS32 acc_z;                                      // 加速计 z 轴
    GS32 frame_id;                                   // 帧序号
    GS32 hr;                                         // 当前心率值
}goodix_hrv_input_rawdata;

typedef struct {
    GS32 rri[RRI_NUM];                               // RRI结果数组
    GS32 rri_confidence;                             // 结果置信度
    GS32 rri_valid_num;                              // RRI有效结果数量
}goodix_hrv_result;
#endif
#endif

#if(__HBD_NADT_ALGORITHM_ENABLE__)
#define NADT_CONFIG_SOFT_AUTOLED_TYPE            0
#define NADT_CONFIG_TURNLIGHT_TYPE               1
#define NADT_CONFIG_UNWEAR_TIME_TYPE             2
#define NADT_CONFIG_DETECT_TIMEOUT_TYPE          3
#define NADT_CONFIG_SAMPLE_RATE_TYPE             4
#define NADT_CONFIG_SLEEP_STATUS_TYPE            5
#define NADT_COFIG_UNWEAR_LEVEL_TYPE             6
#define NADT_CONFIG_LIVE_DETECT_EN_TYPE          7
#define NADT_CONFIG_HB_LOW_THR_TYPE              8
#define NADT_CONFIG_HB_HIGH_THR_TYPE             9
#define NADT_CONFIG_GREEN_SIG_LOW_THR_TYPE       10
#define NADT_CONFIG_GREEN_SIG_HIGH_THR_TYPE      11
#define NADT_CONFIG_BASE_RATIO_EN_TYPE           12
#define NADT_CONFIG_BASE_RATIO_THR_TYPE          13
#define NADT_CONFIG_LIVE_CONFIRM_EN_TYPE         14
#define NADT_CONFIG_IR_SIG_LOW_THR_TYPE          15
#define NADT_CONFIG_IR_SIG_HIGH_THR_TYPE         16
#define NADT_CONFIG_CTR_EN_TYPE                  17
#define NADT_CONFIG_CTR_THR_TYPE                 18
#define NADT_CONFIG_GAIN_EN_TYPE                 19
#define NADT_CONFIG_GAIN_STABLE_TIME_TYPE        20
#define NADT_CONFIG_GREEN_GAIN_THR_TYPE          21
#define NADT_CONFIG_IR_GAIN_THR_TYPE             22
#define NADT_CONFIG_PERIOD_EN_TYPE               23
#define NADT_CONFIG_PERIOD_LOW_THR_TYPE          24
#define NADT_CONFIG_PERIOD_HIGH_THR_TYPE         25
#define NADT_CONFIG_PERIOD_DIFF_THR_TYPE         26
#define NADT_CONFIG_TOTAL_CNT_TYPE               27
#define NADT_CONFIG_UNWEAR_CNT_TYPE              28
#define NADT_CONFIG_ADT_ONLY_SLEEP_TYPE          29
#define NADT_CONFIG_ADT_WEAR_OFF_THR_TYPE        30
#define NADT_CONFIG_ADT_WEAR_OFF_CNT_TYPE        31

#define NADT_CONFIG_ONE_OFFSET             1
#define NADT_CONFIG_TWO_OFFSET             2

typedef struct
{
    GS32 nSleepInState;
} ST_NADT_CONFIG;
#endif

#if (__HB_NEED_ADT_CONFIRM__)
typedef struct
{
	const GF32 *pfCoefB;
	const GF32 *pfCoefA;
	GF32 *pfXbuff;
	GF32 *pfYbuff;
	GS32 lLen;
	GF32 fThr;
} ST_IIR_PARAM;
#endif

/* from hba lib. */
extern GS32 goodix_mem_init(void* mem_addr, GS32 size);
extern void goodix_mem_deinit(void);

#if(__HBD_HB_ALGORITHM_ENABLE__)
extern goodix_hba_ret goodix_hba_version(GU8 version[100]);
extern goodix_hba_ret goodix_hba_init(goodix_hba_config const *cfg);
extern goodix_hba_ret goodix_hba_deinit(void);
extern goodix_hba_ret goodix_hba_update(goodix_hba_input_rawdata* raw, goodix_hba_result *res);
#endif
#if(__HBD_SPO2_ALGORITHM_ENABLE__)
extern goodix_spo2_ret goodix_spo2_version(GU8 version[100]);
extern goodix_spo2_ret goodix_spo2_init(goodix_spo2_config *cfg);
extern goodix_spo2_ret goodix_spo2_calc(goodix_spo2_input_rawdata* raw, goodix_spo2_result* res);
extern goodix_spo2_ret goodix_spo2_deinit(void);
#endif
#if(__HBD_NADT_ALGORITHM_ENABLE__)
extern void NADT_ProcInit(void);
extern void NADT_ProcDeInit(void);
extern GBOOL NADT_Proc(GS32 lPacketInfoArr[], GU8 lResult[]);
extern void NADT_Control(GS32 lOptTye, GS32 lConfigValue[]);
extern GS8* NADT_GetVersion(void);
#endif
#if(__HBD_HRV_ALGORITHM_ENABLE__)
extern GU8 *goodix_hrv_version(void);
extern GU32 goodix_hrv_init(goodix_hrv_config *config);
extern GU32 goodix_hrv_calc(goodix_hrv_input_rawdata *input_data, goodix_hrv_result *output_data);
extern GU32 goodix_hrv_deinit(void);
#endif

#if(__HBD_NADT_ALGORITHM_ENABLE__)
/****************************************************************
* Description: Get Nadt version
* Input:    None,
* Return:  Nadt version
******************************************************************/

#if (__HBD_ALGORITHM_EXTERNANL_CONFIG_ENABLE__)
/****************************************************************
* Description: Set NADT 
* Input:  nAdtOnlyEnable 0:ADT and NADT in sleep mode
                          1:ADT only in sleep mode   
******************************************************************/
void HBD_AdtOnlySleepTypeEnable(GS32 nAdtOnlyEnable);

/****************************************************************
* Description: Set NADT 
* Input:  ST_NADT_CONFIG stConfig  config struct for NADT
******************************************************************/
void NADT_ParamConfigure(ST_NADT_CONFIG stConfig);
#endif

#endif

#if(__HBD_HB_ALGORITHM_ENABLE__)
/****************************************************************
* Description: Get hba algorithm version
* Input:    None,
* Return: library hba algorithm version
******************************************************************/
GS8 * HBD_GetHbaVersion (void);

#if (__HBD_ALGORITHM_EXTERNANL_CONFIG_ENABLE__)
/****************************************************************
* Description: Hb algorithm scenario config 
* Input:    uchScenario: 0~24,see hba_scenes_e
                         others: fixed 0(default) and return HBD_RET_PARAMETER_ERROR
* Return: HBD_RET_OK=success, 
          HBD_RET_PARAMETER_ERROR=paramters error,
******************************************************************/
GS8 HBD_HbAlgoScenarioConfig (hba_scenes_e uchScenario);

/****************************************************************
* Description: Hb algorithm test mode config ,before HBD_HbDetectStart
* Input:    emHbaTestMode: test mode, see hba_test_mode
            usSenselessModeStep
            usSenselessModeDuration
******************************************************************/
void HBD_HbaTestModeConfig (hba_test_mode emHbaTestMode, GU16 usSenselessModeStep, GU16 usSenselessModeDuration);

/****************************************************************
* Description: get current Hb algorithm test mode
* Input:    emCurHbaTestMode: test mode, see EM_HBA_TEST_MODE
******************************************************************/
GU8 HBD_GetHbaTestMode (void);

/****************************************************************
* Description: Hb algorithm output time config, only nonFast mode valid 
* Input:    nHbaLatestOutputTime
            nHbaEarliestOutputTime
* Return: HBD_RET_OK=success, 
******************************************************************/
GS8 HBD_HbAlgoOutputTimeConfig (GS32 nHbaLatestOutputTime, GS32 nHbaEarliestOutputTime);
#endif
#endif

/****************************************************************
* Description: Hb algorithm sleep flag config,before hb calculate     
* Input:    uchSleepFlg: 0:not sleep; 1:sleep
******************************************************************/
void HBD_HbaSleepFlagConfig(GU8 uchSleepFlg);

/****************************************************************
* Description: Hb algorithm get sleep flag
* Return:    guchSleepFlag: 0:not sleep; 1:sleep
******************************************************************/
GU8 HBD_HbaGetSleepFlag(void);

#if(__HBD_HRV_ALGORITHM_ENABLE__)
/****************************************************************
* Description: Get hrv algorithm version
* Input:    None,
* Return: library hrv algorithm version
******************************************************************/
GS8 * HBD_GetHrvVersion(void);
#endif

#if(__HBD_SPO2_ALGORITHM_ENABLE__)
/****************************************************************
* Description: Get SPO2 algorithm version
* Input:    None,
* Return: library SPO2 algorithm version
******************************************************************/
GU8 * HBD_GetSpo2Version (void);
#endif

#if (__HB_NEED_ADT_CONFIRM__)

void IIRFilter2Init(ST_IIR_PARAM *pstIIRInfo, const GF32 *pfCoefA, const GF32 *pfCoefB, GF32 *pfXbuff, GF32 *pfYbuff, GS32 lLen, GF32 fThr);
GF32 IIRFilterEx(ST_IIR_PARAM *pstIIRInfo, GF32 fDataIn);

/****************************************************************
* Description: adt confrim init
* Input:  ulNewAdtPpgThrVal : adt ppg thr val
* Return: None
******************************************************************/
void ADTConfirmAlgoInit(GU32 ulNewAdtPpgThrVal);

/****************************************************************
* Description: adt confrim calc
* Input:  *plRawdataBuff : rawdata bufferr
* Return: wear flag, 0x00: ThrCntMax error 0x11:wear, 0x12:unwear
******************************************************************/
GU8 ADTConfirmAlgoCalc(GS32 *plRawdataBuff);

/****************************************************************
* Description: config adt confrim
* Input:  usAdtConfirmGsThrVal : gsensor confirm thr
          uchAdtConfirmGsCalcThrCntMax: gsensor confirm thr cnt max 
          uchAdtConfirmGsCalcThrCnt  : gsensor confirm thr cnt
* Return: None
******************************************************************/
void HBD_AdtConfirmConfig(GU16 usAdtConfirmGsThrVal, GU8 uchAdtConfirmGsCalcThrCntMax, GU8 uchAdtConfirmGsCalcThrCnt);
#endif
#endif
