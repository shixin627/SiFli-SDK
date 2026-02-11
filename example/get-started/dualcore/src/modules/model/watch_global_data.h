#ifndef __watch_GLOBAL_DATA_H__
#define __watch_GLOBAL_DATA_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include <stdbool.h>
#include "share_prefs.h"
#include "watch_sys_service.h"
#include "bsp_board.h"

#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_REVISION 230
#define VERSION_DEV 1

#define MAX_GESTURE_SAMPLES 76
#define GESTURE_TAP_TIME_STEP 20
#define GESTURE_RELEASE_TIME_STEP 35

#define FEATURE_NUM 7

#define BYTES_PER_SAMPLE                                                       \
    (6 * 2 + 6 + 2 +                                                           \
     1) // 6軸，每軸2字節 + timestamp 6字節(UINT32 + UINT16 ) + ppg_data 2字節

/* Feature and buffer size definitions */
#define BLE_G_SENSOR_SAMPLE_AMOUNT (SAMPLE_SIZE) /* sample count */
#if USE_FFT_FILTER
    #define BYTES_PER_FEATURE (2)
    #define BLE_G_SENSOR_BUF_SIZE                                              \
        ((FEATURE_NUM + 1) * BYTES_PER_FEATURE * SAMPLE_SIZE) /* total bytes   \
                                                               */
#else
    #define BLE_G_SENSOR_BUF_SIZE (BYTES_PER_SAMPLE * MAX_RAWDATA_TIME_STEP)
    extern uint8_t gsensorSamplesBuffer[BLE_G_SENSOR_BUF_SIZE];
#endif

#define USER_ID_LENGTH (28)

    enum sys_power_status
    {
        SYS_POWER_STATUS_ON = 0,
        SYS_POWER_STATUS_SLEEP = 1,
        SYS_POWER_STATUS_OFF = 2,
    };

    typedef struct
    {
        float x;
        float y;
        float z;
    } Vector3;

    typedef struct
    {
        uint32_t wait_start_time;
        bool gesture_started;
        bool gesture_ended;
        int gesture_sample_count;
        float waveform[MAX_RAWDATA_TIME_STEP][FEATURE_NUM];
        uint32_t timestamp_s[MAX_RAWDATA_TIME_STEP];
        uint16_t timestamp_ms[MAX_RAWDATA_TIME_STEP];
        uint16_t ppg_data[MAX_RAWDATA_TIME_STEP];
        bool on_pressed[MAX_RAWDATA_TIME_STEP];
    } gesture_dataset_t;

    typedef struct
    {
        float w;
        float x;
        float y;
        float z;
    } Quaternion;

    typedef struct
    {
        float roll;
        float pitch;
        float yaw;
    } euler_angle_t;

    typedef enum
    {
        UnknownTool,
        WeatherTool,
        CalendarQueryTool,
        CalendarCreateTool,
        CreateNoteTool,
        WebPageTool,
        WebSearchTool,
        RagTool,
        FinanceTool,
        CurrencyConversionTool,
        ImageAnalyzeTool,
        HintTool,
    } LangchainToolKey;

#define MAX_ALARM_NUM (3) // support 3 alarm now

    extern uint8_t heart_series_buf[60];
    extern uint8_t heart_rate_result_buf[80];

    typedef struct
    {
        uint32_t seconds : 6;
        uint32_t minute : 6;
        uint32_t hours : 5;
        uint32_t day : 5;
        uint32_t month : 4;
        uint32_t year : 6;
    } time_bit_field_type_t;

    typedef union
    {
        uint32_t data;
        time_bit_field_type_t time;
    } time_union_t;

    typedef struct
    {
        uint64_t day_repeat_flag : 7;
        uint64_t reserved : 4;
        uint64_t id : 3;
        uint64_t minute : 6;
        uint64_t hour : 5;
        uint64_t day : 5;
        uint64_t month : 4;
        uint64_t year : 6;
    } alarm_clock_bit_field_type_t;

    typedef union
    {
        uint64_t data;
        alarm_clock_bit_field_type_t alarm;
    } T_ALARM;

    typedef struct
    {
        uint64_t reserved : 8;
        uint64_t on_off : 8;
        uint64_t step_low_limit : 16;
        uint64_t sit_min : 8;
        uint64_t start_hour : 8;
        uint64_t end_hour : 8;
        uint64_t day_flag_bits : 8;
    } long_time_sit_alert_field_type_t;

    typedef union
    {
        uint64_t data;
        long_time_sit_alert_field_type_t sit_alert;
    } T_SIT_ALERT;
    typedef enum
    {
        TGET_UP = 0x00,
        TSLEEP = 0x01,
        TDEEP_SLEEP = 0x02,
        TNOT_WEAR = 0x03,
    } T_SLEEP_STATUS;

    typedef enum
    {
        NoCharge = 0,     /* Show not connect to plugs*/
        InCharging,       /* Device connect to the plugs*/
        ChargingComplete, /* Show charging complete & still connect to plugs*/
    } T_CHARGE_STATUS;

    typedef struct
    {
        uint32_t reserve : 5;
        uint32_t gender : 1; // 0-female, 1-male
        uint32_t age : 7;
        uint32_t height : 9;  // Q1, cm
        uint32_t weight : 10; // Q1, kg
    } usr_prof_t;

    typedef union
    {
        uint32_t data;
        usr_prof_t bit_field;
    } userprofile_union_t;

    typedef struct
    {
        uint8_t user_id[32];
        userprofile_union_t user_profile;
    } T_USER_DATA;

    typedef struct
    {
        volatile uint16_t year;   // 2000+
        volatile uint8_t month;   // 0-11
        volatile uint8_t day;     // 0-30
        volatile uint8_t seconds; // 0-59
        volatile uint8_t minutes; // 0-59
        volatile uint8_t hour;    // 0-23
        volatile uint8_t weekday; // 0-6
    } T_UTC_TIME;

    typedef union
    {
        uint32_t data;
        struct
        {
            uint32_t stopwatch_sw : 1;
            uint32_t stopwatch_st : 1;
            uint32_t findPhone_sw : 1;
            uint32_t findPhone_st : 1;
            uint32_t lockScreen_sw : 1;
            uint32_t lockScreen_st : 1;
            uint32_t realtimeHR_sw : 1;
            uint32_t realtimeHR_st : 1;
        } status;
    } Hidden_FunC_t;

    /**
     * @brief Structure to hold global pedometer data
     */
    typedef struct
    {
        volatile uint16_t quarter_steps;
        volatile uint32_t quarter_distance;
        volatile uint32_t quarter_calories;

        volatile uint32_t global_steps;
        volatile uint32_t global_distance;
        volatile uint32_t global_calories;

        volatile uint32_t daily_step_target;
        volatile uint16_t daily_sleep_target;
    } T_PEDO_DATA;

    /**
     * @brief Structure to hold storage addresses
     */
    typedef struct
    {
        volatile uint32_t p_read_addr;  /**< @brief Read address pointer */
        volatile uint32_t p_write_addr; /**< @brief Write address pointer */
    } T_STORAGE_ADDR;

    /**
     * @brief Structure to hold sleep data
     */
    typedef struct
    {
        uint8_t sleep_data_month; /**< @brief Month of sleep data */
        uint8_t sleep_data_day;   /**< @brief Day of sleep data */
        uint8_t sleep_data_hour;  /**< @brief Hour of sleep data */

        uint8_t stamp_counts; /**< @brief Count of timestamps */

        uint32_t prev_sleep_stamp; /**< @brief Previous sleep timestamp */

        uint16_t light_sleep_time; /**< @brief Duration of light sleep */
        uint16_t deep_sleep_time;  /**< @brief Duration of deep sleep */
        uint16_t wake_up_time;     /**< @brief Wake up time */

        uint16_t light_sleep_time_18_1; /**< @brief Duration of light sleep from
                                           18:00 to 01:00 */
        uint16_t deep_sleep_time_18_1;  /**< @brief Duration of deep sleep from
                                           18:00 to 01:00 */
        uint16_t
            wake_up_time_18_1; /**< @brief Wake up time from 18:00 to 01:00 */

        uint16_t light_sleep_time_1_5; /**< @brief Duration of light sleep from
                                          01:00 to 05:00 */
        uint16_t deep_sleep_time_1_5;  /**< @brief Duration of deep sleep from
                                          01:00 to 05:00 */
        uint16_t
            wake_up_time_1_5; /**< @brief Wake up time from 01:00 to 05:00 */

        uint16_t light_sleep_time_5_10; /**< @brief Duration of light sleep from
                                           05:00 to 10:00 */
        uint16_t deep_sleep_time_5_10;  /**< @brief Duration of deep sleep from
                                           05:00 to 10:00 */
        uint16_t
            wake_up_time_5_10; /**< @brief Wake up time from 05:00 to 10:00 */

        uint16_t
            light_sleep_time_wake_to_10;     /**< @brief Duration of light sleep
                                                from wake up time to 10:00 */
        uint16_t deep_sleep_time_wake_to_10; /**< @brief Duration of deep sleep
                                                from wake up time to 10:00 */
        uint16_t wake_up_time_wake_to_10; /**< @brief Wake up time from wake up
                                             time to 10:00 */

        uint8_t invalid_sleep_data; /**< @brief Flag to disable data change
                                       after this point */

        uint8_t first_wake_up_after_5; /**< @brief Flag to indicate the first
                                          wake up after 05:00 */
    } T_SLEEP_DATA;

    /** @brief BBPRO HCI LINK connection states*/
    typedef enum
    {
        BBPRO_HCI_LINK_STATE_DISCONNECTED, //!< Disconnected.
        BBPRO_HCI_LINK_STATE_CONNECTING,   //!< Connecting.
        BBPRO_HCI_LINK_STATE_CONNECTED,    //!< Connected.
        BBPRO_HCI_LINK_STATE_DISCONNECTING //!< Disconnecting.
    } T_BBPRO_LINK_STATE;

    /**
     * @brief Device state Event type
     */
    typedef enum
    {
        APP_STATE_OFF = 0x00,
        APP_STATE_PAIRING = 0x01,
        APP_STATE_STANDBY = 0x02,
        APP_STATE_CONNECTED = 0x03,
        APP_STANDBY_WITH_LINK_BACK = 0x04,
        APP_CONNECTED_WITH_LINK_BACK = 0x05,
        APP_STATE_RESERVED = 0x06,
    } EVENT_DEVICE_STATE;

    typedef struct
    {
        volatile bool paired_flag;
        volatile uint8_t paired_num;
        volatile uint8_t newest_addr[8];
    } T_BBPRO_PAIRED_INFO;

    // TODO: replace with ble sible struct
    /** @defgroup Gap_Msg_Exported_Types GAP Msg Exported Types
     * @{
     */
    /** @brief  Device State.*/
    typedef struct
    {
        uint8_t gap_init_state : 1;    //!< @ref GAP_INIT_STATE
        uint8_t gap_adv_sub_state : 1; //!< @ref GAP_ADV_SUB_STATE
        uint8_t gap_adv_state : 2;     //!< @ref GAP_ADV_STATE
        uint8_t gap_scan_state : 2;    //!< @ref GAP_SCAN_STATE
        uint8_t gap_conn_state : 2;    //!< @ref GAP_CONN_STATE
    } T_GAP_DEV_STATE;

    /** @brief GAP connection states*/
    typedef enum
    {
        GAP_CONN_STATE_DISCONNECTED, //!< Disconnected.
        GAP_CONN_STATE_CONNECTING,   //!< Connecting.
        GAP_CONN_STATE_CONNECTED,    //!< Connected.
        GAP_CONN_STATE_DISCONNECTING //!< Disconnecting.
    } T_GAP_CONN_STATE;

    /* SkaiWatchSys flag bit field */
    typedef struct
    {
        uint64_t low_power_flag : 1;
        uint64_t power_off_flag : 1;
        uint64_t is_paired : 1;
        uint64_t device_had_logged : 1;
        uint64_t algorithm_started : 1;
        uint64_t daily_target_achieved : 1;
        uint64_t phone_camera_status : 1;
        uint64_t lift_switch_status : 1;
        uint64_t auto_sync_enable : 1;
        uint64_t pedo_detect_enable : 1;
        uint64_t hrs_detect_onetime : 1;
        uint64_t heart_sample_by_phone : 1;
        uint64_t bp_sample_by_phone : 1;
        uint64_t bond_state : 1;
        uint64_t is_calling_action_alive : 1;
        uint64_t is_calling_ring_start : 1;
        uint64_t hour_format : 1;
        uint64_t distance_unit : 1;
        uint64_t bps_hidden : 1;
        uint64_t bps_detecting : 1;
        uint64_t bps_timeout : 1;
        uint64_t twist_switch_status : 1;
        uint64_t hr_sensor_state : 1;
        uint64_t alarm_snooze_status : 1;
        uint64_t sleep_detect_enable : 1;
        uint64_t system_lock_screen : 1;
        uint64_t device_lock_screen : 1;
        uint64_t headset_pair_button : 1;
        uint64_t headset_pair_handler : 1;
        uint64_t headset_pair_state : 1;
        uint64_t restore_factory : 1;
        uint64_t is_calling_action_accept : 1;
        uint64_t is_wearing : 1;
        uint64_t stopwatch_status : 1;
        uint64_t countdown_status : 1;
        uint64_t findphone_status : 1;
        uint64_t incoming_call_ready_accept : 1;
        uint64_t countdown_popopen : 1;
        uint64_t debug_mode : 1;
        uint64_t reserved : 25;
    } T_FLAG_FIELD;

    /*Do not disturb mode configuration */
    typedef union
    {
        uint32_t data;
        struct
        {
            uint32_t end_min : 6;
            uint32_t end_hour : 5;
            uint32_t start_min : 6;
            uint32_t start_hour : 5;
            uint32_t status : 1;
            uint32_t DNDM_start : 1;
            uint32_t reserve : 8;
        } config;
    } T_DND_MODE;

    typedef union
    {
        uint32_t data;
        struct
        {
            uint32_t switch_call_msg : 1;
            uint32_t switch_qq_msg : 1;
            uint32_t switch_wechat_msg : 1;
            uint32_t switch_shortmessage_msg : 1;
            uint32_t switch_line_msg : 1;
            uint32_t switch_twitter_msg : 1;
            uint32_t switch_facebook_msg : 1;
            uint32_t switch_facebookmessage_msg : 1;
            uint32_t switch_wahtapp_msg : 1;
            uint32_t switch_linkedin_msg : 1;
            uint32_t switch_instagram_msg : 1;
            uint32_t switch_skype_msg : 1;
            uint32_t switch_viber_msg : 1;
            uint32_t switch_kakaotalk_msg : 1;
            uint32_t switch_vkontakte_msg : 1;
            uint32_t switch_mail_msg : 1;
            uint32_t switch_calendar_msg : 1;
            uint32_t switch_appfacetime_msg : 1;
            uint32_t switch_tim_msg : 1;
            uint32_t switch_gmail_msg : 1;
            uint32_t switch_dingtalk_msg : 1;
            uint32_t switch_workwechat_msg : 1;
            uint32_t switch_aplus_msg : 1;
            uint32_t switch_link_msg : 1;
            uint32_t switch_beike_msg : 1;
            uint32_t switch_lianjia_msg : 1;
            uint32_t msg_reserved : 6;
        } bit;
    } T_MSG_SWITCH;

    typedef struct
    {
        volatile uint8_t cur_msg_counts;
        volatile uint8_t max_msg_counts;
        volatile uint8_t single_package_len;
        volatile uint8_t reserved;
        volatile uint32_t p_read_start_addr;
        volatile uint32_t p_write_start_addr;
    } T_MSG_DATA_CONFIG;

    typedef enum
    {
        CLOCK_1ST_MENU,
        CLOCK_2ND_MENU,
        CLOCK_3RD_MENU,
        CLOCK_4TH_MENU,
        CLOCK_5TH_MENU,
        CLOCK_6TH_MENU,
        CLOCK_7TH_MENU,
        CLOCK_8TH_MENU,
        CLOCK_MAX_MENU,
    } T_CLOCK_MENU_TYPE;

    /// @brief watch system type, the symbol "o" mark the data need to be stored
    /// in flash
    /// @param
    typedef struct
    {
        volatile T_FLAG_FIELD flag_field; // o
        volatile T_MSG_SWITCH msg_switch; // o
        volatile uint8_t battery_level_value;
        volatile uint8_t hrs_detect_period;
        volatile uint8_t hrs_start_up_mode;
        volatile uint8_t heart_rate_bpm;
        volatile uint8_t phone_os_version;  // o
        volatile uint8_t alarm_num;         // o
        volatile uint8_t oled_display_time; // o
        volatile uint8_t language;          // o
        volatile uint8_t clock_screen_num;  // o
        volatile uint8_t watch_restart_num; // o
        volatile uint8_t clock_widget_num;  // o
        volatile uint32_t calendar_sync_time;
        volatile uint32_t weather_sync_time;
        volatile uint8_t weather_moment_count;
        volatile uint8_t ota_progress;
        volatile uint8_t brightness; // o
        volatile uint16_t battery_vol_value;
        volatile uint16_t current_stationary_time;
        volatile uint32_t SecondCountRTC;
        volatile uint32_t pre_hcpu_wakeup_tick;

        volatile T_SLEEP_STATUS watch_sleep_status;
        volatile T_CHARGE_STATUS charger_status;
        volatile T_PEDO_DATA gPedoData;  // o
        volatile T_DND_MODE DNDMode;     // o
        T_ALARM alarms[MAX_ALARM_NUM];   // o
        volatile T_USER_DATA user_data;  // o
        T_SIT_ALERT sit_alert_data;      // o
        volatile T_UTC_TIME Global_Time; // o
        volatile T_MSG_DATA_CONFIG msg_data_config;

        volatile T_SLEEP_DATA sleep_data_show; // o
        volatile T_BBPRO_LINK_STATE bbpro_hci_link_status;
        volatile EVENT_DEVICE_STATE bbpro_device_status; // 315/320
        volatile T_BBPRO_PAIRED_INFO paired_info;
        volatile T_GAP_DEV_STATE gap_dev_state;   /**< GAP device state */
        volatile T_GAP_CONN_STATE gap_conn_state; /**< GAP connection state */
        volatile T_CLOCK_MENU_TYPE clock_status;  // o
        bool connected_to_phone;
        uint8_t watch_conn_id;
        uint16_t conn_interval;
        uint16_t conn_latency;
        uint16_t conn_superv_tout;
        uint16_t watch_mtu;
        uint8_t notification_number;
        uint8_t notelist_number;
        watch_sys_minute_activity_t activity;
        uint8_t font_size;
        watch_sys_heath_info_t health_info_today;
        watch_sys_sleep_state_t sleep_state;
        uint8_t wakeup_reson;
        uint8_t reboot_reason;
        uint8_t sys_power_status;
        bool idle_state;
        bool motion_control_lock;
        bool quick_open_app;
        bool display_half_brightness;
    }
#ifdef BSP_USING_PC_SIMULATOR
    SkaiWatchSysType_t;
#else
__attribute__((packed)) SkaiWatchSysType_t;
#endif

    typedef enum
    {
        SENSOR_TYPE_ACCELEROMETER,
        SENSOR_TYPE_GYROSCOPE,
        SENSOR_TYPE_MAGNETOMETER,
        SENSOR_TYPE_STEP_COUNTER,
        SENSOR_TYPE_WAKE_GESTURE,
        SENSOR_TYPE_PICK_UP_GESTURE,
        SENSOR_TYPE_POSE_6DOF,
        SENSOR_TYPE_HEART_RATE,
        SENSOR_TYPE_PPG,
        SENSOR_TYPE_MIC,
    } SENSOR_TYPE;

    typedef struct
    {
        uint8_t type;
        uint8_t status;
        uint8_t thread_safe; // 0: not safe, 1: safe
    } sensor_subscription_t;

    // Define an enum for message parts
    typedef enum
    {
        MESSAGE_PART_START,
        MESSAGE_PART_MIDDLE,
        MESSAGE_PART_END
    } MESSAGE_PART;

    typedef struct
    {
        MESSAGE_PART header; // 0: start, 1: middle, 2: end
        uint8_t *p_msg_value;
        uint16_t length;
    } MSG_DATA_PAYLOAD;

    typedef struct
    {
        uint8_t header; // sentence index
        uint8_t *p_msg_value;
        uint16_t length;
    } VOICE_RECOGNITION_PAYLOAD;

    typedef struct
    {
        char app_id[16];
        char intent[32];
        void *param;
    } AppIntent;

    typedef enum
    {
        WATCH_PREFS_KEY_FLAG_FIELD,
        WATCH_PREFS_KEY_MSG_SWITCH,
        WATCH_PREFS_KEY_PHONE_OS_VERSION,
        WATCH_PREFS_KEY_OLED_DISPLAY_TIME,
        WATCH_PREFS_KEY_LANGUAGE,
        WATCH_PREFS_KEY_CLOCK_SCREEN_NUM,
        WATCH_PREFS_KEY_BACKLIGHT_PERCENT,
        WATCH_PREFS_KEY_PEDODATA,
        WATCH_PREFS_KEY_DNDMODE,
        WATCH_PREFS_KEY_USER_DATA,
        WATCH_PREFS_KEY_SIT_ALERT_DATA,
        WATCH_PREFS_KEY_GLOBAL_TIME,
        WATCH_PREFS_KEY_SLEEP_DATA_SHOW,
        WATCH_PREFS_KEY_CLOCK_STATUS,
        WATCH_PREFS_KEY_GESTURE_THRESHOLD,
        WATCH_PREFS_NUM_KEYS, // This is used to get the number of fields in the
                              // enum
    } watch_prefs_key;

    extern void store_watch_prefs(watch_prefs_key key);

    /// @brief watch preferences type based on the data in SkaiWatchSysType_t
    /// which need to be stored in flash
    typedef struct
    {
        share_prefs_t *pref;
        void (*read_flag_field)(share_prefs_t *pref);
        void (*write_flag_field)(share_prefs_t *pref);
        void (*read_msg_switch)(share_prefs_t *pref);
        void (*write_msg_switch)(share_prefs_t *pref);
        void (*read_phone_os_version)(share_prefs_t *pref);
        void (*write_phone_os_version)(share_prefs_t *pref);
        void (*read_oled_display_time)(share_prefs_t *pref);
        void (*write_oled_display_time)(share_prefs_t *pref);
        void (*read_language)(share_prefs_t *pref);
        void (*write_language)(share_prefs_t *pref);
        void (*read_clock_screen_num)(share_prefs_t *pref);
        void (*write_clock_screen_num)(share_prefs_t *pref);
        void (*read_brightness)(share_prefs_t *pref);
        void (*write_brightness)(share_prefs_t *pref);
        void (*read_restart_num)(share_prefs_t *pref);
        void (*reset_restart_num)(share_prefs_t *pref);
        void (*read_clock_widget_num)(share_prefs_t *pref);
        void (*write_clock_widget_num)(share_prefs_t *pref);
        void (*read_pedo_data)(share_prefs_t *pref);
        void (*write_pedo_data)(share_prefs_t *pref);
        void (*read_dnd_mode)(share_prefs_t *pref);
        void (*write_dnd_mode)(share_prefs_t *pref);
        void (*read_user_data)(share_prefs_t *pref);
        void (*write_user_data)(share_prefs_t *pref);
        void (*read_sit_alert)(share_prefs_t *pref);
        void (*write_sit_alert)(share_prefs_t *pref);
        void (*read_global_time)(share_prefs_t *pref);
        void (*write_global_time)(share_prefs_t *pref);
        void (*read_sleep_data)(share_prefs_t *pref);
        void (*write_sleep_data)(share_prefs_t *pref);
        void (*read_clock_status)(share_prefs_t *pref);
        void (*write_clock_status)(share_prefs_t *pref);
        void (*read_gesture_threshold)(share_prefs_t *pref);
        void (*write_gesture_threshold)(share_prefs_t *pref);
    } WatchPrefs_t;

    extern SkaiWatchSysType_t SkaiWatchSys;
    extern WatchPrefs_t WatchPrefs;
    extern void watch_config_struct_flash_read(void);

    typedef enum
    {
        Notify_calendar,
        Notify_facebook,
        Notify_facetime,
        Notify_instagram,
        Notify_kakaotalk,
        Notify_line,
        Notify_linkedin,
        Notify_mail,
        Notify_messenger,
        Notify_others,
        Notify_QQ,
        Notify_skype,
        Notify_SMS,
        Notify_snap,
        Notify_twitter,
        Notify_wechat,
        Notify_whatsapp,
        Notify_gmail,
        Notify_dingtalk,
        Notify_googlechat,
        Notify_discord,
        Notify_youtube,
        NOTIFICATION_APP_QUANTITY,
    } Notifications_Type; // NOTIFICATIONS_TYPE;

    /******************************************************************************
     *                              BT status parameters
     ******************************************************************************/
#define MSG_DATA_PACKAGE_LEN 64

    extern void watch_storage_api_lock(void);
    extern void watch_storage_api_unlock(void);

    extern void reset_watch_config(void);
    extern void watch_config_struct_flash_write(void);
    extern void watch_config_data_init(void);
#ifdef __cplusplus
}
#endif

#endif //__watch_GLOBAL_DATA_H__
