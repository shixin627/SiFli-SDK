#ifndef __SYSYEM_SCHEDULE_H__
#define __SYSYEM_SCHEDULE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

	/**
	 * @brief Status values returned by sit alert processing
	 */
	typedef enum
	{
		SIT_ALERT_NONE,		// No alert needed
		SIT_ALERT_TRIGGERED // Alert should be shown to user
	} sit_alert_status_t;

	/**
	 * @brief Process the sit alert monitoring
	 * @return SIT_ALERT_NONE if no alert needed, SIT_ALERT_TRIGGERED if alert should be shown
	 */
	sit_alert_status_t sit_alert_process(void);

	/**
	 * @brief Configure sit alert settings
	 * @param enabled Set to true to enable, false to disable
	 * @param step_limit Minimum step count to consider as activity
	 * @param sit_minutes Maximum sitting time before alert (in minutes)
	 * @param start_hour Hour to start monitoring (0-23)
	 * @param end_hour Hour to end monitoring (0-23)
	 * @param day_flags Bit flags for days to monitor (bit 0 = Sunday)
	 * @return true if settings were applied, false otherwise
	 */
	bool sit_alert_configure(bool enabled, uint16_t step_limit, uint8_t sit_minutes,
							 uint8_t start_hour, uint8_t end_hour, uint8_t day_flags);

	/**
	 * @brief Reset sit alert tracking after user acknowledges the alert
	 */
	void sit_alert_acknowledge(void);

	/**
	 * @brief Handle sit alert dismissal from UI
	 */
	void on_sit_alert_dismissed(void);

	rt_err_t bloc_system_schedule_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSYEM_SCHEDULE_H__ */