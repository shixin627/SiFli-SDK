/**
 * @file   app_onboarding.h
 * @brief  Watch onboarding tutorial overlay (4-direction drawer hints +
 *         release/tap gesture cues). Public API.
 */
#ifndef __APP_ONBOARDING_H__
#define __APP_ONBOARDING_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start the tutorial overlay. Idempotent — no-op if already active.
 * Safe to call from MSH thread. The overlay will stop itself when the
 * user reaches the AI page (the final tutorial milestone). */
void onboarding_start(void);

/* Stop and tear down the overlay early. */
void onboarding_stop(void);

/* True while the overlay is showing. */
bool onboarding_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ONBOARDING_H__ */
