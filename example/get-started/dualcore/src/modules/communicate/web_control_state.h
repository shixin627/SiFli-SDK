/* Opt-in web control math, independent of RT-Thread so it can be host-tested. */
#ifndef SKAI_WEB_CONTROL_STATE_H
#define SKAI_WEB_CONTROL_STATE_H
#include <stdbool.h>
#include <stdint.h>

#define WEB_CONTROL_HEARTBEAT_MS 50u
#define WEB_CONTROL_MAX_SAMPLE_GAP_MS 100u
/* ponytail: physical sensor tuning knob; do not integrate stationary gyro noise. */
#define WEB_CONTROL_DEADZONE_DPS 0.8f

typedef struct {
    bool enabled, route, active, touching;
    uint32_t sample_ms, sent_ms;
    float yaw, pitch;
} web_control_state_t;

static void web_control_release(web_control_state_t *s, uint32_t now)
{
    s->touching = false;
    s->yaw = s->pitch = 0;
    s->sample_ms = now;
}

static bool web_control_ready(const web_control_state_t *s)
{
    return s->enabled && s->route && s->active;
}

static void web_control_touch(web_control_state_t *s, bool down, uint32_t now)
{
    web_control_release(s, now);
    s->touching = down && web_control_ready(s);
}

static float web_control_integrate(float angle, float rate, float seconds)
{
    if (rate > -WEB_CONTROL_DEADZONE_DPS && rate < WEB_CONTROL_DEADZONE_DPS)
        return angle;
    angle += rate * seconds;
    return angle > 180 ? 180 : angle < -180 ? -180 : angle;
}

/* Rates are physical degrees/second: right yaw and downward pitch are positive.
 * A scheduler/sensor gap cancels contact instead of integrating a stale rate. */
static bool web_control_sample(web_control_state_t *s, uint32_t now,
                               float yaw_rate, float pitch_rate)
{
    if (!web_control_ready(s)) return false;
    uint32_t elapsed = now - s->sample_ms;
    s->sample_ms = now;
    if (!(yaw_rate >= -4000 && yaw_rate <= 4000 &&
          pitch_rate >= -4000 && pitch_rate <= 4000) ||
        elapsed > WEB_CONTROL_MAX_SAMPLE_GAP_MS)
    {
        bool was_down = s->touching;
        web_control_release(s, now);
        if (was_down) return true;
    }
    else if (s->touching)
    {
        s->yaw = web_control_integrate(s->yaw, yaw_rate, elapsed * 0.001f);
        s->pitch = web_control_integrate(s->pitch, pitch_rate, elapsed * 0.001f);
    }
    return now - s->sent_ms >= WEB_CONTROL_HEARTBEAT_MS;
}
#endif
