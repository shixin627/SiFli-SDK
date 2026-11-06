/* Host-only: compile with a C99 compiler; no SDK or watch build required. */
#include "../example/get-started/dualcore/src/modules/communicate/web_control_state.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    web_control_state_t s = {0};
    web_control_touch(&s, true, 0);
    assert(!s.touching && !web_control_sample(&s, 50, 90, 90));
    s.enabled = s.route = s.active = true;
    web_control_touch(&s, true, 100);
    s.sent_ms = 100;
    assert(s.touching && s.yaw == 0 && s.pitch == 0);
    assert(!web_control_sample(&s, 125, 40, -20));
    assert(web_control_sample(&s, 150, 40, -20));
    assert(fabsf(s.yaw - 2) < 0.0001f && fabsf(s.pitch + 1) < 0.0001f);
    s.sent_ms = 150;
    /* A steady wrist retains its commanded angle; it is not cursor velocity. */
    assert(web_control_sample(&s, 200, 0.2f, -0.2f));
    assert(s.yaw == 2 && s.pitch == -1);
    web_control_touch(&s, false, 201);
    assert(!s.touching && s.yaw == 0 && s.pitch == 0);
    assert(web_control_sample(&s, 250, 40, -20));
    assert(s.yaw == 0 && s.pitch == 0); /* Handfree motion cannot move a web app. */
    web_control_touch(&s, true, 300);
    s.sent_ms = 300;
    web_control_sample(&s, 350, -40, 20);
    assert(s.yaw == -2 && s.pitch == 1); /* Left and forward/down signs. */
    assert(web_control_sample(&s, 500, 90, 90));
    assert(!s.touching && s.yaw == 0 && s.pitch == 0); /* Stale sensor cancels. */
    web_control_touch(&s, true, 501);
    assert(web_control_sample(&s, 502, NAN, 0));
    assert(!s.touching && s.yaw == 0);
    web_control_touch(&s, true, 600);
    for (uint32_t now = 650; now <= 2600; now += 50)
        web_control_sample(&s, now, 4000, -4000);
    assert(s.yaw == 180 && s.pitch == -180);
    web_control_touch(&s, true, UINT32_MAX - 24);
    s.sent_ms = UINT32_MAX - 24;
    assert(web_control_sample(&s, 25, 40, -20));
    assert(s.yaw == 2 && s.pitch == -1); /* Millisecond rollover. */
    s.route = false;
    web_control_touch(&s, true, 30);
    assert(!s.touching && !web_control_sample(&s, 100, 90, 90));
    s.route = true;
    s.active = false;
    web_control_touch(&s, true, 100);
    assert(!s.touching && !web_control_sample(&s, 150, 90, 90));
    puts("web_control_state_test: PASS");
    return 0;
}
