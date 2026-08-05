/**
 * PC-simulator validation of hr_autocorr.c against the same 24 cases as the
 * reference implementation (modules/algorithm/health/hr_autocorr_test.py).
 *
 * The estimator is pure integer C with no RTOS dependency, so the simulator can
 * exercise the SAME object the watch runs. This separates two questions that
 * would otherwise be tangled on real hardware: "is the algorithm right?" (the
 * Python suite answers that) and "did the fixed-point port break it?" (this
 * does). Only after both are green is it worth flashing.
 *
 * MSH:  hr_ac_test
 */
#include <rtthread.h>

#ifdef BSP_USING_PC_SIMULATOR

#include <math.h>
#include <stdint.h>
#include "hr_autocorr.h"

#define N HR_AUTOCORR_WIN

/* Deterministic LCG — the C must be reproducible run to run, and it must not
   depend on the host libc's rand() differing from Python's. Noise is only ever
   compared in aggregate (does the estimate land within tolerance), so matching
   Python's exact noise sequence is neither possible nor necessary. */
static uint32_t s_lcg;
static float frnd(void)
{
    s_lcg = s_lcg * 1103515245u + 12345u;
    return (float)((s_lcg >> 16) & 0x7FFF) / 32767.0f;
}

static void synth(uint32_t *out, float bpm, float h2, float noise, float wander,
                  uint32_t seed)
{
    s_lcg = seed;
    float f = bpm / 60.0f;
    for (int i = 0; i < N; i++)
    {
        float t = (float)i / (float)HR_AUTOCORR_FS;
        float v = sinf(2.0f * 3.14159265f * f * t)
                + h2 * sinf(2.0f * 3.14159265f * 2.0f * f * t + 0.9f);
        v += wander * sinf(2.0f * 3.14159265f * 0.05f * t);
        v += noise * (frnd() - 0.5f);
        out[i] = (uint32_t)(60000.0f + 800.0f * v);
    }
}

struct hrac_case
{
    const char *name;
    int   bpm;          /* 0 = pure noise, must be refused */
    float h2, noise, wander;
};

static const struct hrac_case CASES[] =
{
    {"clean, weak 2nd",              55, 0.30f, 0.05f, 0.0f},
    {"STRONG 2nd (1.4x)",            55, 1.40f, 0.05f, 0.0f},
    {"strong 2nd (2.5x)",            55, 2.50f, 0.10f, 0.0f},
    {"strong 2nd+noise+wander",      58, 1.80f, 0.60f, 1.5f},
    {"low HR 47 + strong 2nd",       47, 1.60f, 0.10f, 0.5f},
    {"very strong 2nd (4x)",         52, 4.00f, 0.20f, 0.5f},
    {"harmonic at 61 (08-04)",       61, 2.00f, 0.40f, 1.0f},
    {"harmonic 58 clean",            58, 2.20f, 0.05f, 0.0f},
    {"harmonic 54, h2=3",            54, 3.00f, 0.30f, 0.8f},
    {"REAL 110 exercise",           110, 0.40f, 0.10f, 0.0f},
    {"REAL 110 clean",              110, 0.30f, 0.05f, 0.0f},
    {"REAL 118 must not halve",     118, 0.30f, 0.20f, 0.5f},
    {"REAL 95",                      95, 0.50f, 0.30f, 0.8f},
    {"REAL 130",                    130, 0.35f, 0.25f, 0.4f},
    {"REAL 100",                    100, 0.40f, 0.20f, 0.4f},
    {"REAL 168 max",                168, 0.30f, 0.20f, 0.2f},
    {"REAL 62 quiet",                62, 0.35f, 0.15f, 0.3f},
    {"REAL 48 deep sleep",           48, 0.45f, 0.15f, 0.3f},
    {"REAL 73",                      73, 0.40f, 0.25f, 0.6f},
    {"REAL 86",                      86, 0.45f, 0.20f, 0.4f},
    {"REAL 52 clean",                52, 0.35f, 0.08f, 0.2f},
    {"noise only #1",                 0, 0.0f,  1.0f,  0.0f},
    {"noise only #2",                 0, 0.0f,  1.0f,  0.0f},
    {"noise only #3",                 0, 0.0f,  1.0f,  0.0f},
};

static int hr_ac_test(int argc, char **argv)
{
    (void)argc; (void)argv;
    static uint32_t buf[N];
    int bad = 0;
    const int n = (int)(sizeof(CASES) / sizeof(CASES[0]));

    rt_kprintf("%-26s %6s %10s %s\n", "case", "true", "est", "verdict");
    for (int k = 0; k < n; k++)
    {
        const struct hrac_case *c = &CASES[k];
        synth(buf, c->bpm ? (float)c->bpm : 0.0f, c->h2, c->noise, c->wander,
              (uint32_t)(k + 1) * 2654435761u);

        hr_autocorr_reset();
        /* Feed the way the driver does — in FIFO-sized chunks, so the ring's
           wrap handling is exercised rather than bypassed by one big write. */
        for (int i = 0; i < N; i += 2)
            hr_autocorr_feed(2, &buf[i]);

        uint8_t conf = 0;
        uint8_t est = hr_autocorr_estimate(&conf);

        int ok;
        if (c->bpm == 0)
            ok = (est == 0) || (conf < 50);      /* must refuse pure noise */
        else
        {
            int d = (int)est - c->bpm;
            ok = (est != 0) && (d <= 5) && (d >= -5);
        }
        if (!ok) bad++;
        rt_kprintf("%-26s %6d %6d(c%2d) %s\n", c->name, c->bpm, est, conf,
                   ok ? "OK" : "*** WRONG ***");
    }
    rt_kprintf("\nhr_autocorr: %d/%d wrong\n", bad, n);
    return bad;
}
MSH_CMD_EXPORT(hr_ac_test, validate hr_autocorr against synthetic PPG cases);

#endif /* BSP_USING_PC_SIMULATOR */
