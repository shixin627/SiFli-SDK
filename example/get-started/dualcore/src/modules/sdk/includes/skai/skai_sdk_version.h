/*
 * Skai SDK — external API version (ADR-0019).
 *
 * This is the number an app's manifest negotiates against ("skai": ">=1.2").
 * It versions the EXTERNAL contract only. The internal C layer is API-not-ABI
 * and is free to be refactored without touching these.
 *
 * MINOR bumps when a capability is ADDED. Adding is the only non-breaking
 * change there is; existing capabilities never change meaning.
 * MAJOR bumps only for a breaking change, which also means keeping a compat
 * projection for the old major — so treat a MAJOR bump as a project, not a
 * commit.
 */
#ifndef SKAI_SDK_VERSION_H
#define SKAI_SDK_VERSION_H

#define SKAI_API_MAJOR  1
#define SKAI_API_MINOR  0

/* True when firmware satisfies an app manifest's ">=maj.min". */
#define SKAI_API_SATISFIES(maj, min) \
    ((maj) == SKAI_API_MAJOR && (min) <= SKAI_API_MINOR)

#endif /* SKAI_SDK_VERSION_H */
