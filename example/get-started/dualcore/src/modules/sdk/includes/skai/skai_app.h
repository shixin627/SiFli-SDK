/*
 * Skai SDK — an app's control over its own lifetime.
 */
#ifndef SKAI_APP_H
#define SKAI_APP_H

#include <stdbool.h>

#include "skai_export.h"

/* Close this app and go back. T1 with no argument on purpose: an app can only
 * ever close ITSELF, which is strictly less power than staying open, and there
 * is no id to pass that could name someone else's app.
 *
 * Tap-to-dismiss is how several of the built-in screens end, so without this a
 * reproduction can draw the screen and then trap the user on it. */
SKAI_EXPORT("app.exit", SKAI_T1, SKAI_THREAD_LVGL)
bool skai_app_exit(void);

#endif /* SKAI_APP_H */
