#pragma once
#include <zephyr/kernel.h>

// ============================================================================
// Stoplights for producer → consumer flow control
//
// Both start at count 0 so producers block immediately.
// OpenVINS (the consumer) gives each semaphore exactly once per cycle.
// Max count = 1 so a double-give is harmlessly dropped.
// ============================================================================
K_SEM_DEFINE(stoplight_imu, 0, 1);
K_SEM_DEFINE(stoplight_cam, 0, 1);
K_SEM_DEFINE(stoplight_ready, 0, 20);  // max=20 matches MAX_REGISTERED_PLUGINS