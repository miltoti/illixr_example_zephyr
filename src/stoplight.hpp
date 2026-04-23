#pragma once

#include <zephyr/kernel.h>

// ==============================================================================
// STOPLIGHT — flow control between producers (IMU, camera) and consumer (openvins)
//
// Protocol:
//   1. IMU + camera block on their respective semaphores at the START of each window/frame
//   2. Openvins gives both semaphores after finishing feed_stereo() + update
//   3. This ensures:
//      - No data is produced faster than openvins can consume
//      - No heap exhaustion from queued cloned images
//      - IMU and camera stay in lockstep with each other
//
// Initial count = 1 so the first window/frame can proceed without waiting.
// ==============================================================================

// IMU takes this before each window of 10 samples.
// Openvins gives it after processing the corresponding camera frame.
extern struct k_sem stoplight_imu;

// Camera takes this before each frame.
// Openvins gives it after processing the frame.
extern struct k_sem stoplight_cam;

extern struct k_sem stoplight_ready;