#pragma once
#include <zephyr/kernel.h>

// ============================================================================
// Shared message queues (defined in openvins/plugin.cpp via K_MSGQ_DEFINE)
//
// Both carry POINTERS to avoid Eigen alignment faults from k_msgq memcpy.
// Producers push directly, consumer (openvins) pulls directly.
// ============================================================================
extern struct k_msgq openvins_imu_queue;   // carries ImuMsg*
extern struct k_msgq openvins_cam_queue;   // carries CamMsg*openvins_queues