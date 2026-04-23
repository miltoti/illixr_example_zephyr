#pragma once

// --- Kill common macro collisions (Zephyr/newlib/etc.) ---
#ifdef fmt
#undef fmt
#endif
#ifdef FileNode
#undef FileNode
#endif
#ifdef KeyPoint
#undef KeyPoint
#endif
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#ifdef EMPTY
#undef EMPTY
#endif

// --- Now include OpenCV immediately, BEFORE anything can redefine macros again ---
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// If you use these:
#include <opencv2/core/mat.hpp>