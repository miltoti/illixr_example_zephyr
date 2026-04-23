#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <cstdio>
#include <cmath>
#include <vector>

#ifdef EMPTY
#undef EMPTY
#endif

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// ---------------- helpers ----------------
static inline double deg2rad(double d) { return d * M_PI / 180.0; }
static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

static bool approx_abs(double a, double b, double tol) { return std::abs(a - b) <= tol; }

static void passfail(const char* name, bool ok) {
    printk("  %s: %s\n", name, ok ? "PASSED" : "FAILED");
}

static cv::Mat make_synthetic_image(int H, int W) {
    cv::Mat img(H, W, CV_8UC1, cv::Scalar(0));

    // High-frequency + corners: dots, boxes, lines (good for FFT + phase correlation)
    for (int y = 30; y < H; y += 50) {
        for (int x = 30; x < W; x += 50) {
            cv::circle(img, cv::Point(x, y), 2, cv::Scalar(255), -1);
        }
    }
    cv::rectangle(img, cv::Rect(W/6, H/5, W/4, H/6), cv::Scalar(200), -1);
    cv::rectangle(img, cv::Rect(W/2, H/2, W/3, H/3), cv::Scalar(120), 2);
    cv::line(img, cv::Point(10, H-20), cv::Point(W-10, H-60), cv::Scalar(180), 1);

    // Add a mild blur so it’s not too “alias-y”
    cv::GaussianBlur(img, img, cv::Size(3,3), 0.6);

    return img;
}

static cv::Mat apply_motion(const cv::Mat& src, double rot_deg, double dx, double dy) {
    cv::Point2f c((float)src.cols * 0.5f, (float)src.rows * 0.5f);
    cv::Mat M = cv::getRotationMatrix2D(c, rot_deg, 1.0);
    M.at<double>(0,2) += dx;
    M.at<double>(1,2) += dy;

    cv::Mat dst;
    cv::warpAffine(src, dst, M, src.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    return dst;
}

// Pretend IMU gyro: constant angular velocity with small bias/noise, integrated over dt
static double integrate_fake_gyro(double theta_true_deg, double dt, int N, double gyro_bias_deg_s) {
    // constant omega that yields theta_true over N samples
    double T = dt * N;
    double omega_true = theta_true_deg / T; // deg/s

    double theta = 0.0;
    for (int i = 0; i < N; i++) {
        double omega_meas = omega_true + gyro_bias_deg_s; // omit random noise for determinism
        theta += omega_meas * dt;
    }
    return theta; // deg
}

// Build FFT-magnitude image for rotation estimation (Fourier-Mellin style)
static cv::Mat fft_mag32f(const cv::Mat& gray_u8) {
    cv::Mat f;
    gray_u8.convertTo(f, CV_32F);

    // Hanning window helps phase correlation stability
    cv::Mat win;
    cv::createHanningWindow(win, f.size(), CV_32F);
    f = f.mul(win);

    // DFT complex output
    cv::Mat complex;
    cv::dft(f, complex, cv::DFT_COMPLEX_OUTPUT);

    // magnitude of complex spectrum
    std::vector<cv::Mat> ch(2);
    cv::split(complex, ch);
    cv::Mat mag;
    cv::magnitude(ch[0], ch[1], mag);

    // log(1+mag) for dynamic range compression
    mag += 1.0f;
    cv::log(mag, mag);

    // shift so DC is in center (better for polar transform)
    int cx = mag.cols / 2;
    int cy = mag.rows / 2;
    cv::Mat q0(mag, cv::Rect(0, 0, cx, cy));
    cv::Mat q1(mag, cv::Rect(cx, 0, cx, cy));
    cv::Mat q2(mag, cv::Rect(0, cy, cx, cy));
    cv::Mat q3(mag, cv::Rect(cx, cy, cx, cy));
    cv::Mat tmp;
    q0.copyTo(tmp); q3.copyTo(q0); tmp.copyTo(q3);
    q1.copyTo(tmp); q2.copyTo(q1); tmp.copyTo(q2);

    // normalize
    cv::normalize(mag, mag, 0.0f, 1.0f, cv::NORM_MINMAX);
    return mag;
}

// Estimate rotation (deg) between two images using:
// FFT magnitude -> warpPolar(log-polar) -> phaseCorrelate
static double estimate_rotation_deg(const cv::Mat& img1_u8, const cv::Mat& img2_u8) {
    cv::Mat m1 = fft_mag32f(img1_u8);
    cv::Mat m2 = fft_mag32f(img2_u8);

    // Log-polar mapping around center:
    // angle becomes one axis; rotation becomes translation in that axis
    cv::Point2f center((float)m1.cols * 0.5f, (float)m1.rows * 0.5f);
    double maxRadius = std::min(center.x, center.y);

    // Using warpPolar (log-polar)
    cv::Mat lp1, lp2;
    cv::warpPolar(m1, lp1, m1.size(), center, maxRadius,
                  cv::WARP_POLAR_LOG | cv::INTER_LINEAR | cv::WARP_FILL_OUTLIERS);
    cv::warpPolar(m2, lp2, m2.size(), center, maxRadius,
                  cv::WARP_POLAR_LOG | cv::INTER_LINEAR | cv::WARP_FILL_OUTLIERS);

    // Phase correlation gives shift between lp images
    cv::Point2d shift = cv::phaseCorrelate(lp1, lp2);

    // In warpPolar, vertical axis typically corresponds to angle (depends on implementation),
    // but for OpenCV's warpPolar, angle maps to rows (y) and radius maps to cols (x).
    // So rotation is shift.y scaled to 360 degrees.
    double rot_est = (shift.y * 360.0) / (double)lp1.rows;

    // Wrap to [-180, 180]
    while (rot_est > 180.0) rot_est -= 360.0;
    while (rot_est < -180.0) rot_est += 360.0;

    return rot_est;
}

static cv::Mat rotate_only(const cv::Mat& src, double rot_deg) {
    cv::Point2f c((float)src.cols * 0.5f, (float)src.rows * 0.5f);
    cv::Mat M = cv::getRotationMatrix2D(c, rot_deg, 1.0);
    cv::Mat dst;
    cv::warpAffine(src, dst, M, src.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    return dst;
}

int main() {
    k_sleep(K_MSEC(300));

    const int H = 256, W = 256;

    // Ground truth motion between frames
    const double theta_true_deg = 7.0;   // rotation
    const double dx_true = 6.5;          // translation x (pixels)
    const double dy_true = -4.0;         // translation y (pixels)

    // Generate “pixels”
    cv::Mat img1 = make_synthetic_image(H, W);
    cv::Mat img2 = apply_motion(img1, theta_true_deg, dx_true, dy_true);

    // Fake IMU gyro samples between img1 and img2
    // (If you want “IMU wrong”, bump bias)
    const double dt = 0.002;    // 500 Hz
    const int N = 200;          // 0.4 s
    const double gyro_bias_deg_s = 0.02; // small bias
    const double theta_imu_deg = integrate_fake_gyro(theta_true_deg, dt, N, gyro_bias_deg_s);

    printf("GT rot=%.4f deg, IMU rot=%.4f deg (bias %.4f deg/s)\n",
           theta_true_deg, theta_imu_deg, gyro_bias_deg_s);

    // ------------------------------------------------------------
    // Test A: OpenCV estimates rotation from images (FFT/log-polar/phase)
    // ------------------------------------------------------------
    double theta_cv_deg = estimate_rotation_deg(img1, img2);
    double rot_err = std::abs(theta_cv_deg - theta_true_deg);
    // handle wrap-around
    rot_err = std::min(rot_err, std::abs((theta_cv_deg + 360.0) - theta_true_deg));
    rot_err = std::min(rot_err, std::abs((theta_cv_deg - 360.0) - theta_true_deg));

    printf("\nRotation from images:\n");
    printf("  theta_cv=%.4f deg, err=%.4f deg\n", theta_cv_deg, rot_err);

    bool ok_rot = rot_err < 1.0; // tolerant, FFT method can vary a bit on embedded
    passfail("ROTATION EST (image-only)", ok_rot);

    // ------------------------------------------------------------
    // Test B: Use IMU to derotate, then estimate translation via phaseCorrelate
    // ------------------------------------------------------------
    // Derotate img2 by -theta_imu (undo rotation)
    // NOTE: img2 also has translation, but phaseCorrelate will estimate it.
    cv::Mat img2_derot = rotate_only(img2, -theta_imu_deg);

    // For phaseCorrelate, float images work best
    cv::Mat f1, f2;
    img1.convertTo(f1, CV_32F);
    img2_derot.convertTo(f2, CV_32F);

    // Hanning window improves stability
    cv::Mat win;
    cv::createHanningWindow(win, f1.size(), CV_32F);
    f1 = f1.mul(win);
    f2 = f2.mul(win);

    cv::Point2d shift = cv::phaseCorrelate(f1, f2);
    // phaseCorrelate returns shift to align src2 to src1 (depending on call order).
    // Here: shift estimates how much f2 must shift to match f1, so it should be approx (-dx, -dy)
    double dx_est = shift.x;
    double dy_est = shift.y;

    printf("\nTranslation from phaseCorrelate after IMU derotation:\n");
    printf("  est shift = (%.3f, %.3f) px\n", dx_est, dy_est);

    bool ok_dx = approx_abs(dx_est, -dx_true, 1.5); // allow error due to derotation bias
    bool ok_dy = approx_abs(dy_est, -dy_true, 1.5);

    // ------------------------------------------------------------
    // Test C: End-to-end alignment quality (RMS pixel error)
    // ------------------------------------------------------------
    // Apply estimated translation to f2 and compare
    cv::Mat M = (cv::Mat_<double>(2,3) << 1, 0, dx_est, 0, 1, dy_est);
    cv::Mat f2_aligned;
    cv::warpAffine(f2, f2_aligned, M, f2.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

    cv::Mat diff = f1 - f2_aligned;
    cv::Scalar mse = cv::mean(diff.mul(diff));
    double rmse = std::sqrt(mse[0]);

    printf("\nAlignment RMSE (lower is better): rmse=%.6f\n", rmse);
    bool ok_rmse = rmse < 8.0; // depends on content; should be comfortably small if OpenCV math works

    printk("\n=== DONE ===\n");
    while (1) {
        k_sleep(K_SECONDS(1));
    }
    return 0;
}
