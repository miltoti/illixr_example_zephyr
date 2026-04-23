#ifndef OPENVINS_SLAM_MATH_HPP
#define OPENVINS_SLAM_MATH_HPP

/*
 * OpenVINS SLAM - MSCKF VIO Implementation for RTOS
 * Minimal OpenCV: only core + imgproc
 * No features2d, no video/tracking needed
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

namespace OpenVINS {

// ==============================================================================
// CONFIGURATION
// ==============================================================================
struct VIOConfig {
    struct CameraIntrinsics {
        double fx, fy, cx, cy;
        double k1, k2, p1, p2;
    };
    
    CameraIntrinsics cam0, cam1;
    Eigen::Matrix3d R_ItoC0, R_ItoC1;
    Eigen::Vector3d p_C0inI, p_C1inI;
    
    double sigma_gyro = 0.00016968;
    double sigma_accel = 0.002;
    double sigma_gyro_bias = 1.9393e-05;
    double sigma_accel_bias = 0.003;
    double sigma_pixel = 1.0;
    
    Eigen::Vector3d gravity{0, 0, -9.81};
    
    int max_features = 150;
    int min_features = 50;
    int max_clone_size = 11;
    int min_track_length = 3;
    double max_reprojection_error = 2.0;
    
    // Template matching params
    int template_size = 15;
    int search_radius = 20;          // temporal (frame-to-frame) NCC search radius
    int stereo_search_radius = 60;   // stereo (left→right) NCC search radius
                                     // EuRoC baseline 0.11m, fx≈458 → disparity≈50px at 1m
    double match_threshold = 0.8;
    double fb_check_thresh = 2.0;    // forward-backward consistency threshold (pixels)
};

// ==============================================================================
// STATE STRUCTURES
// ==============================================================================
struct CameraClone {
    double timestamp;
    Eigen::Quaterniond q_GtoC;
    Eigen::Vector3d p_CinG;
};

struct IMUState {
    double timestamp;
    Eigen::Quaterniond q_GtoI;
    Eigen::Vector3d p_IinG;
    Eigen::Vector3d v_IinG;
    Eigen::Vector3d b_gyro;
    Eigen::Vector3d b_accel;
    Eigen::Matrix<double, 15, 15> P_imu;
    std::map<double, CameraClone> clones;
    Eigen::MatrixXd P_full;
    
    IMUState() {
        timestamp = 0;
        q_GtoI.setIdentity();
        p_IinG.setZero();
        v_IinG.setZero();
        b_gyro.setZero();
        b_accel.setZero();
        P_imu.setIdentity();
        P_imu *= 0.01;
        P_full = P_imu;
    }
    
    int state_dim() const { return 15 + 6 * clones.size(); }
};

struct Feature {
    size_t id;
    std::map<double, std::pair<Eigen::Vector2d, Eigen::Vector2d>> observations;
    Eigen::Vector3d p_FinG;
    bool triangulated = false;
    bool should_marginalize = false;
};

// ==============================================================================
// HELPER FUNCTIONS
// ==============================================================================
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return m;
}

inline Eigen::Quaterniond delta_q(const Eigen::Vector3d& theta) {
    double norm = theta.norm();
    if (norm < 1e-8) {
        return Eigen::Quaterniond(1, theta(0)/2, theta(1)/2, theta(2)/2);
    }
    double c = cos(norm / 2);
    double s = sin(norm / 2);
    Eigen::Vector3d axis = theta / norm;
    return Eigen::Quaterniond(c, s*axis(0), s*axis(1), s*axis(2));
}

inline Eigen::Vector2d project_point(const Eigen::Vector3d& p_FinC,
                                     const VIOConfig::CameraIntrinsics& cam) {
    if (p_FinC(2) <= 0) return Eigen::Vector2d(-1, -1);
    
    double x = p_FinC(0) / p_FinC(2);
    double y = p_FinC(1) / p_FinC(2);
    
    double r2 = x*x + y*y;
    double radial = 1 + cam.k1*r2 + cam.k2*r2*r2;
    double dx = 2*cam.p1*x*y + cam.p2*(r2 + 2*x*x);
    double dy = cam.p1*(r2 + 2*y*y) + 2*cam.p2*x*y;
    
    x = x*radial + dx;
    y = y*radial + dy;
    
    return Eigen::Vector2d(cam.fx * x + cam.cx, cam.fy * y + cam.cy);
}

inline Eigen::Vector2d undistort_point(const Eigen::Vector2d& uv,
                                       const VIOConfig::CameraIntrinsics& cam) {
    double x = (uv(0) - cam.cx) / cam.fx;
    double y = (uv(1) - cam.cy) / cam.fy;
    
    for (int i = 0; i < 5; i++) {
        double r2 = x*x + y*y;
        double radial = 1 + cam.k1*r2 + cam.k2*r2*r2;
        double dx = 2*cam.p1*x*y + cam.p2*(r2 + 2*x*x);
        double dy = cam.p1*(r2 + 2*y*y) + 2*cam.p2*x*y;
        
        x = ((uv(0) - cam.cx) / cam.fx - dx) / radial;
        y = ((uv(1) - cam.cy) / cam.fy - dy) / radial;
    }
    
    return Eigen::Vector2d(x, y);
}

// ==============================================================================
// TEMPLATE MATCHING (Replaces KLT)
// ==============================================================================
inline double ncc_match(const cv::Mat& img1, const cv::Mat& img2,
                       int x1, int y1, int x2, int y2, int template_size) {
    int half = template_size / 2;
    
    if (x1 - half < 0 || x1 + half >= img1.cols ||
        y1 - half < 0 || y1 + half >= img1.rows ||
        x2 - half < 0 || x2 + half >= img2.cols ||
        y2 - half < 0 || y2 + half >= img2.rows) {
        return -1.0;
    }
    
    cv::Mat templ1 = img1(cv::Rect(x1 - half, y1 - half, template_size, template_size));
    cv::Mat templ2 = img2(cv::Rect(x2 - half, y2 - half, template_size, template_size));
    
    double sum1 = 0, sum2 = 0, sum12 = 0, sum11 = 0, sum22 = 0;
    int count = template_size * template_size;
    
    for (int dy = 0; dy < template_size; dy++) {
        const uint8_t* row1 = templ1.ptr<uint8_t>(dy);
        const uint8_t* row2 = templ2.ptr<uint8_t>(dy);
        for (int dx = 0; dx < template_size; dx++) {
            double v1 = row1[dx];
            double v2 = row2[dx];
            sum1 += v1;
            sum2 += v2;
            sum12 += v1 * v2;
            sum11 += v1 * v1;
            sum22 += v2 * v2;
        }
    }
    
    double mean1 = sum1 / count;
    double mean2 = sum2 / count;
    double var1 = sum11 / count - mean1 * mean1;
    double var2 = sum22 / count - mean2 * mean2;
    double covar = sum12 / count - mean1 * mean2;
    
    if (var1 < 1e-6 || var2 < 1e-6) return -1.0;
    
    return covar / sqrt(var1 * var2);
}

inline bool track_point_template(const cv::Mat& prev_img, const cv::Mat& curr_img,
                                 const cv::Point2f& prev_pt, cv::Point2f& curr_pt,
                                 int template_size, int search_radius, double threshold) {
    int x0 = (int)prev_pt.x;
    int y0 = (int)prev_pt.y;
    
    double best_score = -1.0;
    int best_x = x0, best_y = y0;
    
    for (int dy = -search_radius; dy <= search_radius; dy++) {
        for (int dx = -search_radius; dx <= search_radius; dx++) {
            int x = x0 + dx;
            int y = y0 + dy;
            
            double score = ncc_match(prev_img, curr_img, x0, y0, x, y, template_size);
            
            if (score > best_score) {
                best_score = score;
                best_x = x;
                best_y = y;
            }
        }
    }
    
    if (best_score < threshold) return false;
    
    curr_pt.x = best_x;
    curr_pt.y = best_y;
    return true;
}

// Search for a stereo match along the epipolar line in img_right.
// l = F * [x0, y0, 1]^T  is the epipolar line ax+by+c=0 in the right image.
// We step along the line from disparity -disp_max to +disp_max (in x),
// evaluating NCC against the template extracted from img_left at (x0,y0).
// This replaces the O(r²) box search with an O(r) line search.
inline bool track_stereo_epipolar(const cv::Mat& img_left, const cv::Mat& img_right,
                                  const cv::Point2f& pt_left, cv::Point2f& pt_right,
                                  const Eigen::Matrix3d& F,
                                  int template_size, int disp_max, double threshold) {
    int x0 = (int)pt_left.x;
    int y0 = (int)pt_left.y;
    int half = template_size / 2;

    // Epipolar line in right image: [a, b, c] = F * [x0, y0, 1]
    Eigen::Vector3d l = F * Eigen::Vector3d(x0, y0, 1.0);
    double a = l(0), b = l(1), c = l(2);
    double denom = std::sqrt(a*a + b*b);
    if (denom < 1e-9) return false;

    double best_score = -1.0;
    int best_x = x0, best_y = y0;

    // Step along x from x0-disp_max to x0+disp_max, compute y from line eq
    for (int dx = -disp_max; dx <= disp_max; dx++) {
        int xi = x0 + dx;
        int yi = static_cast<int>(std::round(-(a * xi + c) / b));

        if (xi - half < 0 || xi + half >= img_right.cols ||
            yi - half < 0 || yi + half >= img_right.rows) continue;

        double score = ncc_match(img_left, img_right, x0, y0, xi, yi, template_size);
        if (score > best_score) {
            best_score = score;
            best_x = xi;
            best_y = yi;
        }
    }

    if (best_score < threshold) return false;
    pt_right.x = best_x;
    pt_right.y = best_y;
    return true;
}

// ==============================================================================
// MSCKF ESTIMATOR
// ==============================================================================
class MSCKFEstimator {
public:
    MSCKFEstimator(const VIOConfig& config)
        : config_(config), initialized_(false), feature_id_counter_(0) {
        // Pre-compute stereo fundamental matrix F = K1^{-T} E K0^{-1}
        // where E = [t_10]× R_10, R_10/t_10 are the relative pose from cam0 to cam1.
        Eigen::Matrix3d R_ItoC0 = config_.R_ItoC0;
        Eigen::Matrix3d R_ItoC1 = config_.R_ItoC1;
        Eigen::Vector3d p_C0inI = config_.p_C0inI;
        Eigen::Vector3d p_C1inI = config_.p_C1inI;

        // Pose of cam1 relative to cam0
        Eigen::Matrix3d R_C0toI = R_ItoC0.transpose();
        Eigen::Matrix3d R_10    = R_ItoC1 * R_C0toI;          // R from cam0 to cam1
        Eigen::Vector3d t_10    = R_ItoC1 * (p_C0inI - p_C1inI); // translation in cam1 frame

        Eigen::Matrix3d E = skew(t_10) * R_10;

        // K0^{-1} and K1^{-T}
        Eigen::Matrix3d K0_inv, K1_invT;
        K0_inv << 1.0/config_.cam0.fx, 0, -config_.cam0.cx/config_.cam0.fx,
                  0, 1.0/config_.cam0.fy, -config_.cam0.cy/config_.cam0.fy,
                  0, 0, 1;
        K1_invT << 1.0/config_.cam1.fx, 0, -config_.cam1.cx/config_.cam1.fx,
                   0, 1.0/config_.cam1.fy, -config_.cam1.cy/config_.cam1.fy,
                   0, 0, 1;
        F_stereo_ = K1_invT.transpose() * E * K0_inv;
    }
    
    void feed_imu(double timestamp, const Eigen::Vector3d& w, const Eigen::Vector3d& a);
    void feed_stereo(double timestamp, const cv::Mat& img0, const cv::Mat& img1);
    
    bool is_initialized() const { return initialized_; }
    const IMUState& get_state() const { return state_; }
    
private:
    VIOConfig config_;
    IMUState state_;
    bool initialized_;
    
    struct IMUMeasurement { double timestamp; Eigen::Vector3d w, a; };
    std::vector<IMUMeasurement> imu_buffer_;
    
    cv::Mat prev_img0_, prev_img1_;
    std::map<size_t, Feature> feature_tracks_;
    size_t feature_id_counter_;
    Eigen::Matrix3d F_stereo_;   // pre-computed fundamental matrix cam0→cam1
    
    void propagate_imu(double timestamp, const Eigen::Vector3d& w_m, const Eigen::Vector3d& a_m);
    bool try_initialize(double timestamp);
    void track_features(double timestamp, const cv::Mat& img0, const cv::Mat& img1);
    double epipolar_distance(const cv::Point2f& p0, const cv::Point2f& p1) const;
    void augment_state(double timestamp);
    bool triangulate_feature(Feature& feat);
    void msckf_update(std::vector<Feature*>& features);
    void marginalize_oldest_clone();
};

} // namespace OpenVINS

#endif // OPENVINS_SLAM_MATH_HPP
