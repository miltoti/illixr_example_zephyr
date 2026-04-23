#include "SLAMMath.hpp"

namespace OpenVINS {

// ==============================================================================
// IMU PROPAGATION
// ==============================================================================
void MSCKFEstimator::propagate_imu(double timestamp, const Eigen::Vector3d& w_m, const Eigen::Vector3d& a_m) {
    double dt = timestamp - state_.timestamp;
    if (dt <= 0 || dt > 0.5) return;

    Eigen::Vector3d w = w_m - state_.b_gyro;
    Eigen::Vector3d a = a_m - state_.b_accel;

    // ── RK4 integration for position and velocity ─────────────────────────────
    // Kinematic equation: a_true_global = R_ItoG * a_measured + g_global
    // where g_global = {0,0,-9.81} and a_measured includes the gravity reaction
    // force (~+9.81 on z when stationary and level).
    // config_.gravity = {0, 0, -9.81}, so the correct form is + config_.gravity.
    Eigen::Matrix3d R1 = state_.q_GtoI.toRotationMatrix();
    Eigen::Vector3d k1_v = R1.transpose() * a + config_.gravity;
    Eigen::Vector3d k1_p = state_.v_IinG;

    // k2: midpoint rotation from initial state
    Eigen::Quaterniond q2 = state_.q_GtoI * delta_q(w * dt / 2);
    Eigen::Matrix3d    R2 = q2.toRotationMatrix();
    Eigen::Vector3d    v2 = state_.v_IinG + k1_v * dt / 2;
    Eigen::Vector3d k2_v  = R2.transpose() * a + config_.gravity;
    Eigen::Vector3d k2_p  = v2;

    // k3: midpoint rotation from q2 (FIX: original used state_.q_GtoI again)
    Eigen::Quaterniond q3 = q2 * delta_q(w * dt / 2);
    Eigen::Matrix3d    R3 = q3.toRotationMatrix();
    Eigen::Vector3d    v3 = state_.v_IinG + k2_v * dt / 2;
    Eigen::Vector3d k3_v  = R3.transpose() * a + config_.gravity;
    Eigen::Vector3d k3_p  = v3;

    // k4: full-step rotation from initial state
    Eigen::Quaterniond q4 = state_.q_GtoI * delta_q(w * dt);
    Eigen::Matrix3d    R4 = q4.toRotationMatrix();
    Eigen::Vector3d    v4 = state_.v_IinG + k3_v * dt;
    Eigen::Vector3d k4_v  = R4.transpose() * a + config_.gravity;
    Eigen::Vector3d k4_p  = v4;

    state_.v_IinG += (k1_v + 2 * k2_v + 2 * k3_v + k4_v) * dt / 6.0;
    state_.p_IinG += (k1_p + 2 * k2_p + 2 * k3_p + k4_p) * dt / 6.0;
    state_.q_GtoI  = state_.q_GtoI * delta_q(w * dt);
    state_.q_GtoI.normalize();

    // ── Linearised covariance propagation ────────────────────────────────────
    // Error state ordering: [δθ(0:3), δp(3:6), δv(6:9), δbg(9:12), δba(12:15)]
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();

    // δθ_dot = -[w]× δθ - δbg
    F.block<3,3>(0, 0)  = -skew(w);
    F.block<3,3>(0, 9)  = -Eigen::Matrix3d::Identity();   // gyro-bias → attitude

    // δp_dot = δv
    F.block<3,3>(3, 6)  =  Eigen::Matrix3d::Identity();

    // δv_dot = -R_ItoG [a]× δθ - R_ItoG δba
    F.block<3,3>(6, 0)  = -R1.transpose() * skew(a);
    F.block<3,3>(6, 12) = -R1.transpose();                // accel-bias → velocity

    // Discrete transition: Phi ≈ I + F*dt
    Eigen::Matrix<double, 15, 15> Phi =
        Eigen::Matrix<double, 15, 15>::Identity() + F * dt;

    // Process noise (continuous-to-discrete, first-order Van Loan)
    Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
    Q.block<3,3>(0,  0)  = Eigen::Matrix3d::Identity() * config_.sigma_gyro       * config_.sigma_gyro       * dt;
    Q.block<3,3>(6,  6)  = Eigen::Matrix3d::Identity() * config_.sigma_accel      * config_.sigma_accel      * dt;
    Q.block<3,3>(9,  9)  = Eigen::Matrix3d::Identity() * config_.sigma_gyro_bias  * config_.sigma_gyro_bias  * dt;
    Q.block<3,3>(12, 12) = Eigen::Matrix3d::Identity() * config_.sigma_accel_bias * config_.sigma_accel_bias * dt;

    // Embed into full state — clones are frozen during IMU propagation
    int full_dim = state_.state_dim();
    Eigen::MatrixXd Phi_full = Eigen::MatrixXd::Identity(full_dim, full_dim);
    Phi_full.block<15,15>(0, 0) = Phi;

    Eigen::MatrixXd Q_full = Eigen::MatrixXd::Zero(full_dim, full_dim);
    Q_full.block<15,15>(0, 0) = Q;

    state_.P_full    = Phi_full * state_.P_full * Phi_full.transpose() + Q_full;
    state_.P_imu     = state_.P_full.block<15,15>(0, 0);
    state_.timestamp = timestamp;
}

// ==============================================================================
// INITIALIZATION
// ==============================================================================
bool MSCKFEstimator::try_initialize(double timestamp) {
    if (imu_buffer_.size() < 20) return false;

    // Mean accelerometer reading over the static window
    Eigen::Vector3d a_avg = Eigen::Vector3d::Zero();
    for (const auto& m : imu_buffer_) a_avg += m.a;
    a_avg /= static_cast<double>(imu_buffer_.size());

    // Reject if platform is not stationary
    double accel_var = 0;
    for (const auto& m : imu_buffer_) accel_var += (m.a - a_avg).squaredNorm();
    accel_var /= static_cast<double>(imu_buffer_.size());
    if (std::sqrt(accel_var) > 1.5) return false;

    // ── Gravity alignment via Rodrigues rotation formula ──────────────────────
    // We want q_GtoI such that R_GtoI * {0,0,1} = a_avg.normalized().
    // Reasoning: at rest, a_measured = R_GtoI * (-g_global) = R_GtoI * {0,0,+9.81}.
    // So R_GtoI maps global-z to a_avg direction.
    //
    // FIX: original code used z_I = -a_avg.normalized(), which computed a 180°
    // inverted rotation. That caused R_ItoG to flip the accelerometer reading,
    // putting +9.81 and gravity {0,0,-9.81} in the same direction → 2g downward.
    // Correct form is z_I = +a_avg.normalized().
    Eigen::Vector3d z_G(0, 0, 1);
    Eigen::Vector3d z_I = a_avg.normalized();   // FIX: was -a_avg.normalized()
    Eigen::Vector3d v   = z_G.cross(z_I);
    double          c   = z_G.dot(z_I);

    if (c < -0.99) {
        // 180° degenerate case: rotate around x-axis
        state_.q_GtoI = Eigen::Quaterniond(0, 1, 0, 0);
    } else {
        double s = v.norm();
        Eigen::Matrix3d vx = skew(v);
        Eigen::Matrix3d R  = Eigen::Matrix3d::Identity() + vx + vx * vx * ((1.0 - c) / (s * s));
        state_.q_GtoI = Eigen::Quaterniond(R);
    }
    state_.q_GtoI.normalize();

    state_.p_IinG.setZero();
    state_.v_IinG.setZero();
    state_.b_gyro.setZero();
    state_.b_accel.setZero();
    state_.timestamp = timestamp;

    state_.P_imu.setIdentity();
    state_.P_imu *= 0.01;
    state_.P_full = state_.P_imu;

    imu_buffer_.clear();
    return true;
}

// ==============================================================================
// FEATURE TRACKING
// ==============================================================================
// Returns the Sampson epipolar distance for a stereo correspondence.
// p0/p1 are distorted pixel coordinates.
double MSCKFEstimator::epipolar_distance(const cv::Point2f& p0,
                                          const cv::Point2f& p1) const {
    Eigen::Vector3d x0(p0.x, p0.y, 1.0);
    Eigen::Vector3d x1(p1.x, p1.y, 1.0);
    Eigen::Vector3d Fx0  = F_stereo_ * x0;
    Eigen::Vector3d FtX1 = F_stereo_.transpose() * x1;
    double num  = x1.dot(Fx0);
    double den  = Fx0(0)*Fx0(0) + Fx0(1)*Fx0(1) + FtX1(0)*FtX1(0) + FtX1(1)*FtX1(1);
    return (den > 1e-10) ? std::abs(num) / std::sqrt(den) : 1e9;
}

void MSCKFEstimator::track_features(double timestamp, const cv::Mat& img0, const cv::Mat& img1) {

    // Histogram equalization — normalises contrast so NCC scores are stable
    // across illumination changes (mirrors what reference TrackKLT does).
    cv::Mat img0e, img1e;
    cv::equalizeHist(img0, img0e);
    cv::equalizeHist(img1, img1e);

    if (prev_img0_.empty()) {
        // ── First frame: detect corners in cam0, stereo-match into cam1 ──────
        std::vector<cv::Point2f> corners0;
        cv::goodFeaturesToTrack(img0e, corners0,
                                config_.max_features,
                                0.01,   // qualityLevel
                                10.0);  // minDistance

        for (const auto& c0 : corners0) {
            cv::Point2f c1;
            bool found = track_stereo_epipolar(img0e, img1e, c0, c1,
                                               F_stereo_,
                                               config_.template_size,
                                               config_.stereo_search_radius,
                                               config_.match_threshold);
            if (!found) continue;

            Feature feat;
            feat.id = feature_id_counter_++;
            feat.observations[timestamp] = {
                undistort_point(Eigen::Vector2d(c0.x, c0.y), config_.cam0),
                undistort_point(Eigen::Vector2d(c1.x, c1.y), config_.cam1)
            };
            feature_tracks_[feat.id] = feat;
        }

        prev_img0_ = img0e.clone();
        prev_img1_ = img1e.clone();
        return;
    }

    // ── Subsequent frames: track forward in both cameras ─────────────────────
    for (auto& kv : feature_tracks_) {
        Feature& feat = kv.second;
        if (feat.observations.empty()) continue;

        const auto& last_obs = feat.observations.rbegin()->second;

        // Normalised undistorted → distorted pixel via full projection.
        Eigen::Vector2d px0 = project_point(
            Eigen::Vector3d(last_obs.first(0),  last_obs.first(1),  1.0), config_.cam0);
        Eigen::Vector2d px1 = project_point(
            Eigen::Vector3d(last_obs.second(0), last_obs.second(1), 1.0), config_.cam1);
        double x0 = px0(0), y0 = px0(1);
        double x1 = px1(0), y1 = px1(1);

        cv::Point2f prev0(x0, y0), prev1(x1, y1), curr0, curr1;

        bool ok0 = track_point_template(prev_img0_, img0e, prev0, curr0,
                                        config_.template_size, config_.search_radius,
                                        config_.match_threshold);
        bool ok1 = track_point_template(prev_img1_, img1e, prev1, curr1,
                                        config_.template_size, config_.search_radius,
                                        config_.match_threshold);

        if (!ok0 || !ok1) continue;

        // ── Forward-backward consistency check ────────────────────────────────
        // Track curr→prev and reject if round-trip error exceeds threshold.
        // Catches NCC false positives without needing cv::calib3d/RANSAC.
        cv::Point2f back0, back1;
        bool fb0 = track_point_template(img0e, prev_img0_, curr0, back0,
                                        config_.template_size, config_.search_radius,
                                        config_.match_threshold);
        bool fb1 = track_point_template(img1e, prev_img1_, curr1, back1,
                                        config_.template_size, config_.search_radius,
                                        config_.match_threshold);

        float err0 = std::hypot(back0.x - prev0.x, back0.y - prev0.y);
        float err1 = std::hypot(back1.x - prev1.x, back1.y - prev1.y);
        if (!fb0 || !fb1 || err0 > config_.fb_check_thresh || err1 > config_.fb_check_thresh)
            continue;

        feat.observations[timestamp] = {
            undistort_point(Eigen::Vector2d(curr0.x, curr0.y), config_.cam0),
            undistort_point(Eigen::Vector2d(curr1.x, curr1.y), config_.cam1)
        };
    }

    // ── Re-detect new features when tracked count falls below min_features ────
    int active = 0;
    for (const auto& kv : feature_tracks_)
        if (kv.second.observations.count(timestamp)) active++;

    if (active < config_.min_features) {
        // Mask out existing tracked feature locations
        cv::Mat mask = cv::Mat::ones(img0.size(), CV_8UC1) * 255;
        for (const auto& kv : feature_tracks_) {
            if (!kv.second.observations.count(timestamp)) continue;
            const auto& obs = kv.second.observations.rbegin()->second;
            Eigen::Vector2d mpx = project_point(
                Eigen::Vector3d(obs.first(0), obs.first(1), 1.0), config_.cam0);
            int px = static_cast<int>(mpx(0));
            int py = static_cast<int>(mpx(1));
            cv::circle(mask, cv::Point(px, py), 10, cv::Scalar(0), -1);
        }

        std::vector<cv::Point2f> new_corners;
        cv::goodFeaturesToTrack(img0e, new_corners,
                                config_.max_features - active,
                                0.01, 10.0, mask);

        for (const auto& c0 : new_corners) {
            cv::Point2f c1;
            if (!track_stereo_epipolar(img0e, img1e, c0, c1,
                                      F_stereo_,
                                      config_.template_size,
                                      config_.stereo_search_radius,
                                      config_.match_threshold)) continue;
            Feature feat;
            feat.id = feature_id_counter_++;
            feat.observations[timestamp] = {
                undistort_point(Eigen::Vector2d(c0.x, c0.y), config_.cam0),
                undistort_point(Eigen::Vector2d(c1.x, c1.y), config_.cam1)
            };
            feature_tracks_[feat.id] = feat;
        }
    }

    prev_img0_ = img0e.clone();
    prev_img1_ = img1e.clone();
}

// ==============================================================================
// STATE AUGMENTATION
// ==============================================================================
void MSCKFEstimator::augment_state(double timestamp) {
    CameraClone clone;
    clone.timestamp = timestamp;

    Eigen::Matrix3d R_GtoI = state_.q_GtoI.toRotationMatrix();
    Eigen::Matrix3d R_ItoG = R_GtoI.transpose();

    clone.q_GtoC = Eigen::Quaterniond(config_.R_ItoC0 * R_GtoI);
    clone.q_GtoC.normalize();
    clone.p_CinG = state_.p_IinG + R_ItoG * config_.p_C0inI;

    state_.clones[timestamp] = clone;

    int old_dim = state_.state_dim() - 6;   // dim before this clone was added
    int new_dim = state_.state_dim();

    // ── Jacobian J: d(clone_error) / d(IMU_error) ────────────────────────────
    //
    // δθ_C = R_ItoC0 * δθ_I
    //   → J(0:3, 0:3) = R_ItoC0
    //
    // p_CinG = p_IinG + R_ItoG * p_C0inI
    // Perturb rotation: R_ItoG → R_ItoG * (I + [δθ]×)
    //   δp_CinG = R_ItoG * [δθ]× * p_C0inI
    //           = -R_ItoG * [p_C0inI]× * δθ
    //           = -[R_ItoG * p_C0inI]× * δθ   (adjoint: R[v]× = [Rv]×R, applied here)
    //   → J(3:6, 0:3) = -skew(R_ItoG * p_C0inI)
    //   FIX: was R_ItoG * skew(p_C0inI) — not the same expression
    //
    // δp_CinG from δp_IinG:
    //   → J(3:6, 3:6) = I
    Eigen::Matrix<double, 6, 15> J = Eigen::Matrix<double, 6, 15>::Zero();
    J.block<3,3>(0, 0) = config_.R_ItoC0;
    J.block<3,3>(3, 0) = -skew(R_ItoG * config_.p_C0inI);
    J.block<3,3>(3, 3) = Eigen::Matrix3d::Identity();

    // ── Augment covariance ────────────────────────────────────────────────────
    // New clone cross-covariance with all existing states:
    //   Cov(clone, state_k) = J * P_full(0:15, k)
    // FIX: original only filled the IMU-to-clone block using P_imu, leaving
    // all old-clone cross-correlations at zero.
    Eigen::MatrixXd P_new = Eigen::MatrixXd::Zero(new_dim, new_dim);
    P_new.block(0, 0, old_dim, old_dim) = state_.P_full;

    Eigen::MatrixXd cross = J * state_.P_full.block(0, 0, 15, old_dim); // 6 × old_dim
    P_new.block(new_dim - 6, 0,           6,       old_dim) = cross;
    P_new.block(0,           new_dim - 6, old_dim, 6      ) = cross.transpose();
    P_new.block(new_dim - 6, new_dim - 6, 6,       6      ) = J * state_.P_imu * J.transpose();

    state_.P_full = P_new;
    // P_imu is unchanged by augmentation
}

// ==============================================================================
// TRIANGULATION  (DLT, cam0 + cam1)
// ==============================================================================
bool MSCKFEstimator::triangulate_feature(Feature& feat) {
    if (feat.observations.size() < 2) return false;

    // Pre-allocate for 2 rows per camera per frame (cam0 + cam1)
    int max_rows = static_cast<int>(feat.observations.size()) * 4;
    Eigen::MatrixXd A(max_rows, 3);
    Eigen::VectorXd b_vec(max_rows);
    int row = 0;

    for (const auto& obs_pair : feat.observations) {
        double t = obs_pair.first;
        if (!state_.clones.count(t)) continue;

        const CameraClone& clone = state_.clones.at(t);
        Eigen::Matrix3d R_CtoG  = clone.q_GtoC.toRotationMatrix().transpose();

        // ── cam0 bearing ─────────────────────────────────────────────────────
        // Keep z=1 structure (no normalise()) for better DLT conditioning.
        {
            const Eigen::Vector2d& uv0 = obs_pair.second.first;
            Eigen::Vector3d bear_C(uv0(0), uv0(1), 1.0);
            Eigen::Vector3d bear_G = R_CtoG * bear_C;
            Eigen::Matrix3d Bx = skew(bear_G);
            A.block<2,3>(row, 0)  = Bx.topRows<2>();
            b_vec.segment<2>(row) = (Bx * clone.p_CinG).head<2>();
            row += 2;
        }

        // ── cam1 bearing ─────────────────────────────────────────────────────
        // FIX: cam1 observations were entirely unused in the original.
        // Recover cam1 pose from the stored cam0 clone + stereo extrinsics.
        {
            Eigen::Matrix3d R_GtoI_clone = config_.R_ItoC0.transpose() *
                                           clone.q_GtoC.toRotationMatrix();
            Eigen::Matrix3d R_GtoC1      = config_.R_ItoC1 * R_GtoI_clone;
            Eigen::Vector3d p_C1inG      = clone.p_CinG +
                                           R_GtoI_clone.transpose() *
                                           (config_.p_C1inI - config_.p_C0inI);

            const Eigen::Vector2d& uv1 = obs_pair.second.second;
            Eigen::Vector3d bear_C(uv1(0), uv1(1), 1.0);
            Eigen::Vector3d bear_G = R_GtoC1.transpose() * bear_C;
            Eigen::Matrix3d Bx = skew(bear_G);
            A.block<2,3>(row, 0)  = Bx.topRows<2>();
            b_vec.segment<2>(row) = (Bx * p_C1inG).head<2>();
            row += 2;
        }
    }

    if (row < 6) return false;

    Eigen::Vector3d p_FinG =
        A.topRows(row).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                       .solve(b_vec.head(row));

    if (!p_FinG.allFinite()) return false;

    // ── Reprojection check with consistent distorted-pixel comparison ─────────
    // FIX: original mixed undistorted normalised coords (stored) with distorted
    // pixel (project_point output). Now both sides go through project_point.
    double total_error = 0;
    int    count       = 0;

    for (const auto& obs_pair : feat.observations) {
        double t = obs_pair.first;
        if (!state_.clones.count(t)) continue;

        const CameraClone& clone  = state_.clones.at(t);
        Eigen::Matrix3d    R_GtoC = clone.q_GtoC.toRotationMatrix();
        Eigen::Vector3d    p_FinC = R_GtoC * (p_FinG - clone.p_CinG);

        Eigen::Vector2d predicted = project_point(p_FinC, config_.cam0);
        if (predicted(0) < 0) continue;

        // Forward-project stored normalised coord to get distorted observed pixel
        Eigen::Vector3d uv0_h(obs_pair.second.first(0), obs_pair.second.first(1), 1.0);
        Eigen::Vector2d observed = project_point(uv0_h, config_.cam0);

        total_error += (predicted - observed).norm();
        count++;
    }

    if (count == 0 || total_error / count > config_.max_reprojection_error) return false;

    feat.p_FinG       = p_FinG;
    feat.triangulated = true;
    return true;
}

// ==============================================================================
// MSCKF UPDATE
// ==============================================================================
void MSCKFEstimator::msckf_update(std::vector<Feature*>& features) {

    // Triangulate all candidates; keep only successful ones
    std::vector<Feature*> good;
    good.reserve(features.size());
    for (Feature* f : features)
        if (triangulate_feature(*f)) good.push_back(f);
    if (good.empty()) return;

    const int state_dim = state_.state_dim();

    std::vector<Eigen::MatrixXd> H_blocks;
    std::vector<Eigen::VectorXd> r_blocks;

    for (Feature* feat : good) {
        const Eigen::Vector3d& p_FinG = feat->p_FinG;

        // Count valid observations (those with a corresponding clone in the window)
        int n_obs = 0;
        for (const auto& obs : feat->observations)
            if (state_.clones.count(obs.first)) n_obs++;
        if (n_obs == 0) continue;

        // Per-feature Jacobians and residual
        Eigen::MatrixXd H_f(n_obs * 2, 3);          // w.r.t. feature position
        Eigen::MatrixXd H_x(n_obs * 2, state_dim);  // w.r.t. full state
        Eigen::VectorXd r_vec(n_obs * 2);
        H_f.setZero(); H_x.setZero();

        int lr = 0;
        for (const auto& obs_pair : feat->observations) {
            double t = obs_pair.first;
            if (!state_.clones.count(t)) continue;

            const CameraClone& clone  = state_.clones.at(t);
            Eigen::Matrix3d    R_GtoC = clone.q_GtoC.toRotationMatrix();
            Eigen::Vector3d    p_FinC = R_GtoC * (p_FinG - clone.p_CinG);

            double X = p_FinC(0), Y = p_FinC(1), Z = p_FinC(2);
            if (Z <= 0) { lr += 2; continue; }

            // Projection Jacobian in normalised (undistorted) space.
            // Avoids distorted/undistorted mismatch and is simpler than
            // including the full distortion Jacobian.
            Eigen::Matrix<double, 2, 3> J_proj;
            J_proj << 1.0/Z, 0,     -X/(Z*Z),
                      0,     1.0/Z, -Y/(Z*Z);

            // Jacobian w.r.t. clone pose error [δθ_C, δp_CinG]
            Eigen::Matrix<double, 2, 6> H_clone;
            H_clone.block<2,3>(0,0) = -J_proj * skew(p_FinC);  // dz/dδθ_C
            H_clone.block<2,3>(0,3) = -J_proj * R_GtoC;        // dz/dδp_CinG

            // Jacobian w.r.t. feature position (needed for nullspace projection)
            H_f.block<2,3>(lr, 0) = J_proj * R_GtoC;

            // Column offset of this clone's block in P_full
            int clone_col = 15;
            for (const auto& c : state_.clones) {
                if (c.first == t) break;
                clone_col += 6;
            }
            H_x.block<2,6>(lr, clone_col) = H_clone;

            // Residual in normalised space (consistent with J_proj)
            Eigen::Vector2d z_hat(X / Z, Y / Z);
            r_vec.segment<2>(lr) = obs_pair.second.first - z_hat;

            lr += 2;
        }

        // Trim matrices to actually filled rows
        Eigen::MatrixXd H_f_used = H_f.topRows(lr);
        Eigen::MatrixXd H_x_used = H_x.topRows(lr);
        Eigen::VectorXd r_used   = r_vec.head(lr);

        // ── Nullspace projection ──────────────────────────────────────────────
        // FIX: project H_x and r onto the left nullspace of H_f so the update
        // is independent of the (unestimated) feature position. Without this
        // step the EKF update is theoretically inconsistent.
        //
        // Thin QR of H_f: Q_all = [Q_range | Q_null]
        // Project with T = Q_null^T (rows = left nullspace of H_f)
        if (H_f_used.rows() > 3) {
            Eigen::HouseholderQR<Eigen::MatrixXd> qr(H_f_used);
            Eigen::MatrixXd Q_all =
                qr.householderQ() *
                Eigen::MatrixXd::Identity(H_f_used.rows(), H_f_used.rows());
            Eigen::MatrixXd T = Q_all.rightCols(H_f_used.rows() - 3).transpose();
            H_x_used = T * H_x_used;
            r_used   = T * r_used;
        }

        if (H_x_used.rows() > 0) {
            H_blocks.push_back(H_x_used);
            r_blocks.push_back(r_used);
        }
    }

    if (H_blocks.empty()) return;

    // Stack all per-feature projected blocks
    int total_rows = 0;
    for (const auto& h : H_blocks) total_rows += h.rows();

    Eigen::MatrixXd H_big(total_rows, state_dim);
    Eigen::VectorXd r_big(total_rows);
    int cur = 0;
    for (size_t i = 0; i < H_blocks.size(); i++) {
        int nr = H_blocks[i].rows();
        H_big.block(cur, 0, nr, state_dim) = H_blocks[i];
        r_big.segment(cur, nr)             = r_blocks[i];
        cur += nr;
    }

    // Measurement noise in normalised space: σ_n ≈ σ_pixel / fx
    double sigma_n = config_.sigma_pixel / config_.cam0.fx;
    Eigen::MatrixXd R_meas =
        Eigen::MatrixXd::Identity(total_rows, total_rows) * sigma_n * sigma_n;

    // FIX: use LDLT decomposition instead of .inverse() for numerical stability
    Eigen::MatrixXd S = H_big * state_.P_full * H_big.transpose() + R_meas;
    Eigen::MatrixXd K = state_.P_full * H_big.transpose() *
                        S.ldlt().solve(Eigen::MatrixXd::Identity(total_rows, total_rows));

    Eigen::VectorXd delta_x = K * r_big;

    // ── On-manifold state correction ─────────────────────────────────────────
    state_.q_GtoI  = state_.q_GtoI * delta_q(delta_x.segment<3>(0));
    state_.q_GtoI.normalize();
    state_.p_IinG  += delta_x.segment<3>(3);
    state_.v_IinG  += delta_x.segment<3>(6);
    state_.b_gyro  += delta_x.segment<3>(9);
    state_.b_accel += delta_x.segment<3>(12);

    int idx = 15;
    for (auto& cp : state_.clones) {
        cp.second.q_GtoC = cp.second.q_GtoC * delta_q(delta_x.segment<3>(idx));
        cp.second.q_GtoC.normalize();
        cp.second.p_CinG += delta_x.segment<3>(idx + 3);
        idx += 6;
    }

    // Joseph-form covariance update (numerically stable vs plain (I-KH)P)
    Eigen::MatrixXd I_KH =
        Eigen::MatrixXd::Identity(state_dim, state_dim) - K * H_big;
    state_.P_full = I_KH * state_.P_full * I_KH.transpose() + K * R_meas * K.transpose();
    state_.P_imu  = state_.P_full.block<15,15>(0, 0);
}

// ==============================================================================
// MARGINALIZATION
// ==============================================================================
void MSCKFEstimator::marginalize_oldest_clone() {
    if (state_.clones.empty()) return;

    double oldest_time = state_.clones.begin()->first;

    // FIX 1: compute clone_idx BEFORE erasing.
    // Original erased first, then searched — the key was never found so
    // clone_idx ended up pointing past all clones, causing OOB covariance writes.
    int clone_idx = 15;
    for (const auto& c : state_.clones) {
        if (c.first == oldest_time) break;
        clone_idx += 6;
    }

    const int old_dim = state_.state_dim();   // still includes the clone
    state_.clones.erase(oldest_time);
    const int new_dim = state_.state_dim();   // = old_dim - 6

    // FIX 2: plain row/column deletion, not a Schur complement.
    // The clone's observations have already been consumed by msckf_update;
    // we simply remove its block from the covariance matrix.
    //
    // Partition:
    //   A = [0,           clone_idx)       — states before the clone
    //   B = [clone_idx,   clone_idx+6)     — the clone being dropped
    //   C = [clone_idx+6, old_dim)         — states after the clone
    const int sz_A = clone_idx;
    const int sz_C = old_dim - clone_idx - 6;

    Eigen::MatrixXd P_new = Eigen::MatrixXd::Zero(new_dim, new_dim);

    // A-A
    P_new.block(0, 0, sz_A, sz_A) =
        state_.P_full.block(0, 0, sz_A, sz_A);

    if (sz_C > 0) {
        // A-C
        P_new.block(0, sz_A, sz_A, sz_C) =
            state_.P_full.block(0, clone_idx + 6, sz_A, sz_C);
        // C-A
        P_new.block(sz_A, 0, sz_C, sz_A) =
            state_.P_full.block(clone_idx + 6, 0, sz_C, sz_A);
        // C-C
        P_new.block(sz_A, sz_A, sz_C, sz_C) =
            state_.P_full.block(clone_idx + 6, clone_idx + 6, sz_C, sz_C);
    }

    state_.P_full = P_new;
    state_.P_imu  = state_.P_full.block<15,15>(0, 0);
}

// ==============================================================================
// PUBLIC INTERFACE
// ==============================================================================
void MSCKFEstimator::feed_imu(double timestamp,
                               const Eigen::Vector3d& w,
                               const Eigen::Vector3d& a) {
    if (!initialized_) {
        imu_buffer_.push_back({timestamp, w, a});
        return;
    }
    propagate_imu(timestamp, w, a);
}

void MSCKFEstimator::feed_stereo(double timestamp,
                                  const cv::Mat& img0,
                                  const cv::Mat& img1) {
    if (img0.empty() || img1.empty()) return;

    track_features(timestamp, img0, img1);

    if (!initialized_) {
        if (try_initialize(timestamp)) initialized_ = true;
        return;
    }

    augment_state(timestamp);

    // Collect features not tracked in this frame
    std::vector<Feature*> lost_features;
    for (auto& kv : feature_tracks_) {
        Feature& feat = kv.second;
        if (!feat.observations.count(timestamp)) {
            if (feat.observations.size() >= static_cast<size_t>(config_.min_track_length))
                lost_features.push_back(&feat);
            feat.should_marginalize = true;
        }
    }

    if (!lost_features.empty()) msckf_update(lost_features);

    // Prune dead features
    for (auto it = feature_tracks_.begin(); it != feature_tracks_.end(); )
        it = it->second.should_marginalize ? feature_tracks_.erase(it) : ++it;

    // Slide the clone window.
    // Before dropping the oldest clone, do an opportunistic MSCKF update with
    // any features that have an observation there and are still being tracked.
    // Without this, features tracked across the full window never contribute
    // to the EKF update until they happen to drop — wasting all that baseline.
    while (state_.clones.size() > static_cast<size_t>(config_.max_clone_size)) {
        double oldest_time = state_.clones.begin()->first;

        std::vector<Feature*> expiring;
        for (auto& kv : feature_tracks_) {
            Feature& feat = kv.second;
            if (!feat.should_marginalize &&
                feat.observations.count(oldest_time) &&
                feat.observations.size() >= static_cast<size_t>(config_.min_track_length)) {
                expiring.push_back(&feat);
            }
        }
        if (!expiring.empty()) msckf_update(expiring);

        // Drop the oldest observation from still-live features so triangulation
        // never references a clone that no longer exists.
        for (auto& kv : feature_tracks_)
            kv.second.observations.erase(oldest_time);

        marginalize_oldest_clone();
    }
}

} // namespace OpenVINS