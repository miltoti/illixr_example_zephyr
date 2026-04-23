#include <zephyr/kernel.h>
#include <cstdint>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "../../src/data_format_opencv.hpp"
#include "../../src/helper/opencv_helper.hpp"
#include "../../src/threadloop.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/stoplight.hpp"
#include "../openvins/openvins_queues.hpp"

#include "embedded_cam.hpp"

using namespace ILLIXR;

K_THREAD_STACK_DEFINE(offline_cam_stack, 524288);

class Offline_cam : public threadloop {
public:
    explicit Offline_cam(phonebook_new& pb)
        : threadloop{pb, "offline_cam",
                     offline_cam_stack,
                     K_THREAD_STACK_SIZEOF(offline_cam_stack),
                     5}
        , current_idx_{0}
    {
        printf("[offline_cam] constructed  frames=%zu (EuRoC embedded)\n",
               kEmbeddedCamCount);
    }

    void _p_thread_setup() override {
        printf("[offline_cam] _p_thread_setup() tid=%p\n", k_current_get());
    }

    skip_option _p_should_skip() override {
        if (current_idx_ >= kEmbeddedCamCount)
            return skip_option::stop;
        return skip_option::run;
    }

    void _p_one_iteration() override {
        k_sem_take(&stoplight_cam, K_FOREVER);

        const auto& frame = kEmbeddedCam[current_idx_];

        // Wrap raw PNG bytes in a cv::Mat (no copy), then decode
        cv::Mat png0_buf(1, (int)frame.cam0_size, CV_8UC1,
                         const_cast<uint8_t*>(frame.cam0_png));
        cv::Mat png1_buf(1, (int)frame.cam1_size, CV_8UC1,
                         const_cast<uint8_t*>(frame.cam1_png));

        cv::Mat img0 = cv::imdecode(png0_buf, cv::IMREAD_GRAYSCALE);
        cv::Mat img1 = cv::imdecode(png1_buf, cv::IMREAD_GRAYSCALE);

        if (img0.empty() || img1.empty()) {
            printf("[offline_cam] ERROR: imdecode failed frame %zu\n", current_idx_);
            k_sem_give(&stoplight_cam);
            ++current_idx_;
            return;
        }

        CamMsg* msg = new CamMsg{
            ILLIXR::time_point{std::chrono::nanoseconds{frame.ts_ns}},
            img0,
            img1
        };

        int rc = k_msgq_put(&openvins_cam_queue, &msg, K_NO_WAIT);
        
        if (rc != 0) {
            delete msg;
            printf("[offline_cam] ERROR: queue put failed rc=%d\n", rc);
        }

        printf("[offline_cam] SENT frame #%03zu  ts=%lld ns  img=%dx%d\n",
               current_idx_, (long long)frame.ts_ns,
               img0.cols, img0.rows);

        ++current_idx_;
    }

private:
    size_t current_idx_;
};

void start_offline_cam(phonebook_new& pb) {
    static Offline_cam instance{pb};
    instance.start();
}

REGISTER_PLUGIN(offline_cam);