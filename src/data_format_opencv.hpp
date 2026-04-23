#include <relative_clock.hpp>
#include <Eigen/Dense>
// in ../../src/data_format.hpp (approx)
namespace ILLIXR {
    using ullong = unsigned long long;

    struct CamMsg {
        time_point      time;
        cv::Mat         img0;
        cv::Mat         img1;
    };

}
