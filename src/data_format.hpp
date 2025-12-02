#include <relative_clock.hpp>
#include <Eigen/Dense>

// in ../../src/data_format.hpp (approx)
namespace ILLIXR {
    using ullong = unsigned long long;

    struct ImuMsg {
        time_point      time;
        Eigen::Vector3d angular_v;
        Eigen::Vector3d linear_a;
    };
}
