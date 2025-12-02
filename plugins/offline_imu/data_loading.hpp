/*#pragma once

#include <Eigen/Dense>
#include "../../src/helper/csv_iterator.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include "../../src/relative_clock.hpp"
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

typedef unsigned long long ullong;

struct raw_imu_type {
    Eigen::Vector3d angular_v;
    Eigen::Vector3d linear_a;
};

struct sensor_types {
    raw_imu_type imu0;
};

static std::map<ullong, sensor_types> load_data() {
    // Commenting out for now. We will need to rework how data loading works in Zephyr.
    /*
    const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
    printk("[offline_imu] ILLIXR_DATA='%s'\n",
           illixr_data_c_str ? illixr_data_c_str : "NULL");

    if (!illixr_data_c_str) {
        printk("[offline_imu] ERROR: Please define ILLIXR_DATA\n");
        exit(EXIT_FAILURE);
    }
        */
        /*
    const std::string illixr_data = "./data/V1_02_medium/mav0";

    //std::string illixr_data = std::string{illixr_data_c_str};
    std::map<ullong, sensor_types> data;

    const std::string imu0_subpath = "/imu0/data.csv";
    std::string       imu0_path    = illixr_data + imu0_subpath;

    std::ifstream imu0_file{imu0_path};
    printk("[offline_imu] Loading IMU data from: %s\n", imu0_path.c_str());
    if (!imu0_file.good()) {
        printk("[offline_imu] ERROR: Cannot open IMU file: %s\n", imu0_path.c_str());
    }

    for (CSVIterator row{imu0_file, 1}; row != CSVIterator{}; ++row) {
        ullong          t = std::stoull(row[0]);
        Eigen::Vector3d av{std::stod(row[1]), std::stod(row[2]), std::stod(row[3])};
        Eigen::Vector3d la{std::stod(row[4]), std::stod(row[5]), std::stod(row[6])};
        data[t].imu0 = {av, la};
    }

    printk("[offline_imu] Loaded %u IMU samples\n", (unsigned int)data.size());
    return data;
}
*/

#pragma once
#include <cstdint>
#include <cstdint>
#include <map>

namespace ILLIXR {
// 1. Define the structure for a single data point (row)
typedef struct {
    long timestamp; 
    double col1;
    double col2;
    double col3;
    double col4;
    double col5;
    double col6;
} DataPoint;

#define DATA_SIZE 23


DataPoint sensorData[DATA_SIZE] = {
    {1403715523912143104, -0.00069813170079773186, 0.019547687622336492, 0.076794487087750496, 9.2182509999999986, 0.30237170833333332, -3.1544724166666662},
    {1403715523917143040, -0.00069813170079773186, 0.020943951023931952, 0.072605696882964116, 9.3163174999999985, 0.29419949999999995, -3.2525389166666665},
    {1403715523922142976, -0.0034906585039886592, 0.023038346126325153, 0.074700091985357306, 9.2100787916666658, 0.29419949999999995, -3.1789890416666662},
    {1403715523927142912, -0.0027925268031909274, 0.017453292519943295, 0.07819075048934597, 9.3244897083333331, 0.30237170833333332, -3.2607111249999998},
    {1403715523932143104, -0.0034906585039886592, 0.022340214425527419, 0.080285145591739146, 9.2264232083333333, 0.26968287499999999, -3.1953334583333328},
    {1403715523937143040, -0.0027925268031909274, 0.021642082724729686, 0.080983277292536876, 9.3244897083333331, 0.34323275000000003, -3.2198500833333328},
    {1403715523942142976, -0.00069813170079773186, 0.024434609527920613, 0.07749261878854824, 9.2264232083333333, 0.34323275000000003, -3.1789890416666662},
    {1403715523947142912, 0.0, 0.018849555921538759, 0.07819075048934597, 9.2999730833333327, 0.35957716666666667, -3.1789890416666662},
    {1403715523952143104, 0.0, 0.020943951023931952, 0.078888882190143686, 9.2345954166666662, 0.3350605416666666, -3.2116778749999999},
    {1403715523957143040, -0.0020943951023931952, 0.020245819323134219, 0.07749261878854824, 9.3081452916666674, 0.36774937499999999, -3.2035056666666666},
    {1403715523962142976, -0.0027925268031909274, 0.024434609527920613, 0.07958701389094143, 9.2019065833333329, 0.30237170833333332, -3.138128},
    {1403715523967142912, -0.0013962634015954637, 0.020245819323134219, 0.076794487087750496, 9.2918008749999998, 0.34323275000000003, -3.1708168333333329},
    {1403715523972143104, -0.0013962634015954637, 0.021642082724729686, 0.07609635538695278, 9.2345954166666662, 0.3105439166666667, -3.2035056666666666},
    {1403715523977143040, -0.0034906585039886592, 0.020245819323134219, 0.075398223686155036, 9.2918008749999998, 0.35140495833333335, -3.2198500833333328},
    {1403715523982142976, -0.0048869219055841231, 0.023736477827122883, 0.076794487087750496, 9.2591120416666666, 0.3105439166666667, -3.1871612499999999},
    {1403715523987142912, -0.0027925268031909274, 0.019547687622336492, 0.075398223686155036, 9.2754564583333323, 0.34323275000000003, -3.1953334583333328},
    {1403715523992143104, -0.0034906585039886592, 0.020943951023931952, 0.07609635538695278, 9.2264232083333333, 0.31871612500000002, -3.2116778749999999},
    {1403715523997143040, -0.0034906585039886592, 0.017453292519943295, 0.074700091985357306, 9.242767624999999, 0.35140495833333335, -3.2198500833333328},
    {1403715524002142976, -0.0034906585039886592, 0.018849555921538759, 0.080285145591739146, 9.2264232083333333, 0.29419949999999995, -3.1708168333333329},
    {1403715524007142912, -0.0048869219055841231, 0.018849555921538759, 0.07819075048934597, 9.2836286666666652, 0.35957716666666667, -3.2116778749999999},
    {1403715524012143104, -0.0034906585039886592, 0.023736477827122883, 0.075398223686155036, 9.2264232083333333, 0.3350605416666666, -3.2035056666666666},
    {1403715524017143040, 0.00069813170079773186, 0.016755160819145562, 0.078888882190143686, 9.2672842499999994, 0.3350605416666666, -3.2035056666666666},
    {1403715524022142976, -0.0027925268031909274, 0.016755160819145562, 0.078888882190143686, 9.242767624999999, 0.30237170833333332, -3.2035056666666666}
};

typedef struct {
    Eigen::Vector3d angular_v;
    Eigen::Vector3d linear_a;
} raw_imu_type;

typedef struct {
    raw_imu_type imu0;
} sensor_types;


static std::map<ullong, sensor_types> load_data() {
    std::map<ullong, sensor_types> data;
    
    // Iterate through the hardcoded array instead of reading from a file
    for (size_t i = 0; i < DATA_SIZE; ++i) {
        
        // 1. Extract the timestamp and cast it to ullong (unsigned long long)
        ullong t = static_cast<ullong>(sensorData[i].timestamp);
        
        // 2. Load the Angular Velocity (col1, col2, col3) into Eigen::Vector3d
        Eigen::Vector3d av{
            sensorData[i].col1,
            sensorData[i].col2,
            sensorData[i].col3
        };
        
        // 3. Load the Linear Acceleration (col4, col5, col6) into Eigen::Vector3d
        Eigen::Vector3d la{
            sensorData[i].col4,
            sensorData[i].col5,
            sensorData[i].col6
        };
        
        // 4. Populate the map using the defined structs:
        //    data[t].imu0 is a raw_imu_type, which is initialized with {av, la}.
        data[t].imu0 = {av, la}; 
    }
    
    // Log the loaded count (using printf as a substitute for printk/spdlog)
    printf("[offline_imu] Loaded %u IMU samples\n", (unsigned int)data.size());
    
    return data;
}
} // namespace ILLIXR
