#pragma once

#include <Eigen/Core>
#include <GeographicLib/LocalCartesian.hpp>

namespace ryu {

struct ImuData {
    double timestamp;

    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};
using ImuDataPtr = std::shared_ptr<ImuData>;
using ImuDataConstPtr = std::shared_ptr<const ImuData>;

struct gnssData {
    double timestamp;

    Eigen::Vector3d lla;  // Latitude in degree, longitude in degree, and altitude in meter
    Eigen::Matrix3d cov;  // Covariance in m^2
};
using gnssDataPtr = std::shared_ptr<gnssData>;
using gnssDataConstPtr = std::shared_ptr<const gnssData>;

struct odomData {
    double timestamp;

    Eigen::Vector3d position;  
    Eigen::Vector3d vel;  
};
using odomDataPtr = std::shared_ptr<odomData>;
using odomDataConstPtr = std::shared_ptr<const odomData>;

inline Eigen::Matrix3d skew_matrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;

    return w;
}

inline void lla2enu(const Eigen::Vector3d& init_lla,
                    const Eigen::Vector3d& point_lla,
                    Eigen::Vector3d* point_enu) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2),
                            point_enu->data()[0], point_enu->data()[1], point_enu->data()[2]);
}

inline void enu2lla(const Eigen::Vector3d& init_lla,
                    const Eigen::Vector3d& point_enu,
                    Eigen::Vector3d* point_lla) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2),
                            point_lla->data()[0], point_lla->data()[1], point_lla->data()[2]);
}

}  // namespace ryu
