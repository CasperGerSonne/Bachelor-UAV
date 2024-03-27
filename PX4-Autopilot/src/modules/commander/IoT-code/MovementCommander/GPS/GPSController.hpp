#ifndef GPS_CONTROLLER_HPP
#define GPS_CONTROLLER_HPP

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_gps.h>


class GPSController {
private:
    int gps_sub;
    struct sensor_gps_s gps_s;

    double* start_latitude;
    double* start_longitude;
    double* start_altitude;

    double* waypoint_latitude;
    double* waypoint_longitude;
    double* waypoint_altitude;

    double* getDistancesFromStartOrWaypoint(bool Start);

public:
    GPSController(int CBinterval_ms);
    ~GPSController();

    bool getposition(double *latitude, double *longitude, double *altitude, int poll_T_ms);

    double* getDistancesFromStart();
    double* getDistancesFromWaypoint();

    double meters_to_longitude(double meters);
};

#endif // GPS_CONTROLLER_HPP
