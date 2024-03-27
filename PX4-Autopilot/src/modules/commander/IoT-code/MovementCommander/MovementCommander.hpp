#ifndef MOVEMENT_COMMANDER_HPP
#define MOVEMENT_COMMANDER_HPP

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_command.h>

class MovementCommander {
public:
	MovementCommander();
	~MovementCommander();
    int GPSmovementtest();
    int Movingtest();
};

#endif // MOVEMENT_COMMANDER_HPP
