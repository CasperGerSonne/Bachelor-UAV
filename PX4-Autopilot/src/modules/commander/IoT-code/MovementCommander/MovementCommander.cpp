#include <commander/Commander.hpp>
#include <stdio.h>
#include <commander/IoT-code/MovementCommander/GPS/GPSController.hpp>
#include <commander/IoT-code/MovementCommander/MovementCommander.hpp>


static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}


// our includes:

MovementCommander::MovementCommander(){}
MovementCommander::~MovementCommander(){}

int MovementCommander::GPSmovementtest(){


    GPSController C_GPS(5000);

    double* init_lat; double* init_lon; double* init_alt;
    C_GPS.getposition(init_lat,init_lon,init_alt);
    // Define waypoint offsets
    constexpr double LAT_OFFSET = 2.0 / 1e7; // 2 meters in latitude
    double LON_OFFSET = 2.0 / (1e7 * cos(*init_lat * M_PI / 180)); // 2 meters in longitude

    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);

    // Take off to 2 meters altitude
    printf("Takeoff\n");
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF, *init_lat, start_longitude, start_altitude + 2, 0);

    sleep(5);

    printf("Move lat +2\n");
    // Move to waypoint 2 meters in latitude
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_WAYPOINT, *init_lat + LAT_OFFSET, start_longitude, start_altitude + 2, 0);
    sleep(5);

    printf("Move long +2\n");
    // Move to waypoint 2 meters in longitude
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_WAYPOINT, *init_lat + LAT_OFFSET, start_longitude + LON_OFFSET, start_altitude + 2, 0);
    sleep(5);

    printf("Move lat -2\n");
    // Move back to 2 meters altitude
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_WAYPOINT, *init_lat, start_longitude + LON_OFFSET, start_altitude+2, 0);
    sleep(5);

    printf("Move long-2\n");
    // Move back 2 meters in longitude
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_WAYPOINT, *init_lat, start_longitude, start_altitude + 2, 0);
    sleep(5);

    printf("land\n");
    // Land in the starting position
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND, *init_lat, start_longitude, 0, 0);
    return 0;
}



int MovementCommander::Movingtest(){

    printf("Arming\n");

    // possibly not used
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);

    printf("Taking off to 2 meters\n");
    // switch to takeoff mode and arm
    int gps_sub = orb_subscribe(ORB_ID(sensor_gps));
    px4_pollfd_struct_t fds_gps;
    fds_gps.fd = gps_sub;
    fds_gps.events = POLLIN;

    // Define a structure to hold the GPS position data
    struct sensor_gps_s gps_position;

    // Wait for new data
    px4_poll(&fds_gps,1,1000);

    orb_copy(ORB_ID(sensor_gps), gps_sub, &gps_position);

    // Advertise the vehicle_command topic for publication
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF,2.0f);

    printf("sleeping 5\n");
    sleep(5);

    for (int j=0; j<3;j++){
        printf("param%d test\n",j);
        float speedchange[] = {1,0,-1};



        for (int i=0; i<3;i++){


            printf("sending move command %d \n",i);
            // Advertise the vehicle_command topic for publication
            if (j==0){
                send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,speedchange[i],0,0);
            }else if(j==1){
                send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,0,speedchange[i],0);
            }else{
                send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_REPOSITION,0,0,speedchange[i]);
            }


            printf("sleeping for 2 seconds");
            sleep(2);

        }

    }

    // Clean up

    printf("DONE");
    return 0;

}



