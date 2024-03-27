#include <commander/Commander.hpp>




class GPSController{
    private:
    int gps_sub;
    struct sensor_gps_s gps_s;

    double* start_latitude;
    double* start_longitude;
    double* start_altitude;


    double* waypoint_latitude;
    double* waypoint_longitude;
    double* waypoint_altitude;

    double* getDistancesFromStartOrWaypoint(bool Start){
        double pos_lat;double pos_long;double pos_alt;
        if(Start){
            pos_lat = * start_latitude ;pos_long = *start_longitude ;pos_alt = * start_altitude;
        }else{
            pos_lat = * waypoint_latitude ;pos_long = *waypoint_longitude ;pos_alt = * waypoint_altitude;
        }

        double* current_latitude; double* current_longitude; double* current_altitude;

        getposition(current_latitude,current_longitude,current_altitude,2000);

        double* distances = new double[3];
        // Assign values to the array
        distances[0] = (*current_latitude - pos_lat) * 111111;
        distances[1] = (*current_longitude - pos_long) * (M_PI / 180) * 6371000 * cos(*current_latitude * (M_PI / 180));
        distances[2] = *current_altitude - pos_alt;
        // Return pointer to the array
        return distances;

    }

    public:

    GPSController(int CBinterval_ms) {

        gps_sub = orb_subscribe(ORB_ID(sensor_gps));
        orb_set_interval(gps_sub, CBinterval_ms); // callback interval

        // get starting location

        if (!getposition(start_latitude,start_longitude,start_altitude,CBinterval_ms*5)){
            PX4_ERR("No valid stating location");
        };
    }

    ~GPSController() {
        // Destructor implementation
    }


    bool getposition(double *latitude,double *longitude,double *altitude,int poll_T_ms){
        // Set callback interval to 1 second
        px4_pollfd_struct_t fds[1];
        fds[0].fd = gps_sub;
        fds[0].events = POLLIN;

        int poll_ret = px4_poll(fds, 1, poll_T_ms);

        if (poll_ret == 0) {
            PX4_WARN("Timeout: No data received");
            return 0;

        } else if(poll_ret < 0) {
            PX4_ERR("Error: poll failed");
            return 0;

        }

        orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps_s);

        *latitude = gps_s.latitude_deg / 1e7; // Convert from 1e7 scale to degrees
        *longitude = gps_s.longitude_deg / 1e7; // Convert from 1e7 scale to degrees
        *altitude = gps_s.altitude_msl_m / 1e3;

        return 1;

    }

    double* getDistancesFromStart(){
        return getDistancesFromStartOrWaypoint(true);
    }
    double* getDistancesFromWaypoint(){
        return getDistancesFromStartOrWaypoint(false);
    }


    double meters_to_longitude(double meters) {
        // Calculate the circumference of the Earth at the given latitude
        return meters / (6371000 * cos(*start_latitude * (M_PI / 180)) * (M_PI / 180));
    }

};


