#include <ros/ros.h>
#include <mavros_msgs/RCOut.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h> // for strerror
#include <stdio.h>  // for printf and fprintf
#include <errno.h>  // for errno
#include <stdint.h>
#include <sys/time.h>


#define GRAVITY_MSS     9.80665f

static const uint64_t GAZEBO_TIMEOUT_US = 5000000;

void send_servos(const struct sitl_input &input); // Function declaration
void sync_frame_time(void); // Function declaration



// UDP server information
const char* gazebo_address = "127.0.0.1"; // Localhost
const int gazebo_port_in = 9003;
const int gazebo_port_out = 9002;
int udpSocket;

/*
Matrix3f dcm;                        // rotation matrix, APM conventions, from body to earth
Vector3f gyro;                       // rad/s
Vector3f velocity_ef;                // m/s, earth frame
Vector3f position;                   // meters, NED from origin
Vector3f accel_body{0.0f, 0.0f, -GRAVITY_MSS}; // m/s/s NED, body frame*/

float imuLinearAccelerationXYZ[3];
float imuAngularVelocityRPY[3];


uint64_t time_now_us;
uint64_t last_wall_time_us;
double last_timestamp;
float rate_hz = 1200.0f;
float target_speedup;
uint64_t frame_time_us;
bool use_time_sync = true;
uint64_t last_time_us;
uint32_t frame_counter;
uint32_t last_ground_contact_ms;
/* return a monotonic wall clock time in microseconds */
uint64_t get_wall_time_us(void);
int64_t sleep_debt_us;
#if defined(__CYGWIN__) || defined(__CYGWIN64__)
    const uint32_t min_sleep_time{20000};
#else
    const uint32_t min_sleep_time{5000};
#endif
uint32_t last_fps_report_ms;
uint32_t last_frame_count;





/*
  Drain remaining data on the socket to prevent phase lag.
 */
void drain_sockets()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    errno = 0;
    do {
        received = recv(udpSocket,buf, buflen, 0);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
                fprintf(stderr, "error recv on socket in: %s \n",
                        strerror(errno));
            }
        } else {
            // fprintf(stderr, "received from control socket: %s\n", buf);
        }
    } while (received > 0);

}

/* advance time by deltat in seconds */
void time_advance()
{
    // we only advance time if it hasn't been advanced already by the
    // backend
    if (last_time_us == time_now_us) {
        time_now_us += frame_time_us;
    }
    last_time_us = time_now_us;
    if (use_time_sync) {
        sync_frame_time();
    }
}

void sync_frame_time(void)
{
    frame_counter++;
    uint64_t now = get_wall_time_us();
    uint64_t dt_us = now - last_wall_time_us;

    const float target_dt_us = 1.0e6/(rate_hz*target_speedup);

    // accumulate sleep debt if we're running too fast
    sleep_debt_us += target_dt_us - dt_us;

    if (sleep_debt_us < -1.0e5) {
        // don't let a large negative debt build up
        sleep_debt_us = -1.0e5;
    }
    if (sleep_debt_us > min_sleep_time) {
        // sleep if we have built up a debt of min_sleep_tim
        usleep(sleep_debt_us);
        sleep_debt_us -= (get_wall_time_us() - now);
    }
    last_wall_time_us = get_wall_time_us();

    uint32_t now_ms = last_wall_time_us / 1000ULL;
    float dt_wall = (now_ms - last_fps_report_ms) * 0.001;
    if (dt_wall > 2.0) {
#if 0
        const float achieved_rate_hz = (frame_counter - last_frame_count) / dt_wall;
        ::printf("Rate: target:%.1f achieved:%.1f speedup %.1f/%.1f\n",
                 rate_hz*target_speedup, achieved_rate_hz,
                 achieved_rate_hz/rate_hz, target_speedup);
#endif
        last_frame_count = frame_counter;
        last_fps_report_ms = now_ms;
    }
}

/*
  structure passed in giving servo positions as PWM values in
  microseconds
*/
struct sitl_input {
    float servos[16];
    struct {
        float speed;      // m/s
        float direction;  // degrees 0..360
        float turbulence;
        float dir_z;	  //degrees -90..90
    } wind;
};

sitl_input input;

struct sockaddr_in udpServerAddr; // Declare udpServerAddr here

    struct fdm_packet {
      double timestamp;  // in seconds
      double imu_angular_velocity_rpy[3];
      double imu_linear_acceleration_xyz[3];
      double imu_orientation_quat[4];
      double velocity_xyz[3];
      double position_xyz[3];
    };


/* adjust frame_time calculation */
void adjust_frame_time(float new_rate)
{
    frame_time_us = uint64_t(1.0e6f/new_rate);
    rate_hz = new_rate;
}

uint64_t get_wall_time_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);

}

/*
  receive an update from the FDM
  This is a blocking function
 */
void recv_fdm(const struct sitl_input &input)
{
    fdm_packet pkt;

    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    while (recv(udpSocket, &pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        send_servos(input);
        // Reset the timestamp after a long disconnection, also catch gazebo reset
        if (get_wall_time_us() > last_wall_time_us + GAZEBO_TIMEOUT_US) {
            last_timestamp = 0;
        }
    }

    const double deltat = pkt.timestamp - last_timestamp;  // in seconds
    if (deltat < 0) {  // don't use old packet
        time_now_us += 1;
        return;
    }
    
    
    
    //imuLinearAccelerationXYZ[0] = linearAccel.X();
    //imuLinearAccelerationXYZ[1] = linearAccel.Y();
    //imuLinearAccelerationXYZ[2] = linearAccel.Z();

    //imuAngularVelocityRPY[0] = angularVel.X();
    //imuAngularVelocityRPY[1] = angularVel.Y();
    //imuAngularVelocityRPY[2] = angularVel.Z();

    
    /*
    // get imu stuff
    accel_body = Vector3f(static_cast<float>(pkt.imu_linear_acceleration_xyz[0]),
                          static_cast<float>(pkt.imu_linear_acceleration_xyz[1]),
                          static_cast<float>(pkt.imu_linear_acceleration_xyz[2]));

    gyro = Vector3f(static_cast<float>(pkt.imu_angular_velocity_rpy[0]),
                    static_cast<float>(pkt.imu_angular_velocity_rpy[1]),
                    static_cast<float>(pkt.imu_angular_velocity_rpy[2]));

    // compute dcm from imu orientation
    Quaternion quat(static_cast<float>(pkt.imu_orientation_quat[0]),
                    static_cast<float>(pkt.imu_orientation_quat[1]),
                    static_cast<float>(pkt.imu_orientation_quat[2]),
                    static_cast<float>(pkt.imu_orientation_quat[3]));
    quat.rotation_matrix(dcm);

    velocity_ef = Vector3f(static_cast<float>(pkt.velocity_xyz[0]),
                           static_cast<float>(pkt.velocity_xyz[1]),
                           static_cast<float>(pkt.velocity_xyz[2]));

    position = Vector3f(static_cast<float>(pkt.position_xyz[0]),
                        static_cast<float>(pkt.position_xyz[1]),
                        static_cast<float>(pkt.position_xyz[2]));*/


    // auto-adjust to simulation frame rate
    time_now_us += static_cast<uint64_t>(deltat * 1.0e6);

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(static_cast<float>(1.0/deltat));
    }
    last_timestamp = pkt.timestamp;

}



/*
packet sent to Gazebo
*/
struct servo_packet {
    // size matches sitl_input upstream
    float motor_speed[16];
};

/*
  Create and set in/out socket
*/
void set_interface_ports(const char* address, const int port_in, const int port_out) {
    // Create a UDP socket
    udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0) {
        perror("SITL: socket creation failed");
        exit(1);
    }

    // Bind to a specific port
    struct sockaddr_in bindAddr;
    memset(&bindAddr, 0, sizeof(bindAddr));
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_port = htons(port_in);
    bindAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(udpSocket, (struct sockaddr*)&bindAddr, sizeof(bindAddr)) < 0) {
        perror("HITL: socket bind failed");
        close(udpSocket);
        exit(1);
    }
    printf("Bind %s:%d for HITL in\n", "127.0.0.1", port_in);

    // Configure UDP server address
    memset(&udpServerAddr, 0, sizeof(udpServerAddr));
    udpServerAddr.sin_family = AF_INET;
    udpServerAddr.sin_port = htons(port_out);
    udpServerAddr.sin_addr.s_addr = inet_addr(address);

    printf("Setting Gazebo interface to %s:%d \n", address, port_out);
}

void send_servos(const struct sitl_input &input)
{
    servo_packet pkt;
    // should rename servo_command
    // 16 because struct sitl_input.servos is 16 large in SIM_Aircraft.h
    for (unsigned i = 0; i < 16; ++i)
    {
      pkt.motor_speed[i] = -1;
      if (input.servos[i]!=0){
      pkt.motor_speed[i] = ((input.servos[i]-1000) / 1000.0f);
      }
      //fprintf(stderr, "\n PWM %d is %f \n ", i, pkt.motor_speed[i]);
    }
    //socket_sitl.sendto(&pkt, sizeof(pkt), _gazebo_address, _gazebo_port);] // TODO: ADICIONAR SocketAPM DE AP_HAL/UTILITY
    sendto(udpSocket, &pkt, sizeof(pkt), 0, (struct sockaddr*)&udpServerAddr, sizeof(udpServerAddr));    
}


void rcOutCallback(const mavros_msgs::RCOut::ConstPtr& msg) {
    // Prepare the data to be sent via UDP

    // should rename servo_command
    // 16 because struct sitl_input.servos is 16 large in SIM_Aircraft.h
    //servo_packet pkt;
    //sitl_input input; 

    for (unsigned i = 0; i < 16; ++i) {
    //    pkt.motor_speed[i] = (msg->channels[i] - 1000) / 1000.0f;
          input.servos[i] = msg->channels[i];
    //    fprintf(stderr, "\n %f \n ", pkt.motor_speed[i]);
    }

    // Send the data via UDP
    //sendto(udpSocket, &pkt, sizeof(pkt), 0, (struct sockaddr*)&udpServerAddr, sizeof(udpServerAddr));
    
    
    //send_servos(input);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rx_data_node");
    ros::NodeHandle nh;

    set_interface_ports(gazebo_address, gazebo_port_in, gazebo_port_out);

    // ROS subscriber to /mavros/rc/out
    ros::Subscriber rcOutSub = nh.subscribe("/mavros/rc/out", 1, rcOutCallback);

    // Create a ROS rate object to control the rate of execution
    ros::Rate rate(1000); // 10 Hz, adjust the rate as needed

    while (ros::ok()) {
        // Use the ROS rate to control the rate of execution
        rate.sleep();
        send_servos(input);
        //recv_fdm(input);
        time_advance();
        //drain_sockets();
        
            
        ros::spinOnce(); // Process any pending ROS callbacks

    }

    // Close the UDP socket when done
    close(udpSocket);

    return 0;
}

