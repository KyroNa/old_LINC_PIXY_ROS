//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include "pixy.h"
#include <cmath>
#include <iostream>
#include <array>
#include <algorithm> // for std::min, std::max
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#define BLOCK_BUFFER_SIZE    3

// Real distances between beacons in meters
// To be updated 
constexpr double real_distance1 = 2.5;
constexpr double real_distance2 = real_distance1;
constexpr double real_distance3 = 3.0;

// Pixy Block buffer // 
struct Block blocks[BLOCK_BUFFER_SIZE];

static bool run_flag = true;

void handle_SIGINT(int unused)
{
  // On CTRL+C - abort! //

  run_flag = false;
}

// Function to calculate the distance between two points on image plane 
inline double distance (Block b1, Block b2)
{
  return sqrt(pow((b1.x) - (b2.x),2) + pow((b1.y) - (b2.y),2));
}

// Function to calculate the distance ratio
double calculate_distance_ratio(int pixel_distance, double real_distance) {
    return real_distance / pixel_distance;
}

// Function to calculate the average distance ratio
double calculate_average_ratio(int d1, double real_distance1, int d2, double real_distance2, int d3, double real_distance3) {
    double distance_ratio1 = calculate_distance_ratio(d1, real_distance1);
    double distance_ratio2 = calculate_distance_ratio(d2, real_distance2);
    double distance_ratio3 = calculate_distance_ratio(d3, real_distance3);
    return (distance_ratio1 + distance_ratio2 + distance_ratio3) / 3;
}

// Function to convert pixel measurement to meters
double pixel_to_meters(int pixel_value, double distance_ratio) {
    return pixel_value * distance_ratio;
}

// Function to calculate the angle with the y-axis
double get_angle_with_y_axis(const std::array<double, 2>& heading_vector) {
    // Define the y-axis vector
    std::array<double, 2> y_axis = {0, 1};

    // Normalize the heading vector
    double magnitude = std::sqrt(std::pow(heading_vector[0], 2) + std::pow(heading_vector[1], 2));
    std::array<double, 2> heading_vector_normalized = {heading_vector[0] / magnitude, heading_vector[1] / magnitude};

    // Calculate the dot product
    double dot_product = heading_vector_normalized[0] * y_axis[0] + heading_vector_normalized[1] * y_axis[1];

    // Calculate the angle in radians
    double angle_rad = std::acos(std::min(std::max(dot_product, -1.0), 1.0));

    // Convert the angle to degrees
    double angle_rel_deg = angle_rad * 180.0 / M_PI;

    // Determine the direction of rotation using the cross product
    double cross_product = y_axis[0] * heading_vector[1] - y_axis[1] * heading_vector[0];
    if (cross_product < 0) {  // Clockwise rotation
        angle_rel_deg = 360 - angle_rel_deg;
    }

    if (angle_rel_deg > 180) {
        angle_rel_deg -= 360;
    }

    return angle_rel_deg;
}

int main(int argc, char * argv[])
{
  int      i = 0;
  int      index;
  int      blocks_copied;
  int      pixy_init_status;
  char     buf[128];
  double   d1 = 0,d2 = 0,d3 = 0;
  double   x_c = 0, y_c = 0;
  double   x_rel = 0, y_rel = 0;
  double   R_x = 0, R_y = 0;
  double   theta = 0;
  double   average_distance_ratio = 0;
  double   x_rel_meters = 0;
  double   y_rel_meters = 0;
  double   angle_rel_deg = 0;
  
  // Catch CTRL+C (SIGINT) signals //
  signal(SIGINT, handle_SIGINT);

  //printf("Hello Pixy:\n libpixyusb Version: %s\n", __LIBPIXY_VERSION__);

  // Connect to Pixy //
  pixy_init_status = pixy_init();

  // Was there an error initializing pixy? //
  if(!pixy_init_status == 0)
  {
    // Error initializing Pixy //
    printf("pixy_init(): ");
    pixy_error(pixy_init_status);

    return pixy_init_status;
  }

  // Request Pixy firmware version //
  {
    uint16_t major;
    uint16_t minor;
    uint16_t build;
    int      return_value;

    return_value = pixy_get_firmware_version(&major, &minor, &build);

    if (return_value) {
      // Error //
      printf("Failed to retrieve Pixy firmware version. ");
      pixy_error(return_value);

      return return_value;
    } else {
      // Success //
      printf(" Pixy Firmware Version: %d.%d.%d\n", major, minor, build);
    }
  }

  #if 0
  // Pixy Command Examples //
  {
    int32_t response;
    int     return_value;

    // Execute remote procedure call "cam_setAWB" with one output (host->pixy) parameter (Value = 1)
    //
    //   Parameters:                 Notes:
    //
    //   pixy_command("cam_setAWB",  String identifier for remote procedure
    //                        0x01,  Length (in bytes) of first output parameter
    //                           1,  Value of first output parameter
    //                           0,  Parameter list seperator token (See value of: END_OUT_ARGS)
    //                   &response,  Pointer to memory address for return value from remote procedure call
    //                           0); Parameter list seperator token (See value of: END_IN_ARGS)
    //

    // Enable auto white balance //
    pixy_command("cam_setAWB", UINT8(0x01), END_OUT_ARGS,  &response, END_IN_ARGS);

    // Execute remote procedure call "cam_getAWB" with no output (host->pixy) parameters
    //
    //   Parameters:                 Notes:
    //
    //   pixy_command("cam_setAWB",  String identifier for remote procedure
    //                           0,  Parameter list seperator token (See value of: END_OUT_ARGS)
    //                   &response,  Pointer to memory address for return value from remote procedure call
    //                           0); Parameter list seperator token (See value of: END_IN_ARGS)
    //

    // Get auto white balance //
    return_value = pixy_command("cam_getAWB", END_OUT_ARGS, &response, END_IN_ARGS);

    // Set auto white balance back to disabled //
    pixy_command("cam_setAWB", UINT8(0x00), END_OUT_ARGS,  &response, END_IN_ARGS);
  }
  #endif

  printf("Detecting blocks...\n");

  // Initialize the ir_test node
  ros::init(argc, argv, "ir_landing_data");

  // Create a handle to the arm_mover node
  ros::NodeHandle n;

  // Create a publisher that can publish a std_msgs::Float64MultiArray message on the /ir_link/landing_location topic
  ros::Publisher ir_landing_data_pub = n.advertise<std_msgs::Float64MultiArray>("/ir_link/landing_location", 100);

  // Set loop frequency of 10Hz
  ros::Rate loop_rate(10);

  while(run_flag && ros::ok())
  {
    // Wait for new blocks to be available //
    while(!pixy_blocks_are_new() && run_flag); 

    // Get blocks from Pixy //
    blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, &blocks[0]);

    if(blocks_copied < 0) {
      // Error: pixy_get_blocks //
      printf("pixy_get_blocks(): ");
      pixy_error(blocks_copied);
    }

    // Display received blocks //
    printf("\nframe %d:\n", i);
    
    for(index = 0; index != blocks_copied; ++index) {    
       blocks[index].print(buf);
       //printf("  %s\n", buf);
    }

    //Display coordinates of 3 beacons 
    for (int j = 0; j < BLOCK_BUFFER_SIZE; j++)
      //printf("\nx%d: %d  y%d: %d width: %d height: %d angle: %d",j,blocks[j].x,j,blocks[j].y,blocks[j].width,blocks[j].height,blocks[j].angle);

    //Calculate distance between 3 beacons  
    d1 = distance(blocks[0],blocks[1]);
    d2 = distance(blocks[0],blocks[2]);
    d3 = distance(blocks[1],blocks[2]);
    //Display the distance between the 3 beacons 
    //printf("\n\nD1: %.3f", d1);
    //printf("\nD2: %.3f", d2);
    //printf("\nD3: %.3f\n", d3);

    //Find the center point coordinates 
    x_c = (blocks[0].x + blocks[1].x + blocks[2].x)/3;
    y_c = (blocks[0].y + blocks[1].y + blocks[2].y)/3;  
    //printf("\nCenter point x: %.3f  y: %.3f\n",x_c,y_c);

    //Relative Distance from image center to triangle center 
    x_rel = x_c - (PIXY_MAX_X /2);
    y_rel = y_c - (PIXY_MAX_Y /2);
    //printf("Relative distance x: %.3f  y: %.3f\n",x_rel,y_rel);

    //Find the heading vector 
    if (d1 == d2)
    {
      //heading is blocks[0]
      R_x = blocks[0].x - x_c; 
      R_y = blocks[0].y - y_c;
      // Calculate the average distance ratio
      average_distance_ratio = calculate_average_ratio(d1, real_distance1, d2, real_distance2, d3, real_distance3);
    }
    else if (d2 == d3)
    {
      //heading is blocks[2]
      R_x = blocks[2].x - x_c; 
      R_y = blocks[2].y - y_c;

      // Calculate the average distance ratio
      average_distance_ratio = calculate_average_ratio(d3, real_distance1, d2, real_distance2, d1, real_distance3);
    }
    else if (d1 == d3)
    {
      //heading is blocks[1]
      R_x = blocks[1].x - x_c; 
      R_y = blocks[1].y - y_c;

      // Calculate the average distance ratio
      average_distance_ratio = calculate_average_ratio(d1, real_distance1, d3, real_distance2, d2, real_distance3);
    }
    //theta = atan2(R_y,R_x);
    //printf("Heading Vector (%.3f,%.3f) \n",R_x,R_y);

    // Convert x_rel and y_rel to meters
    x_rel_meters = pixel_to_meters(x_rel, average_distance_ratio);
    y_rel_meters = pixel_to_meters(y_rel, average_distance_ratio);

    // Calculate the angle relative to the y-axis
    std::array<double, 2> heading_vector = {R_x, R_y};
    angle_rel_deg = get_angle_with_y_axis(heading_vector);

    // Set the landing data
    std_msgs::Float64MultiArray landing_location_msg;
    landing_location_msg.data.clear();

    landing_location_msg.data.push_back(x_rel_meters);
    landing_location_msg.data.push_back(y_rel_meters);
    landing_location_msg.data.push_back(angle_rel_deg);

    // Publish the landing data
    ir_landing_data_pub.publish(landing_location_msg);

    // Sleep for the time remaining until 10 Hz is reached
    loop_rate.sleep();

    // Output results
    //std::cout << "x_rel in meters: " << x_rel_meters << " m" << std::endl;
    //std::cout << "y_rel in meters: " << y_rel_meters << " m" << std::endl;
    //std::cout << "angle_rel_deg: " << angle_rel_deg << " deg" << std::endl;
    std::cout << "[x_rel, y_rel, angle_rel]" <<std::endl;
    printf("[%.3f, %.3f, %.3f]\n",x_rel_meters,y_rel_meters,angle_rel_deg); 

    i++;
  }
  pixy_close();
}