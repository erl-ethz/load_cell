#include "ros/ros.h"
#include <string>
#include <iostream>
#include "std_msgs/String.h"
#include <serial/serial.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/WrenchStamped.h>
#include <stdexcept> 

using namespace std;

//////////////////// Global variables ////////////////////

// Serial port 
serial::Serial ser;
std::string port;

// Publishers and subscribers
ros::Publisher wrench_pub;

// Wrench msg 
geometry_msgs::Wrench wrench_msg;

/* Moving Average Filter variables
geometry_msgs::Wrench filtered_wrench_msg;
const int N;				  	
double forces_cum[3][N];
double torques_cum[3][N];
Eigen::Vector3f sum_forces;
Eigen::Vector3f sum_torques;
bool first_flag = true;*/

//////////////////// Functions ////////////////////

// Enable communication with the serial port
int enableCommunication()
{
	try
	{
		ser.setPort(port);
		ser.setBaudrate(460800);
		serial::Timeout tout = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(tout);
		if(!ser.isOpen())
			ser.open();
		return 1;
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}

	if(ser.isOpen())
	{
		ROS_INFO_STREAM("Serial Port initialized");
		return 1;
	}
	else
		return -1;
}

// Interrogate the sensor through serial port and acquire measurements
void readData()
{
	if(ser.available())
	{		
		ROS_INFO_STREAM("Reading from serial port");
		
		size_t bytes_wrote = ser.write("Cc1,1,1,4");
		size_t bytes_wrote2 = ser.write("R");
		// Read data from the sensor
		std_msgs::String result;
		result.data = ser.read(ser.available());

		try
		{
			cout << "Ouput data: " << result.data << "\n" << endl;
			
			// Split data string into separated values
			vector<string> results; 
			istringstream iss(result.data); 
			for(string s; iss >> s; ) 
    				results.push_back(s); 
			
			// Fill in the wrench msg
			wrench_msg.force.x  = std::stod(results[1]);
			wrench_msg.force.y  = std::stod(results[2]);
			wrench_msg.force.z  = std::stod(results[3]);
			wrench_msg.torque.x = std::stod(results[4]);
			wrench_msg.torque.y = std::stod(results[5]);
			wrench_msg.torque.z = std::stod(results[6]);
		}
		
		catch(const std::invalid_argument&)
		{
			cout << "Read value not correct." << endl;
		  
		}
		catch(const std::out_of_range&)
		{
		  	cout << "Out of range exception." << endl;
		}
	}
}

// Filtering force/torque sensor measurements 
/*
void filterData()
{
	int i, j;		
	// FIR: Moving Average Filtering
	if(first_flag)
	{
		for(i = 0; i < N; i++)
		{
			ros::spinOnce();
			// Fill the first N elements for the 3 forces and 3 torques
			for(j = 0; j < 3; j++)
			{
				forces_cum[j][i]  = raw_forces[j];
				torques_cum[j][i] = raw_torques[j];
				sum_forces[j]    += forces_cum[j][i];
				sum_torques[j]   += torques_cum[j][i];
			}
		}
		first_flag = false;
	}
	else
	{
		// Shift the moving average window
		for(i = 0; i < N-1; i++)
		{
			for(j = 0; j < 3; j++)
			{
				forces_cum[j][i]  = forces_cum[j][i+1];
				torques_cum[j][i] = torques_cum[j][i+1];
			}
		}
			
		ros::spinOnce();
		for(j = 0; j < 3; j++)
		{
			// Add new samples
			forces_cum[j][N-1]  = raw_forces[j];
			torques_cum[j][N-1] = raw_torques[j];
			// Update the sum
			sum_forces[j]      += forces_cum[j][N-1];
			sum_torques[j]     += torques_cum[j][N-1];
		}
	}
	// Publish converted measurements for visualization
	ft_filtered_msg.force.x  = sum_forces[0] / N;
	ft_filtered_msg.force.y  = sum_forces[1] / N; 
	ft_filtered_msg.force.z  = (sum_forces[2] / N) - CAGE_WEIGHT_OFFSET; 
	ft_filtered_msg.torque.x = sum_torques[0] / N;
	ft_filtered_msg.torque.y = sum_torques[1] / N;
	ft_filtered_msg.torque.z = sum_torques[2] / N;
	ft_filtered_pub.publish(ft_filtered_msg);

	// As the window will be shifted, remove the first elements
	for(j = 0; j < 3; j++)
	{
		sum_forces[j]  -= forces_cum[j][0];
		sum_torques[j] -= torques_cum[j][0];
	}

	// Compute force magnitude
	force_magnitude = sqrt(ft_filtered_msg.force.x*ft_filtered_msg.force.x + 
			       ft_filtered_msg.force.y*ft_filtered_msg.force.y + 
			       ft_filtered_msg.force.z*ft_filtered_msg.force.z);

	// Compute polar and azimuth angles
	polar_angle = acos(ft_filtered_msg.force.z/force_magnitude);
	azimuth_angle = atan2(ft_filtered_msg.force.y, ft_filtered_msg.force.x);
}*/

// Main function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "wrench_reader");
	ros::NodeHandle nh("~");

	if(nh.getParam("serial_port", port))
		std::cout << "Found parameter: " << port << std::endl;
   	else
   	{
		// default value if not found
		port = "/dev/tty";
		std::cout << "Parameter not found, set to default." << std::endl;
	}	
	
	ros::Rate loop(100);

	// err equal to 1 if the communication is established correctly
	int err = enableCommunication();

	// Publisher for the wrench values
	wrench_pub = nh.advertise<geometry_msgs::Wrench>("/wrench_measurements", 10);
	
	while(ros::ok())
	{	
		if(err == 1)
		{
			// Read from serial
			readData();
			
			// Publish and print wrench values
			wrench_pub.publish(wrench_msg);
		
			//cout << "Wrench published: " << wrench_msg << endl;
		}
		
		ros::spinOnce();	
		loop.sleep();       
	}
	return 0;
}

