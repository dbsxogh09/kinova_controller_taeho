#pragma once

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <stdint.h>

/* Typedefs used so integer sizes are more explicit */
typedef uint32_t uint32;
typedef int32_t int32;
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint8_t sensor_byte;
typedef struct response_struct {
	uint32 rdt_sequence;
	uint32 ft_sequence;
	uint32 status;
	int32 FTData[6];
} RESPONSE;

#define FT_PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define COMMAND_BIAS 0x0042 /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */

using namespace std;

class ATI_FT
{

private:
	int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
	struct sockaddr_in addr;	/* Address of Net F/T. */
	struct hostent *he;			/* Host entry for Net F/T. */
	sensor_byte request[8];			/* The request data sent to the Net F/T. */
	RESPONSE resp;				/* The structured response received from the Net F/T. */
	sensor_byte response[36];			/* The raw response data received from the Net F/T. */
	int i;						/* Generic loop/array index. */
	int err;					/* Error status of operations. */
	std::string AXES[6];	/* The names of the force and torque axes. */

	

public:
	Eigen::VectorXd bias;
	unsigned int count = 1;

	ATI_FT(const std::string& ipaddress);

	void init(uint16 command, uint32 num_samples);
	void set_bias(uint32 num_samples);

	Eigen::VectorXd get_ft_data();
	Eigen::VectorXd get_biased_ft_data();
};


//**********************************how to use**********************************//

// RDT_FT_Sensor rdt_ft_sensor("172.16.0.4");
// rdt_ft_sensor.Initialize_FT_Sensor(0x02, 0);
// std::vector<double> FT_data = rdt_ft_sensor.Receive_FT_Sensor_Data();
// for(int i=0; i<6; i++)
// {
// std::cout<<FT_data.at(i)<<std::endl;
// }

//******************************************************************************//