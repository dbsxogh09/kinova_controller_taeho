#include "kinova_controller/sensors/ati_ft.hpp"

ATI_FT::ATI_FT(const std::string& ipaddress) : i(0)
{

    /* Calculate number of samples, command code, and open socket here. */
    socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketHandle == -1) {
        exit(1);
    }

    /* Sending the request. */
    he = gethostbyname(ipaddress.c_str());
    memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(FT_PORT);

    err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
    if (err == -1) {
        exit(2);
    }

    AXES[0] = "Fx";
    AXES[1] = "Fy";
    AXES[2] = "Fz";
    AXES[3] = "Tx";
    AXES[4] = "Ty";
    AXES[5] = "Tz";
    
    bias.resize(6);
    bias.setZero();
}

void ATI_FT::init(uint16 command, uint32 num_samples)
{

    *(uint16*)&request[0] = htons(0x1234); /* standard header. */
    *(uint16*)&request[2] = htons(command); /* per table 9.1 in Net F/T user manual. */
    *(uint32*)&request[4] = htonl(num_samples); /* see section 9.1 in Net F/T user manual. */
    send( socketHandle, request, 8, 0 );
}
void ATI_FT::set_bias(uint32 num_samples)
{

    *(uint16*)&request[0] = htons(0x1234); /* standard header. */
    *(uint16*)&request[2] = htons(0x0042); /* per table 9.1 in Net F/T user manual. */
    *(uint32*)&request[4] = htonl(num_samples); /* see section 9.1 in Net F/T user manual. */
    send( socketHandle, request, 8, 0 );
}

Eigen::VectorXd ATI_FT::get_biased_ft_data()
{

    Eigen::VectorXd return_FT_data = this->get_ft_data();

    return_FT_data -= bias;

    return return_FT_data;
}

Eigen::VectorXd ATI_FT::get_ft_data()
{

    /* Receiving the response. */
    recv( socketHandle, response, 36, 0 );

    resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
    resp.ft_sequence = ntohl(*(uint32*)&response[4]);
    resp.status = ntohl(*(uint32*)&response[8]);
    for( i = 0; i < 6; i++ ) {
        resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
    }

    /* Output the response data. */
    // printf( "Status: 0x%08x\n", resp.status );
//		StatusCode[0] = resp.status;
    Eigen::VectorXd return_FT_data(6);
    return_FT_data.setZero();

    for (i =0;i < 6;i++) {
        return_FT_data(i) = (-(double)resp.FTData[i]*1e-5);
    }

    return return_FT_data;
}
