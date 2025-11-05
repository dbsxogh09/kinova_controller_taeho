#pragma once

#include <Eigen/Dense>
#include <math.h>

// Joiint information //
// Joint 1: continuous       -limit in rad,   -limit in deg,
// Joint 2: revolute, limits: [-2.41, 2.41]    [-138, 138]
// Joint 3: continuous
// Joint 4: revolute, limits: [-2.66, 2.66]    [-152, 152]
// Joint 5: continuous
// Joint 6: revolute, limits: [-2.33, 2.33]    [-134, 134]
// Joint 7: continuous

//Convert law (feedback in degree) (output in degree) (feedback in degree) (output in degree)
//Joint 2:        [222~359.99]   ->    [-138 ~ -0.01]    [0.01 ~ 138]    ->   [0.01 ~ 138]
//Joint 4:        [208~359.99]   ->    [-152 ~ -0.01]    [0.01 ~ 152]    ->   [0.01 ~ 152]
//Joint 6:        [226~359.99]   ->    [-134 ~ -0.01]    [0.01 ~ 134]    ->   [0.01 ~ 134]
//                             [-360]                                  [nothing]
class ConvertJointPosition
{
    public:
    ConvertJointPosition(){};

    // input: joint_positions in radian
    // output: converted joint_positions in radian
    void convert(Eigen::VectorXd& joint_positions)
    {
        for(unsigned int i=0; i<7; i++)
        {
            if(is_limit[i])
            {
                if(joint_positions(i) > M_PI)
                {
                    joint_positions(i) -= 2*M_PI;
                }
            }
        }
    }

    private:
    std::array<int, 7> is_limit = {0,1,0,1,0,1,0};

};