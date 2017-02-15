// file: listener.cpp
//
// LCM example program.
//
// compile with:
//  $ gcc -o listener listener.cpp -llcm
//
// On a system with pkg-config, you can also use:
//  $ gcc -o listener listener.cpp `pkg-config --cflags --libs lcm`

#include <stdio.h>

#include <lcm/lcm-cpp.hpp>
#include "kinect/depth_msg_t.hpp"
#include "PointCloudFactory.hpp"

int main(int argc, char** argv)
{
    lcm::LCM lcm;

    if(!lcm.good())
        return 1;

    PointCloudFactory factory;
    lcm.subscribe("DEPTH_IMAGE", &PointCloudFactory::ingestDepthImage, &factory);

    while(0 == lcm.handle());
    return 0;
}