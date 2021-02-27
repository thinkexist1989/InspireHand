//
// Created by think on 2021/2/25.
//

#include "inspirehand.hpp"

#define HAND_ID 0x01

int main(int argc, char** argv)
{
    InspireHand ih(HAND_ID, "COM3");
    int16_t angles[3] = {20,30,40};
    ih.setAngle<1>(20);

    ih.setAngle(0,0,0,0,-1,-1);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    ih.setAngle(1000,0,0,1000,1000,1000);

    return 0;
}