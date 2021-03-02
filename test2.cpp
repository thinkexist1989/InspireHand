//
// Created by think on 2021/2/25.
//

#include "inspirehand.hpp"

#define HAND_ID 0x01

int main(int argc, char** argv)
{
    InspireHand ih(HAND_ID, "COM3");
    int16_t angles[3] = {20,30,40};
//    ih.setAngle<1>(20);

    ih.setAngle(100,100,100,100,-1,-1);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    ih.setAngle(1000,0,0,1000,1000,1000);

//    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::vector<uint16_t> fingers;
    if(ih.getAngle(fingers)) {
        for(auto& v : fingers) {
            static int i = 0;
            std::cout << "finger " << i << " is: " << static_cast<int>(v) << std::endl;
        }
    }

    uint8_t id;
    ih.getHandID(id);



    return 0;
}