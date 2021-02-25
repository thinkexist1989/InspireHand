//
// Created by think on 2021/2/25.
//

#include "InspireHand.h"

class InspireHand::serialPort {

};

InspireHand::InspireHand() :
        sp(new serialPort) {

}

bool InspireHand::setAngle(int16_t h1, int16_t h2, int16_t h3, int16_t h4, int16_t h5, int16_t h6) {
    return false;
}