//
// Created by think on 2021/2/25.
//

#ifndef INSPIREHAND_H
#define INSPIREHAND_H

#include <memory>
#include <array>

#include <iostream>


class InspireHand {
public:
    InspireHand(); //构造函数
private:
    class serialPort;//不完整的内部串口类声明，为了封装所使用的串口类，Bridge模式
    std::shared_ptr<serialPort> sp; //shared_ptr成员变量，用于操作serialPort类

public:
    bool setAngle(int16_t f1, int16_t f2, int16_t f3, int16_t f4, int16_t f5, int16_t f6);

    template<int8_t n1>
    bool setAngle(int16_t f1) {
        std::array<int16_t, 6> angles = {-1};
        angles[n1] = f1;
        std::cout << "angles[" << static_cast<int>(n1) << "] = " << f1 << std::endl;
        return false;
    }

    template<int8_t n1, int8_t n2>
    bool setAngle(int16_t f1, int16_t f2) {
        std::array<int16_t, 6> angles = {-1};
        angles[n1] = f1; angles[n2] = f2;
        std::cout << "angles[" << static_cast<int>(n1) << "] = " << f1 << std::endl;
        std::cout << "angles[" << static_cast<int>(n2) << "] = " << f2 << std::endl;
        return false;
    }

    template<int8_t n1, int8_t n2, int8_t n3>
    bool setAngle(int16_t f1, int16_t f2, int16_t f3);

    template<int8_t n1, int8_t n2, int8_t n3, int8_t n4>
    bool setAngle(int16_t f1, int16_t f2, int16_t f3, int16_t f4);

    template<int8_t n1, int8_t n2, int8_t n3, int8_t n4, int8_t n5>
    bool setAngle(int16_t f1, int16_t f2, int16_t f3, int16_t f4, int16_t f5);

};


#endif //INSPIREHAND_H
