//
// Created by think on 2021/2/25.
//

#ifndef INSPIREHAND_H
#define INSPIREHAND_H

#include <memory>


class InspireHand {
public:
    InspireHand(); //构造函数
private:
    class serialPort;//不完整的内部串口类声明，为了封装所使用的串口类，Bridge模式
    std::shared_ptr<serialPort> sp; //shared_ptr成员变量，用于操作serialPort类

public:
    bool setAngle(int16_t h1, int16_t h2, int16_t h3, int16_t h4, int16_t h5, int16_t h6);

    template<typename... P>
    bool setAngle(P&&... p)

};


#endif //INSPIREHAND_H
