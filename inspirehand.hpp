// Copyright 2021, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#ifndef INSPIREHAND_INSPIREHAND_H
#define INSPIREHAND_INSPIREHAND_H

#define DEFAULT_ID 0x01 //默认灵巧手ID

#include <memory>
#include <array>

#include <iostream>

#include "serialport.hpp"
#include "frame.hpp"

#include <boost/assign.hpp>

using namespace boost::assign;

class InspireHand {

public:
    InspireHand(int8_t id = 0x01, const std::string &serialPortName = "COM3") :
            _id(id),
            _sp(new serialport(serialPortName)) {

        setBaudRate();
        setFlowControl();
        setStopBits();
        setParity();
        setDataBits();

    } //构造函数

public:
    inline void setPortName(const std::string &portName = "COM3") { _sp->setPortName(portName); }                               //设置串口名
    inline void setBaudRate(unsigned int rate = 115200) { _sp->setBaudRate(rate); }                                             //设置串口波特率
    inline void setFlowControl(serialport::flow_control_type fc = serialport::flow_control::none) { _sp->setFlowControl(fc); }  //设置流控制
    inline void setStopBits(serialport::stop_bits_type stopBits = serialport::stop_bits::one) { _sp->setStopBits(stopBits); }   //设置停止位
    inline void setParity(serialport::parity_type parityBit = serialport::parity::none) { _sp->setParity(parityBit); }          //设置奇偶校验位
    inline void setDataBits(unsigned int dataBits = 8) { _sp->setDataBits(dataBits); }                                          //设置数据位

    bool getAngle(std::vector<uint16_t> &fingers) {
//        fingers.clear();
        ProtoGetAngle p(DEFAULT_ID);
        std::vector<uint8_t> sendBuf;
        p.getRequestBuffer(sendBuf);
        if(!send(sendBuf)) {
            return false;
        }

        std::vector<uint8_t> recvBuf(12 + 8);
        receive(recvBuf, recvBuf.size());

        if(!p.parseReturnBuffer(recvBuf, fingers)) {
            std::cout << "ERROR::InspireHand::GetAngle::Return false" << std::endl;
            return false;
        }

        return true;
    }


    bool setAngle(std::vector<uint16_t> &fingers) {
        if (fingers.size() != 6) {
            std::cout << "ERROR::InspireHand::setAngle::Data incomplete" << std::endl;
            return false;
        }
        ProtoSetAngle p(DEFAULT_ID);
        std::vector<uint8_t> sendBuf;
        p.getRequestBuffer(sendBuf, fingers);
        if(!send(sendBuf)) {
            return false;
        }

        std::vector<uint8_t> recvBuf(1 + 8);
        receive(recvBuf, recvBuf.size());

        if(!p.parseReturnBuffer(recvBuf)) {
            std::cout << "ERROR::InspireHand::setAngle::Return false" << std::endl;
            return false;
        }

        return true;
    }

    bool setAngle(int16_t f1, int16_t f2, int16_t f3, int16_t f4, int16_t f5, int16_t f6) {
        std::vector<uint16_t> fingers;
        fingers += f1, f2, f3, f4, f5, f6;

        return setAngle(fingers);
    }

    template<int8_t n1>
    bool setAngle(int16_t f1) {
        std::vector<int16_t> angles = {6 , -1};
        angles[n1] = f1;
        std::cout << "angles[" << static_cast<int>(n1) << "] = " << f1 << std::endl;
        return setAngle(angles);
    }

    template<int8_t n1, int8_t n2>
    bool setAngle(int16_t f1, int16_t f2) {
        std::array<int16_t, 6> angles = {-1};
        angles[n1] = f1;
        angles[n2] = f2;
        std::cout << "angles[" << static_cast<int>(n1) << "] = " << f1 << std::endl;
        std::cout << "angles[" << static_cast<int>(n2) << "] = " << f2 << std::endl;
        return setAngle(angles);
    }

    template<int8_t n1, int8_t n2, int8_t n3>
    bool setAngle(int16_t f1, int16_t f2, int16_t f3);

    template<int8_t n1, int8_t n2, int8_t n3, int8_t n4>
    bool setAngle(int16_t f1, int16_t f2, int16_t f3, int16_t f4);

    template<int8_t n1, int8_t n2, int8_t n3, int8_t n4, int8_t n5>
    bool setAngle(int16_t f1, int16_t f2, int16_t f3, int16_t f4, int16_t f5);

private:
//    template<typename T>
//    //获取变量在内存中的存储值
//    uint8_t getBinary(T t, int i) { return ((char *) &t)[i]; }

private:
    std::shared_ptr<serialport> _sp; //shared_ptr成员变量，用于操作SerialPort类
    std::int8_t _id = DEFAULT_ID;          //灵巧手id

    bool send(std::vector<uint8_t>& buf) { //发送指定字节的数据
        size_t ret = _sp->write(buf);
        if(ret != buf.size()) {
            std::cout << "ERROR::InspireHand::send::Do not send complete data!" << std::endl;
            return false;
        }

        return true;
    }

    bool receive(std::vector<uint8_t>& buf, size_t n) { // 接收指定字节的数据
        size_t ret = _sp->read(buf, n);
        if(ret != n) {
            std::cout << "ERROR::InspireHand::receive::Do not receive complete data!" << std::endl;
            return false;
        }

        return true;
    }
};


#endif //INSPIREHAND_INSPIREHAND_H
