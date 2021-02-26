//
// Created by think on 2021/2/25.
//

#ifndef INSPIREHAND_INSPIREHAND_H
#define INSPIREHAND_INSPIREHAND_H

#define HEADER1 0xEB  // 帧头1
#define HEADER2 0x90  // 帧头2
#define DEFAULT_ID 0x01 //默认灵巧手ID

#include <memory>
#include <array>

#include <iostream>

#include "SerialPort.hpp"

class InspireHand {
    static const uint16_t ADDR_ID = 1000; //灵巧手ID, RW, 1 byte
    static const uint16_t ADDR_BUAD = 1001; //波特率设置, RW, 1 byte
    static const uint16_t ADDR_CLEAR_ERROR = 1004; //清除错误, RW, 1 byte
    static const uint16_t ADDR_SAVE_TO_FLASH = 1005; //保存数据至Flash, RW, 1 byte
    static const uint16_t ADDR_RESET_PARAM = 1006; //恢复出厂设置, RW, 1 byte
    static const uint16_t ADDR_GESTURE_NO_SET = 1008; //收拾序列设置, RW, 1 byte
    static const uint16_t ADDR_GESTURE_FORCE_CLB = 1009; //受力传感器校准, RW, 1 byte

    static const uint16_t ADDR_VOLTAGE = 1472; //系统电压, R, 1 short

    static inline const uint16_t ADDR_CURRENT_LIMIT(uint8_t m) { return 1020 + 2 * m; } //各驱动器电流保护值, RW, 6 short

    static inline const uint16_t ADDR_DEFAULT_SPEED_SET(uint8_t m) { return 1032 + 2 * m } //各手指上电速度值, RW, 6 short

    static inline const uint16_t ADDR_DEFAULT_FORCE_SET(uint8_t m) { return 1044 + 2 * m } //各手指上电力控阈值, RW, 6 short

    static inline const uint16_t ADDR_USER_DEF_ANGLE(uint8_t K, uint8_t m) { // 自定义手势角度值, RW, 32*6 short
        return K < 14 ? 1066 : (1066 + (K - 14) * 12 + m * 2);
    }

    static inline const uint16_t ADDR_POS_SET(uint8_t m) { return 1474 + 2 * m; } //各驱动器位置设置, RW, 6 short,
    static inline const uint16_t ADDR_ANGLE_SET(uint8_t m) { return 1486 + 2 * m; } //各手指角度设置, RW, 6 short
    static inline const uint16_t ADDR_FORCE_SET(uint8_t m) { return 1498 + 2 * m; } //各手指实际力控阈值, RW, 6 short
    static inline const uint16_t ADDR_SPEED_SET(uint8_t m) { return 1522 + 2 * m; } //各手指速度设置, RW, 6 short, 0-1000,
    static inline const uint16_t ADDR_POS_ACT(uint8_t m) {
        return 1534 + 2 * m;
    } //各驱动器实际位置（小拇指0）,R, 6 short, 0-2000, -1
    static inline const uint16_t ADDR_ANGLE_ACT(uint8_t m) {
        return 1546 + 2 * m;
    } //各手指实际角度（小拇指0）,R, 6 short, 0-1000, -1
    static inline const uint16_t ADDR_FORCE_ACT(uint8_t m) {
        return 1582 + 2 * m;
    } //各手指实际受力（小拇指0）,R, 6 short, 0-1000, -1
    static inline const uint16_t ADDR_CURRENT(uint8_t m) {
        return 1594 + 2 * m;
    } //各驱动器电流（小拇指0）, R, 6 short, 0-1000 mA, -1

    static inline const uint16_t ADDR_ERROR(uint8_t m) {
        return 1606 + m;
    } //各驱动器错误（小拇指0）, R, 6 byte, bit0-4=>堵转/过温/过流/电机/通信
    static inline const uint16_t ADDR_STATUS(uint8_t m) {
        return 1612 + m;
    } //各驱动器状态（小拇指0）, R, 6 byte, bit0-7=>松开/抓取/位置到/力控到/电流保护/堵转停/故障停
    static inline const uint16_t ADDR_TEMP(uint8_t m) { return 1618 + m; } //各驱动器温度（小拇指0）, R, 6 byte, 0-100 度

    /*****动作序列相关*******/
    static const uint16_t ADDR_ACTION_SEQ_CHECKDATA1 = 2000; // 当前动作序列校验码1（0x90）, RW, 1 byte
    static const uint16_t ADDR_ACTION_SEQ_CHECKDATA2 = 2001; // 当前动作序列校验码2（0xEB）, RW, 1 byte
    static const uint16_t ADDR_ACTION_SEQ_STEPNUM = 2002; // 当前动作序列总步骤数, RW, 1 byte
    static inline const uint16_t ADDR_ACTION_SEQ_STEP(uint8_t m) {
        return 2016 + 38 * m;
    } // 当前动作序列第m步(0-7), RW, 8*19short
    static const uint16_t ADDR_ACTION_SEQ_INDEX = 2320; // 当前动作序列索引号, RW, 1 byte
    static const uint16_t ADDR_ACTION_SEQ_SAVE = 2321; // 保存当前动作序列, RW, 1 byte
    static const uint16_t ADDR_ACTION_SEQ_RUN = 2322; // 运行当前动作序列, RW, 1 byte

    enum {
        READ = 0x11, WRITE = 0x12
    }; //定义枚举类型表示读/写命令 the enum hack

    enum {
        REQUEST = 0, RETURN = 1
    }; //定义枚举类型表示请求/返回帧 the enum hack

    //帧格式：帧头(2) + ID(1) + 整个数据部分长度data_lens + 3(1) + 读/写命令(1) + 寄存器地址(2) + 数据长度(data_lens) + 校验位(1) = data_lens + 8
//    template<int16_t register_adderss, size_t data_lens, uint8_t type = REQUEST, uint8_t order = READ, uint8_t id = 0x01>
    template<typename T, size_t data_lens, uint8_t type = REQUEST>
    class frame_base {
    public:
        frame_base(int16_t register_address = ADDR_ID, uint8_t order = READ, uint8_t id = DEFAULT_ID) :
                _frame_header1(type ? HEADER2 : HEADER1), //请求帧帧头 0xEB 0x90
                _frame_header2(type ? HEADER1 : HEADER2), //返回帧帧头 0x90 0xEB
                _hand_id(id),
                _frame_data_lens(data_lens + 3),
                _order(order),
                _register_adderss(register_address),
                _data(std::make_shared<std::vector<uint8_t>>(data_lens / sizeof(T), 0)),
                _checksum(0x00) {

        }

        template<typename D>
        uint8_t getBinary(D t, size_t i) { //获取内存中存储数值
            return ((uint8_t *) &t)[i];
        }

        void getFrame(std::vector<uint8_t> &buffer) {
            buffer.clear(); //清除传入的buffer
            uint8_t checksum = 0;
            buffer.push_back(_frame_header1);
            buffer.push_back(_frame_header2);
            buffer.push_back(_hand_id);
            checksum += _hand_id;
            buffer.push_back(_frame_data_lens);
            checksum += _frame_data_lens;
            buffer.push_back(_order);
            checksum += _order;
            buffer.push_back(getBinary(_register_adderss, 0));
            checksum += getBinary(_register_adderss, 0);
            buffer.push_back(getBinary(_register_adderss, 1));
            checksum += getBinary(_register_adderss, 1);

            for (auto &v : *_data) {
                for (int i = 0; i < sizeof(T); i++) {
                    buffer.push_back(getBinary(v, i));
                    checksum += getBinary(v, i);
                }
            }

            buffer.push_back(checksum);
            if (_checksum != checksum)
                std::cout << "Checksum has been changed to " << std::hex << (int) checksum << std::endl;

            _checksum = checksum;

        }

    private:
        uint8_t _frame_header1;
        uint8_t _frame_header2;
        uint8_t _hand_id;                            //id
        uint8_t _frame_data_lens;                    //不同于data_lens, 这个_frame_data_lens = data_lens + 3
        uint8_t _order;                              //命令
        uint16_t _register_adderss;                  //寄存器地址
        std::shared_ptr<std::vector<T>> _data; //指向data区域的指针
        uint8_t _checksum;                           //校验位



    }; // frame基类

    template<uint8_t type = REQUEST>
    class frame_id : public frame_base<uint8_t, 1, type> {
    public:
        explicit frame_id(int16_t register_address = ADDR_ID, uint8_t order = READ, uint8_t id = DEFAULT_ID) :
                frame_base<uint8_t, 1, type>(register_address, order, id) {

        }
    }; // ID

    template<uint8_t type = REQUEST>
    class frame_baud : public frame_base<uint8_t, 1, type> {
    }; // 波特率设置

//    struct frame_clear_error : public frame_base {}; // 清楚错误
//    struct frame_save_to_flash : public frame_base {}; // 保存数据至flash
//    struct frame_reset_param: public frame_base {}; //恢复出厂设置
//    struct frame_gesture_no_set : public frame_base {}; //手势序列设置
//    struct frame_force_clb : public frame_base {}; //受力传感器校准
//    struct frame_angles : public frame_base {};
//    struct frame_forces : public frame_base {};

public:
    InspireHand(int8_t id = 0x01, const std::string &serialPortName = "") :
            _id(id),
            _sp(new SerialPort(serialPortName)) {

    } //构造函数

public:
    bool setAngle(std::vector<std::int16_t> &fingers) {

    }

    bool setAngle(int16_t f1, int16_t f2, int16_t f3, int16_t f4, int16_t f5, int16_t f6) {
        uint8_t checksum = 0;
        uint8_t dataLens = 2 * 6 + 3;
        std::vector<uint8_t> buffer;
        buffer.push_back(HEADER1);
        buffer.push_back(HEADER2); //帧头

        buffer.push_back(_id); //ID
        checksum += _id;
        buffer.push_back(dataLens); //长度
        checksum += dataLens;
        buffer.push_back(0x12); //写寄存器命令
        checksum += 0x12;
        buffer.push_back(0xCE);
        buffer.push_back(0x05);
        checksum += 0xCE + 0x05;
        buffer.push_back(getBinary(f1, 0));
        buffer.push_back(getBinary(f1, 1));
        checksum += getBinary(f1, 0) + getBinary(f1, 1);
//    std::cout << std::hex << (int)getBinary(h1, 0) << (int)getBinary(h1, 1) << std::endl;
        buffer.push_back(getBinary(f2, 0));
        buffer.push_back(getBinary(f2, 1));
        checksum += getBinary(f2, 0) + getBinary(f2, 1);;
        buffer.push_back(getBinary(f3, 0));
        buffer.push_back(getBinary(f3, 1));
        checksum += getBinary(f3, 0) + getBinary(f3, 1);;
        buffer.push_back(getBinary(f4, 0));
        buffer.push_back(getBinary(f4, 1));
        checksum += getBinary(f4, 0) + getBinary(f4, 1);;
        buffer.push_back(getBinary(f5, 0));
        buffer.push_back(getBinary(f5, 1));
        checksum += getBinary(f5, 0) + getBinary(f5, 1);;
        buffer.push_back(getBinary(f6, 0));
        buffer.push_back(getBinary(f6, 1));
        checksum += getBinary(f6, 0) + getBinary(f6, 1);;
        buffer.push_back(checksum);

        std::cout << std::hex << (int) checksum << std::endl;

//        return  buffer;
        return false;
    }

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
        angles[n1] = f1;
        angles[n2] = f2;
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

private:
    template<typename T>
    //获取变量在内存中的存储值
    uint8_t getBinary(T t, int i) { return ((char *) &t)[i]; }

private:

    std::shared_ptr<SerialPort> _sp; //shared_ptr成员变量，用于操作SerialPort类
    std::int8_t _id = DEFAULT_ID;          //灵巧手id

};


#endif //INSPIREHAND_INSPIREHAND_H
