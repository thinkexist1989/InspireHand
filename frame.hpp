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

#ifndef INSPIREHAND_FRAME_HPP
#define INSPIREHAND_FRAME_HPP


#define HEADER_REQ_1 0xEB
#define HEADER_REQ_2 0x90
#define HEADER_RET_1 0x90
#define HEADER_RET_2 0xEB


//#define pID     2               //id位置指针
//#define pLens   3               //frame lens位置指针
//#define pOrder   4               //命令
//#define pAddrL  5               //寄存器地址低
//#define pAddrH  6
//#define pData   7               //指向data区域的指针
//    uint8_t p_checksum;                          //校验位,就是最后一位 data_lens + 7


//因时机械手协议的寄存器地址 2 byte
struct Register {
public:
    enum {
        READ = 0x11, WRITE = 0x12
    }; //定义枚举类型表示读/写命令 the enum hack

    enum {
        REQUEST = 0, RETURN = 1, NONE = 2
    }; //定义枚举类型表示请求/返回帧 the enum hack

    /*****寄存器地址******/
    static const uint16_t ADDR_ID = 1000; //灵巧手ID, RW, 1 byte
    static const uint16_t ADDR_BUAD = 1001; //波特率设置, RW, 1 byte
    static const uint16_t ADDR_CLEAR_ERROR = 1004; //清除错误, RW, 1 byte
    static const uint16_t ADDR_SAVE_TO_FLASH = 1005; //保存数据至Flash, RW, 1 byte
    static const uint16_t ADDR_RESET_PARAM = 1006; //恢复出厂设置, RW, 1 byte
    static const uint16_t ADDR_GESTURE_NO_SET = 1008; //收拾序列设置, RW, 1 byte
    static const uint16_t ADDR_GESTURE_FORCE_CLB = 1009; //受力传感器校准, RW, 1 byte

    static const uint16_t ADDR_VOLTAGE = 1472; //系统电压, R, 1 short

    constexpr static inline const uint16_t ADDR_CURRENT_LIMIT(uint8_t m) { return 1020 + 2 * m; } //各驱动器电流保护值, RW, 6 short

    constexpr static inline const uint16_t ADDR_DEFAULT_SPEED_SET(uint8_t m) { return 1032 + 2 * m; } //各手指上电速度值, RW, 6 short

    constexpr static inline const uint16_t ADDR_DEFAULT_FORCE_SET(uint8_t m) { return 1044 + 2 * m; } //各手指上电力控阈值, RW, 6 short

    constexpr static inline const uint16_t ADDR_USER_DEF_ANGLE(uint8_t K, uint8_t m) { // 自定义手势角度值, RW, 32*6 short
        return K < 14 ? 1066 : (1066 + (K - 14) * 12 + m * 2);
    }

    constexpr static inline const uint16_t ADDR_POS_SET(uint8_t m) { return 1474 + 2 * m; } //各驱动器位置设置, RW, 6 short,
    constexpr static inline const uint16_t ADDR_ANGLE_SET(uint8_t m) { return 1486 + 2 * m; } //各手指角度设置, RW, 6 short
    constexpr static inline const uint16_t ADDR_FORCE_SET(uint8_t m) { return 1498 + 2 * m; } //各手指实际力控阈值, RW, 6 short
    constexpr static inline const uint16_t ADDR_SPEED_SET(uint8_t m) { return 1522 + 2 * m; } //各手指速度设置, RW, 6 short, 0-1000,
    constexpr static inline const uint16_t ADDR_POS_ACT(uint8_t m) {
        return 1534 + 2 * m;
    } //各驱动器实际位置（小拇指0）,R, 6 short, 0-2000, -1
    constexpr static inline const uint16_t ADDR_ANGLE_ACT(uint8_t m) {
        return 1546 + 2 * m;
    } //各手指实际角度（小拇指0）,R, 6 short, 0-1000, -1
    constexpr static inline const uint16_t ADDR_FORCE_ACT(uint8_t m) {
        return 1582 + 2 * m;
    } //各手指实际受力（小拇指0）,R, 6 short, 0-1000, -1
    constexpr static inline const uint16_t ADDR_CURRENT(uint8_t m) {
        return 1594 + 2 * m;
    } //各驱动器电流（小拇指0）, R, 6 short, 0-1000 mA, -1

    constexpr static inline const uint16_t ADDR_ERROR(uint8_t m) {
        return 1606 + m;
    } //各驱动器错误（小拇指0）, R, 6 byte, bit0-4=>堵转/过温/过流/电机/通信
    constexpr static inline const uint16_t ADDR_STATUS(uint8_t m) {
        return 1612 + m;
    } //各驱动器状态（小拇指0）, R, 6 byte, bit0-7=>松开/抓取/位置到/力控到/电流保护/堵转停/故障停
    constexpr static inline const uint16_t ADDR_TEMP(uint8_t m) { return 1618 + m; } //各驱动器温度（小拇指0）, R, 6 byte, 0-100 度

    /*****动作序列相关*******/
    static const uint16_t ADDR_ACTION_SEQ_CHECKDATA1 = 2000; // 当前动作序列校验码1（0x90）, RW, 1 byte
    static const uint16_t ADDR_ACTION_SEQ_CHECKDATA2 = 2001; // 当前动作序列校验码2（0xEB）, RW, 1 byte
    static const uint16_t ADDR_ACTION_SEQ_STEPNUM = 2002; // 当前动作序列总步骤数, RW, 1 byte
    constexpr static inline const uint16_t ADDR_ACTION_SEQ_STEP(uint8_t m) {
        return 2016 + 38 * m;
    } // 当前动作序列第m步(0-7), RW, 8*19short
    static const uint16_t ADDR_ACTION_SEQ_INDEX = 2320; // 当前动作序列索引号, RW, 1 byte
    static const uint16_t ADDR_ACTION_SEQ_SAVE = 2321; // 保存当前动作序列, RW, 1 byte
    static const uint16_t ADDR_ACTION_SEQ_RUN = 2322; // 运行当前动作序列, RW, 1 byte

};


// 帧格式：帧头(2) + ID(1) + 整个数据部分长度data_lens + 3(1) +
//        读/写命令(1) + 寄存器地址(2) + 数据长度(data_lens) + 校验位(1) = data_lens + 8

class ProtocolBase {
public:
    explicit ProtocolBase(uint8_t id, uint8_t order, uint16_t register_addr) :
            _id(id),
            _order(order),
            _register_address(register_addr) {
    }

protected:
    template<typename D>
    inline uint8_t getBinary(D t, size_t i) { //获取内存中存储数值
        return ((uint8_t *) &t)[i];
    }

    /// 计算校验和
    /// \param src 输入指针，计算从src[0]到src[n-1]的校验和
    /// \param n   计算的个数
    /// \return    返回uint8_t类型的校验位
    uint8_t getChecksum(uint8_t *src, size_t n) {
        uint8_t checksum = 0;
        for (int i = 0; i < n; i++) {
            checksum += src[i];
        }
        return checksum;
    }

    /// 生成符合协议的Buffer
    /// \param dest  传入的目的地buffer指针
    /// \param pData 传入的数据指针
    /// \param lens  数据长度
    void generateBuffer(uint8_t *dest, uint8_t *pData, size_t lens) {
        dest[0] = HEADER_REQ_1;
        dest[1] = HEADER_REQ_2;
        dest[2] = _id;
        dest[3] = lens + 3;
        dest[4] = _order;
        dest[5] = getBinary(_register_address, 0);
        dest[6] = getBinary(_register_address, 1);
        memcpy(&dest[7], pData, lens);
        dest[lens + 7] = getChecksum(&dest[2], lens + 5);
    }

    bool isValid(uint8_t *buf, size_t n) {
        if ((buf[0] != HEADER_RET_1) || (buf[1] != HEADER_RET_2)) {
            std::cout << "WARNING::ProtcolWrite::Frame header not suitable" << std::endl;
            return false;
        }
        if (n != (buf[3] + 5)) {
            std::cout << "WARNING::ProtocolWrite::Frame length is not suitable" << std::endl;
            return false;
        }

        if (buf[n - 1] != getChecksum(&buf[2], n - 3)) {
            std::cout << "WARNING::ProtocolWrite::Checksum is not suitable" << std::endl;
            return false;
        }

        return true;
    }

protected:
    uint8_t _id;
    uint8_t _order;
    uint16_t _register_address;
};

// 写入寄存器协议类
template<typename Tx, size_t num, uint16_t register_address>
struct ProtocolWrite : ProtocolBase {
public:
    explicit ProtocolWrite(uint8_t id = 0x01) : ProtocolBase(id, Register::WRITE , register_address) {
    }

    /// 得到写寄存器请求帧
    /// \param buffer 需要填充的数据帧
    /// \param data 需要写入寄存器的数据，其类型应和模板类型Tx对应
    /// \return 返回成功或失败
    bool getRequestBuffer(std::vector<uint8_t> &buffer, std::vector<Tx> &data) {
        if (data.size() != num) {
            std::cout << "ERROR::ProtocolWrite::getRequestBuffer::Data size is not available" << std::endl;
            return false;
        }

        size_t data_lens = sizeof(Tx) * num;
        buffer.resize(data_lens + 8);
        generateBuffer(&buffer[0], (uint8_t*)(&data[0]), data_lens);
        return true;
    }

    /// 解析写寄存器返回帧
    /// \param buffer 需要解析的数据帧，应为串口返回的数据帧
    /// \return 返回是否解析成功
    bool parseReturnBuffer(std::vector<uint8_t> &buffer) {
        if (!isValid(&buffer[0], buffer.size())) {
            std::cout << "ERROR::ProtocolWrite::Received Frame is not valid" << std::endl;
            return false;
        }

        if(buffer[7] == 0x01) {
            std::cout << "OK::ProtocolWrite::Write succeeded" << std::endl;
            return true;
        }
        else
            return false;

    }

};


// 读取寄存器协议类
template<typename Rx, size_t num, uint16_t register_address>
struct ProtocolRead : ProtocolBase {
public:
    explicit ProtocolRead(uint8_t id = 0x01) : ProtocolBase(id, Register::READ, register_address) {
    }

    //
    /// 得到读寄存器请求帧
    /// \param buffer 需要填充的数据帧，读取协议只需要发送需要读取的寄存器中数据长度byte
    /// \return 返回成功或失败
    bool getRequestBuffer(std::vector<uint8_t> &buffer) {
        uint8_t data = sizeof(Rx) * num ; //读取的话数据部分为需要读取的长度
        buffer.resize(9);
        generateBuffer(&buffer[0], &data, 1);
        return true;
    }

    /// 解析写寄存器返回帧
    /// \param buffer 需要解析的数据帧，应为串口返回的数据帧
    /// \param data 解析出的返回数据
    /// \return 返回是否解析成功
    bool parseReturnBuffer(std::vector<uint8_t> &buffer, std::vector<Rx> &data) {
        if (!isValid(&buffer[0], buffer.size())) {
            std::cout << "ERROR::ProtocolRead::Received Frame is not valid" << std::endl;
            return false;
        }

        size_t data_lens = sizeof(Rx) * num;
        if(buffer[3] !=  data_lens + 3) {
            std::cout << "ERROR::ProtocolRead::Received Data is not suitable" << std::endl;
            return false;
        }

        data.resize(num);
        memcpy(&data[0], &buffer[7], data_lens);
        return true;
    }

};


typedef ProtocolRead<uint8_t, 1, Register::ADDR_ID> ProtoGetID; // 读取ID
typedef ProtocolWrite<uint8_t, 1, Register::ADDR_ID> ProtoSetID; // 设置ID

typedef ProtocolRead<uint8_t, 1, Register::ADDR_BUAD> ProtoGetBaud; // 读取波特率
typedef ProtocolWrite<uint8_t, 1, Register::ADDR_BUAD> ProtoSetBaud; // 设置波特率

typedef ProtocolWrite<uint8_t, 1, Register::ADDR_CLEAR_ERROR> ProtoClearError; // 清除错误

typedef ProtocolWrite<uint8_t, 1, Register::ADDR_SAVE_TO_FLASH> ProtoSaveToFlash; // 保存数据到Flash

typedef ProtocolWrite<uint8_t, 1, Register::ADDR_RESET_PARAM> ProtoResetParam; // 恢复出厂设置

typedef ProtocolRead<uint16_t, 1, Register::ADDR_VOLTAGE> ProtoGetVoltage; // 系统电压

typedef ProtocolWrite<uint16_t, 6, Register::ADDR_POS_SET(0)> ProtoSetDriverPos; // 设置驱动器位置
typedef ProtocolRead<uint16_t, 6, Register::ADDR_POS_ACT(0)>  ProtoGetDrivePos;  // 获取驱动器位置

typedef ProtocolWrite<uint16_t, 6, Register::ADDR_ANGLE_SET(0)> ProtoSetAngle; // 设置手指角度
typedef ProtocolRead<uint16_t, 6, Register::ADDR_ANGLE_ACT(0)> ProtoGetAngle;  // 获取手指角度

typedef ProtocolWrite<uint16_t, 6, Register::ADDR_FORCE_SET(0)> ProtoSetForce; // 设置手指角度
typedef ProtocolRead<uint16_t, 6, Register::ADDR_FORCE_ACT(0)> ProtoGetForce; // 获取手指角度

typedef ProtocolWrite<uint16_t, 6, Register::ADDR_SPEED_SET(0)> ProtoSetSpeed; // 设置速度

typedef ProtocolRead<uint16_t, 6, Register::ADDR_CURRENT(0)> ProtoGetDriverCurrent; // 各自由度的驱动器的电流值

typedef ProtocolRead<uint8_t, 6, Register::ADDR_ERROR(0)> ProtoGetDriverError; // 各自由度的驱动器的故障信息

typedef ProtocolRead<uint8_t, 6, Register::ADDR_STATUS(0)> ProtoGetState; // 各自由度的状态信息

typedef ProtocolRead<uint8_t, 6, Register::ADDR_TEMP(0)> ProtoGetTemp; // 各自由度的驱动器的温度

typedef ProtocolWrite<uint8_t, 1, Register::ADDR_ACTION_SEQ_CHECKDATA1> ProtoSetActionSeqCheck1; // 设置当前动作序列检验码1
typedef ProtocolWrite<uint8_t, 1, Register::ADDR_ACTION_SEQ_CHECKDATA2> ProtoSetActionSeqCheck2; // 设置当前动作序列检验码2

typedef ProtocolWrite<uint8_t, 1, Register::ADDR_ACTION_SEQ_INDEX> ProtoSetActionSeqIndex; // 当前动作序列索引号
typedef ProtocolWrite<uint8_t, 1, Register::ADDR_ACTION_SEQ_STEPNUM> ProtoSetActionSeqStepNum; // 当前动作序列总步骤数

typedef ProtocolWrite<uint16_t, 19, Register::ADDR_ACTION_SEQ_STEP(0)> ProtoSetActionSeqStep0; //当前动作序列第0 步设置
typedef ProtocolWrite<uint16_t, 19, Register::ADDR_ACTION_SEQ_STEP(1)> ProtoSetActionSeqStep1; //当前动作序列第1 步设置
typedef ProtocolWrite<uint16_t, 19, Register::ADDR_ACTION_SEQ_STEP(2)> ProtoSetActionSeqStep2; //当前动作序列第2 步设置
typedef ProtocolWrite<uint16_t, 19, Register::ADDR_ACTION_SEQ_STEP(3)> ProtoSetActionSeqStep3; //当前动作序列第3 步设置
typedef ProtocolWrite<uint16_t, 19, Register::ADDR_ACTION_SEQ_STEP(4)> ProtoSetActionSeqStep4; //当前动作序列第4 步设置
typedef ProtocolWrite<uint16_t, 19, Register::ADDR_ACTION_SEQ_STEP(5)> ProtoSetActionSeqStep5; //当前动作序列第5 步设置
typedef ProtocolWrite<uint16_t, 19, Register::ADDR_ACTION_SEQ_STEP(6)> ProtoSetActionSeqStep6; //当前动作序列第6 步设置
typedef ProtocolWrite<uint16_t, 19, Register::ADDR_ACTION_SEQ_STEP(7)> ProtoSetActionSeqStep7; //当前动作序列第7 步设置

typedef ProtocolWrite<uint8_t, 1, Register::ADDR_ACTION_SEQ_SAVE> ProtoSetActionSeqSave; // 保存当前动作序列
typedef ProtocolWrite<uint8_t, 1, Register::ADDR_ACTION_SEQ_RUN> ProtoSetActionSeqRun; // 运行当前动作序列


//class Protocol {
//public:
//    explicit Protocol(uint8_t id = 0x01) : _id(id) {
//    }
//
//    //// 获取帧头
//// Example:
////    auto header1 = getHeader(0);
////    auto header2 = getHeader(1);
//inline uint8_t getHeader(uint8_t i) { return i < 2 ? _buffer[i] : 0x00; }
//// 获取ID
//inline uint8_t getHandID() { return _buffer[p_hand_id]; }
////获取帧数据长度
//inline size_t getFrameDataLens() { return _buffer[p_frame_data_lens]; }
//inline size_t getFrameLens() { return _buffer[p_frame_data_lens] + 5; }
//inline size_t getDataLens() { return _buffer[p_frame_data_lens] - 3; }
////获取寄存器地址
//inline uint16_t getRegisterAddress() { return *((uint16_t*)(&_buffer[p_register_adderss])); }
//
//inline void setHeader(uint8_t h1, uint8_t h2) { _buffer[0] = h1; _buffer[1] = h2; }
//inline void setHandID(uint8_t ID) { _buffer[p_hand_id] = ID; }
//inline void setData(uint8_t* pData, size_t n) {
//memcpy(&_buffer[p_data], pData, n);
//_buffer[p_frame_data_lens] = static_cast<uint8_t>(n) + 3;
//}
//
//bool isFrameValid(uint8_t* pFrame) {
//    if((pFrame[0] == HEADER_REQ_1) && (pFrame[1] == HEADER_REQ_2)) { // REQUEST帧
//        type = TP::REQUEST;
//        return true;
//    }
//    else if((pFrame[0] == HEADER_RET_1) && (pFrame[1] == HEADER_RET_2)) { // RETURN帧
//        type = TP::RETURN;
//        return true;
//    }
//    else {
//        std::cout << "The frame header is not valid" << std::endl;
//        type = TP::NONE;
//        return false;
//    }
//}
//
//
//private:
//    uint8_t _id;
//};




#endif //INSPIREHAND_FRAME_HPP
