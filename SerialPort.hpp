//
// Created by think on 2021/2/26.
//

#ifndef INSPIREHAND_SERIALPORT_HPP
#define INSPIREHAND_SERIALPORT_HPP

#include <boost/asio.hpp>

class SerialPort {
    typedef boost::system::error_code error_code;
    typedef boost::asio::io_service io_service;
    typedef boost::asio::serial_port serial_port;

    typedef serial_port::baud_rate baud_rate;       // class baud_rate
    typedef serial_port::flow_control::type flow_control_type; // class flow_control
    typedef serial_port::stop_bits::type stop_bits_type;       // class stop_bits
    typedef serial_port::parity::type parity_type;             // class parity
    typedef serial_port::character_size data_bits;  // class data_bits
    typedef serial_port::flow_control flow_control; // class flow_control
    typedef serial_port::stop_bits stop_bits;       // class stop_bits
    typedef serial_port::parity parity;             // class parity
public:
    SerialPort() = delete; //禁用default构造函数

    explicit SerialPort(const std::string &portName = "") :
            _portName(portName),
            _ioService(),
            _serialPort(_ioService) {
        if (!_portName.empty()) {
            _serialPort.open(portName, _errorCode);

            if (_errorCode) {
                std::cout << "Serial port \'" << _portName
                          << "\' can not open due to \'" << _errorCode.message() << "\'" << std::endl;
            }

        } else {
            std::cout << "Serial port name has not been specified."
                         " Please use SerialPort::open() to open the serial." << std::endl;
        }

    } //构造传入端口名，explicit禁用隐式转换

    inline void setPortName(const std::string &portName) { _portName = portName; }                //设置串口名
    inline void setBaudRate(unsigned int rate = 0) { _serialPort.set_option(baud_rate(rate)); }   //设置串口波特率
    inline void setFlowControl(flow_control_type flowControl = flow_control::none) { _serialPort.set_option(flowControl); } //设置流控制
    inline void setStopBits(stop_bits_type stopBits) { _serialPort.set_option(stopBits); }             //设置停止位
    inline void setParity(parity_type parityBit) { _serialPort.set_option(parityBit); }                //设置奇偶校验位
    inline void setDataBits(unsigned int dataBits) { _serialPort.set_option(data_bits(dataBits)); } //设置数据位

    inline std::string getPortName() { return _portName; }  //获取串口名
    inline unsigned int getBaudRate() {                     //获取串口波特率
        baud_rate baud;
        _serialPort.get_option(baud);
        return baud.value();
    }
    inline flow_control_type getFlowControl() {                  //获取流控制
        flow_control flow;
        _serialPort.get_option(flow);
        return flow.value();
    }
    inline parity_type getParity() {                            //获取奇偶校验
        parity par;
        _serialPort.get_option(par);
        return par.value();
    }
    inline stop_bits_type getStopBits() {                        //获取停止位
        stop_bits sb;
        _serialPort.get_option(sb);
        return sb.value();
    }
    inline unsigned int getDataBits() {                        //获取数据位
        data_bits db;
        _serialPort.get_option(db);
        return db.value();
    }




private:
    io_service _ioService; // 前摄器模式需定义io_service
    error_code _errorCode; // 错误代码
    serial_port _serialPort; //内部串口利用boost::asio::serial_port实现
    std::string _portName; //端口名称

};


#endif //INSPIREHAND_SERIALPORT_HPP
