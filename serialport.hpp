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

#ifndef INSPIREHAND_SERIALPORT_HPP
#define INSPIREHAND_SERIALPORT_HPP

#include <boost/asio.hpp>

class serialport {
public:
    typedef boost::system::error_code   error_code;
    typedef boost::asio::io_service     io_service;
    typedef boost::asio::serial_port    serial_port;

    typedef serial_port::baud_rate baud_rate;       // class baud_rate
    typedef serial_port::flow_control::type flow_control_type; // class flow_control
    typedef serial_port::stop_bits::type stop_bits_type;       // class stop_bits
    typedef serial_port::parity::type parity_type;             // class parity
    typedef serial_port::character_size data_bits;  // class data_bits
    typedef serial_port::flow_control flow_control; // class flow_control
    typedef serial_port::stop_bits stop_bits;       // class stop_bits
    typedef serial_port::parity parity;             // class parity
public:
    serialport() = delete; //禁用default构造函数

    explicit serialport(const std::string &portName = "") :
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
    inline void setFlowControl(flow_control_type fc = flow_control::none) { _serialPort.set_option(flow_control(fc)); } //设置流控制
    inline void setStopBits(stop_bits_type stopBits) { _serialPort.set_option(stop_bits(stopBits)); }             //设置停止位
    inline void setParity(parity_type parityBit) { _serialPort.set_option(parity(parityBit)); }                //设置奇偶校验位
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

    bool open() {
        _serialPort.open(_portName, _errorCode);
        if(_errorCode) {
            std::cout << "Open serial port failed! Reason: " << _errorCode.message() << std::endl;
            return false;
        }
        return true;
    }

    inline size_t write(std::vector<uint8_t>& buf) {
        return _serialPort.write_some(boost::asio::buffer(buf), _errorCode);
    }

    inline size_t write(uint8_t *buf, size_t n) {
        return _serialPort.write_some(boost::asio::buffer(buf, n), _errorCode);
    }

    inline size_t read(std::vector<uint8_t>& buf, size_t n) {
        return _serialPort.read_some(boost::asio::buffer(buf, n), _errorCode);
    }

    inline size_t read(uint8_t *buf, size_t n) {
        return _serialPort.read_some(boost::asio::buffer(buf, n), _errorCode);
    }





private:
    io_service _ioService; // 前摄器模式需定义io_service
    error_code _errorCode; // 错误代码
    serial_port _serialPort; //内部串口利用boost::asio::serial_port实现
    std::string _portName; //端口名称

};


#endif //INSPIREHAND_SERIALPORT_HPP
