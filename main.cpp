//
// Created by think on 2021/2/9.
//

#include <boost/asio.hpp>

#include <iostream>

using namespace boost::asio;

const char HEADER1 = 0xEB;
const char HEADER2 = 0x90;
const char HAND_ID = 0x01;

template<typename T>
uint8_t getBinary(T t, int i)
{
    return ((char*)&t)[i];
}

std::vector<uint8_t> setAngle(uint16_t h1, uint16_t h2, uint16_t h3, uint16_t h4, uint16_t h5, uint16_t h6)
{
    uint8_t checksum = 0;
    uint8_t dataLens = 2*6 + 3;
    std::vector<uint8_t> buffer;
    buffer.push_back(HEADER1); buffer.push_back(HEADER2); //帧头

    buffer.push_back(HAND_ID); //ID
    checksum += HAND_ID;
    buffer.push_back(dataLens); //长度
    checksum += dataLens;
    buffer.push_back(0x12); //写寄存器命令
    checksum += 0x12;
    buffer.push_back(0xCE); buffer.push_back(0x05);
    checksum += 0xCE + 0x05;
    buffer.push_back(getBinary(h1, 0)); buffer.push_back(getBinary(h1, 1));
    checksum += getBinary(h1, 0) + getBinary(h1, 1);
//    std::cout << std::hex << (int)getBinary(h1, 0) << (int)getBinary(h1, 1) << std::endl;
    buffer.push_back(getBinary(h2, 0)); buffer.push_back(getBinary(h2, 1));
    checksum += getBinary(h2, 0) + getBinary(h2, 1);;
    buffer.push_back(getBinary(h3, 0)); buffer.push_back(getBinary(h3, 1));
    checksum += getBinary(h3, 0) + getBinary(h3, 1);;
    buffer.push_back(getBinary(h4, 0)); buffer.push_back(getBinary(h4, 1));
    checksum += getBinary(h4, 0) + getBinary(h4, 1);;
    buffer.push_back(getBinary(h5, 0)); buffer.push_back(getBinary(h5, 1));
    checksum += getBinary(h5, 0) + getBinary(h5, 1);;
    buffer.push_back(getBinary(h6, 0)); buffer.push_back(getBinary(h6, 1));
    checksum += getBinary(h6, 0) + getBinary(h6, 1);;
    buffer.push_back(checksum);

    std::cout << std::hex << (int)checksum << std::endl;

    return  buffer;

}

int main(int argc, char** argv)
{
    io_service ioService;
    boost::system::error_code ec;
    serial_port serialPort(ioService);
    serialPort.open("COM3", ec);

    if(ec) {
        std::cout << "Can not open serial port, error reason: " << ec.message() << std::endl;
        return -1;
    }

    serialPort.set_option(serial_port::baud_rate(115200)); //波特率15200
    serialPort.set_option(serial_port::flow_control(serial_port::flow_control::none)); //无流控制
    serialPort.set_option(serial_port::parity(serial_port::parity::none)); //无奇偶校验
    serialPort.set_option(serial_port::stop_bits(serial_port::stop_bits::one)); //1位奇偶校验
    serialPort.set_option(serial_port::character_size(8)); //数据位8位

    std::array<char, 20> array;
    array[0] = 0xEB; array[1] = 0x90;
    array[2] = 0x01;
    array[3] = 0x0F;
    array[4] = 0x12;
    array[5] = 0xCE; array[6] = 0x05;
    array[7] = 0x64; array[8] = 0x00;
    array[9] = 0x64; array[10] = 0x00;
    array[11] = 0x64;array[12] = 0x00;
    array[13] = 0x64; array[14] = 0x00;
    array[15] = 0xD0; array[16] = 0x07;
    array[17] = 0x00; array[18] = 0x00;
    array[19] = 0x5C;

    uint16_t t = 100;
    std::cout << (int)getBinary(t, 0) << " " << (int)getBinary(t, 1) << std::endl;

    write(serialPort, buffer(array));

    std::this_thread::sleep_for(std::chrono::seconds(1));

    write(serialPort, buffer(setAngle(1000,0,0,1000,1000,1000)));

    ioService.run();

    return 0;
}

