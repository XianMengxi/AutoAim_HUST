//
// Created by zhiyu on 2021/8/20.
//

#include "SerialPort.h"
#include <iostream>
#include <thread>
namespace ly
{
    std::chrono::steady_clock::time_point TX2_BeginTime; //收到的第一帧单片机时间戳作为辅瞄代码起始时间戳
    int MCU_BeginTime;
    void *read_data(void *params_p)
    {
        auto params = reinterpret_cast<Params_ToSerialPort *>(params_p);
        auto serial_port = reinterpret_cast<SerialPort *>(params->__this);
        while (true)
        {
            try
            {
                SerialPortData imu_data;
                serial_port->readData(&imu_data);
                params->serial_data_queue->push(imu_data);
            }
            catch (...)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(ThreadDelay::serial_thread_delay));
        }
        return static_cast<void *>(0);
    }

    SerialPort::SerialPort()
    {
        new (this) SerialPort("/dev/ttyUSB0");
    }

    SerialPort::SerialPort(const string &port_name)
    {
        try
        {
            this->_serial_port = new serial_port(_io_service, port_name);
            this->_serial_port->set_option(serial_port::baud_rate(115200));
            this->_serial_port->set_option(serial_port::flow_control(serial_port::flow_control::none));
            this->_serial_port->set_option(serial_port::parity(serial_port::parity::none));
            this->_serial_port->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
            this->_serial_port->set_option(serial_port::character_size(8));

            _data_tmp = (uint8_t *)malloc((size_t)_data_len);
            pingpong = (uint8_t *)malloc((size_t)_data_len * 2);
        }
        catch (...)
        {
            LOG(ERROR) << "create serial port object error! ";
            is_start = false;
        }
    }

    SerialPort::~SerialPort()
    {
        free(_data_tmp);
        free(pingpong);
        delete _serial_port;
    }

    void SerialPort::serialPortRead(uint8_t *msg, uint8_t max_len)
    {
        try
        {
            read(*_serial_port, boost::asio::buffer(msg, max_len), _err);
        }
        catch (...)
        {
            LOG(ERROR) << "readData from serial port error! " << _err.message();
        }
    }

    void SerialPort::serialPortWrite(uint8_t *msg, int len)
    {
        try
        {
            write(*_serial_port, buffer(msg, (size_t)len));
        }
        catch (...)
        {
            LOG(ERROR) << "write to serial port error! ";
        }
    }

    void SerialPort::addCRC(SerialPortData *msg)
    {
        if (!msg)
            return;
        msg->crc = getCRC(msg);
    }

    void SerialPort::addCRC(unsigned char *msg)
    {
        if (!msg)
            return;
        msg[_data_len_write - 1] = getCRC(msg);
    }

    uint8_t SerialPort::getCRC(SerialPortData *data)
    {
        auto _data = reinterpret_cast<unsigned char *>(data);
        int dwLength = _data_len - 1;
        unsigned char ucCRC8 = CRC8_INIT;
        unsigned char ucIndex;
        while (dwLength--)
        {
            ucIndex = ucCRC8 ^ (*_data++);
            ucCRC8 = CRC8_TAB[ucIndex];
        }

        return ucCRC8;
    }

    uint8_t SerialPort::getCRC(unsigned char *data)
    {
        auto _data = reinterpret_cast<unsigned char *>(data);
        int dwLength = _data_len_write - 1;
        unsigned char ucCRC8 = CRC8_INIT;
        unsigned char ucIndex;
        while (dwLength--)
        {
            ucIndex = ucCRC8 ^ (*_data++);
            ucCRC8 = CRC8_TAB[ucIndex];
        }

        return ucCRC8;
    }

    bool SerialPort::verifyCRC(SerialPortData *data)
    {
        if (!data)
            return false;
        return data->crc == getCRC(data);
    }

    void SerialPort::readData(SerialPortData *imu_data)
    {
        //    auto data_read = reinterpret_cast<SerialPortData*>(params_p);
        serialPortRead(_data_tmp, _data_len);
        memcpy(pingpong + _data_len, _data_tmp, _data_len);
        for (int start_bit = 0; start_bit < _data_len; start_bit++)
        {
            if (pingpong[start_bit] == '!')
            {
                //            DLOG(INFO) << "receive imu data!";
                memcpy(_data_tmp, pingpong + start_bit, _data_len);
                //@TODO CRC校验
                if (verifyCRC(reinterpret_cast<SerialPortData *>(_data_tmp))) //CRC校验
                {
                    //memcpy(imu_data, _data_tmp, _data_len);
                    imu_data->flag = _data_tmp[1];
                    imu_data->pitch = (_data_tmp[2] << 8) | _data_tmp[3];
                    imu_data->yaw = (((int)_data_tmp[4]) << 24) | (((int)_data_tmp[5]) << 16) | (((int)_data_tmp[6]) << 8) |
                                    (_data_tmp[7]);
                    imu_data->MCU_Time = (((int)_data_tmp[8]) << 24) | (((int)_data_tmp[9]) << 16) | (((int)_data_tmp[10]) << 8) | (_data_tmp[11]); //ms
#ifdef USE_SHOOT_SPEED
                    imu_data->shoot_speed = ((int)_data_tmp[12] << 8) + (int)_data_tmp[13];
#endif
                    //时间戳初始化
                    if (!BeginTime_Flag)
                    {
                        MCU_BeginTime = imu_data->MCU_Time;
                        BeginTime_Flag = true;
                        time_start = std::chrono::steady_clock::now();
                        TX2_BeginTime = time_start;
                    }
                    break;
                }
            }
        }
        memcpy(pingpong, pingpong + _data_len, _data_len);
    }

    void SerialPort::writeData(SerialPortWriteData *_data_write)
    {
        msg[0] = '!';
        unsigned char *tmp = (unsigned char *)(&_data_write->shootStatus);
        msg[1] = tmp[0];
        //        std::cout<<"shoot:"<<(int)tmp[0]<<std::endl;
        tmp = (unsigned char *)(&_data_write->pitch);
        // LOG(INFO) << "send pitch:" << (int)_data_write->pitch;
        msg[2] = tmp[1];
        msg[3] = tmp[0];

        tmp = (unsigned char *)(&_data_write->yaw);
        // LOG(INFO) << "send yaw:" << (int)_data_write->yaw;
        msg[4] = tmp[3];
        msg[5] = tmp[2];
        msg[6] = tmp[1];
        msg[7] = tmp[0];
#ifndef KALMANDEBUG
        addCRC(msg);
#endif
        serialPortWrite(msg, _data_len_write);
    }

    void SerialPort::startSerialPortRead(Params_ToSerialPort &params_to_serial_port)
    {
        // params in
        thread_params.__this = (void *)(this);
        // 共享指针
        thread_params.serial_data_queue = params_to_serial_port.serial_data_queue;

        int nRet = true;
        nRet = pthread_create(&threadID, nullptr, read_data, static_cast<void *>(&thread_params));

        LOG_IF(ERROR, nRet == -1) << "error in creating serial port read thread!";
    }
}
