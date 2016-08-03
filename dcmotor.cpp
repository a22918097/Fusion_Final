#include "dcmotor.h"
#include <QDebug>
dcmotor::dcmotor()
{

}

void dcmotor::Open(const QString &comport, const int baudrate)
{

        this->QSerialPort::close();                 // reset the comport
        this->QSerialPort::setPortName(comport);
        switch (baudrate) {
        case 9600:
            this->QSerialPort::setBaudRate(QSerialPort::Baud9600);
            break;
        case 19200:
            this->QSerialPort::setBaudRate(QSerialPort::Baud19200);
            break;
        case 38400:
            this->QSerialPort::setBaudRate(QSerialPort::Baud38400);
            break;
        case 115200:
            this->QSerialPort::setBaudRate(QSerialPort::Baud115200);
            break;
        default:
            this->QSerialPort::setBaudRate(QSerialPort::Baud9600);
        }
        this->QSerialPort::setDataBits(QSerialPort::Data8);
        this->QSerialPort::setParity(QSerialPort::NoParity);
        this->QSerialPort::setStopBits(QSerialPort::OneStop);
        this->QSerialPort::setFlowControl(QSerialPort::NoFlowControl);
        this->QSerialPort::open(QSerialPort::ReadWrite);
}

void dcmotor::SetHome()
{
    std::stringstream cmd;
    cmd << "HO\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
    command_queue.push(command::OK);
    std::cout << "command_queue size = " << command_queue.size() << std::endl;

}

void dcmotor::stop()
{

     std::stringstream cmd;
     cmd << "V0\n";
     this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));

}

void dcmotor::R(int rpm)
{
    std::stringstream cmd;
    cmd << "V" << rpm << "\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
}

void dcmotor::D(int rpm)
{
    std::stringstream cmd;
    cmd << "V" << rpm << "\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
}

void dcmotor::SetMaxVelocity(double v_rpm)
{
    std::stringstream cmd;
    cmd << "SP" << v_rpm << "\n";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
//    command_queue.push(command::OK);
    //    std::cout << "command_queue size = " << command_queue.size() << std::endl;
}

void dcmotor::RotateRelativeDistancce(int value)
{
    std::stringstream cmd;
    cmd << "LR" << value << "\n" << "M\r";
    this->write(cmd.str().c_str(), strlen(cmd.str().c_str()));
//    command_queue.push(command::OK);
//    std::cout << "command_queue size = " << command_queue.size() << std::endl;
//    command_queue.push(command::NP);
//    std::cout << "command_queue size = " << command_queue.size() << std::endl;

}
