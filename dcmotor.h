#ifndef DCMOTOR_H
#define DCMOTOR_H
#include <QSerialPort>          // for inheritance of QSerialport
#include <QDataStream>
#include <sstream>              // for std::stringstream
#include <iostream>             // for std::cout
#include <QString>
#include <QtConcurrent/QtConcurrent>
#include <QDebug>
#include <queue>
enum command{
    OK = 0,
    NP = 1,
    POS = 2
};
class dcmotor : public  QSerialPort
{
public:
    dcmotor();
    void Open(const QString &comport, const int baudrate);
    void test();
    void SetHome();
    void stop();
    void D(int rpm);
    void R(int rpm);
    void SetMaxVelocity(double v_rpm);
    void RotateRelativeDistancce(int value);


private:
    std::queue<command> command_queue;
};

#endif // DCMOTOR_H
