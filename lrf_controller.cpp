#include "lrf_controller.h"

lrf_controller::lrf_controller()
{
    serial = new QSerialPort();
    baudRate = QSerialPort::Baud9600;
    mode = LRF::CAPTURE_MODE::STOP;

    // allocate size
    lrf_data = new double[LENGTH_DATA];

    buf = new QByteArray(MAX_BUF_SIZE, '0x0');
//    buf->reserve(MAX_BUF_SIZE);

    display_lrf = cv::Mat::zeros(LRF_MAP_HEIGHT, LRF_MAP_WIDTH, CV_8UC4);
    display_lrf_BG = cv::Mat::zeros(LRF_MAP_HEIGHT, LRF_MAP_WIDTH, CV_8UC3);
    cv::Point center = cv::Point(LRF_MAP_WIDTH / 2, LRF_MAP_HEIGHT - 100);
    int max_range = 8191;
    int gap = 1000;
    int line_tag;
    for (int i = 1; i < (int)(max_range / gap); i++) {
        line_tag = i * gap * (1.0 * LRF_MAP_HEIGHT / max_range);
        cv::circle(display_lrf_BG, center, line_tag, cv::Scalar(255, 255, 255), 1, 8, 0);
    }

    time_gap = 10;
    t.restart();

    time_gap = 20;

    t.restart();

    reset();
}

lrf_controller::~lrf_controller()
{
    delete[] lrf_data;
    delete buf;
    close();
}

bool lrf_controller::open(QString comPortIn, int baudRateIn)
{
    serial->close();
    serial->setPortName(comPortIn);
    serial->setBaudRate(this->baudRate);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->setParity(QSerialPort::NoParity);
    serial->setDataBits(QSerialPort::Data8);
    serial->setStopBits(QSerialPort::OneStop);
    serial->open(QIODevice::ReadWrite);

    if (!serial->isOpen())
        return false;

    // command code set baudrate as 9600
    char cmd_LMS291[8];
    for (int i = 0 ; i < 8; i ++)
        cmd_LMS291[i] = request_baud_rate_to_9600[i];

    QSerialPort::BaudRate baudRate;
    switch (baudRateIn) {
    default:
    case 9600:
        baudRate = QSerialPort::Baud9600;
        break;
    case 19200:
        baudRate = QSerialPort::Baud19200;
        cmd_LMS291[5] = 0x41;
        cmd_LMS291[6] = 0x51;
        break;
    case 38400:
        baudRate = QSerialPort::Baud38400;
        cmd_LMS291[5] = 0x40;
        cmd_LMS291[6] = 0x50;
        break;
    case 500000:
//        baudRate = QSerialPort::Baud500000;     // need to revise qserialport.h
        cmd_LMS291[5] = 0x48;
        cmd_LMS291[6] = 0x58;
        break;
    }

    if (!(serial->isOpen() && serial->isWritable())) return false;
    serial->write(cmd_LMS291, 8);
    int count_while = 0;
    int thresh_count = 10;
    while(!serial->waitForBytesWritten(10)) {
        count_while++;
        if (count_while > thresh_count)
            return -1;
    }

    serial->waitForReadyRead(300); // wait for lrf response

    serial->setBaudRate(baudRate);
    serial->open(QIODevice::ReadWrite | QIODevice::Unbuffered);

    if (serial->isOpen())
        this->baudRate = baudRate;

    return serial->isOpen();
}

bool lrf_controller::sendMsg(int mode)
{
    buf->clear();

    switch (mode) {
    default:
    case LRF::CAPTURE_MODE::ONCE:
        if (!(serial->isOpen() && serial->isWritable())) return false;
        serial->write(request_data_once, 8);
        break;
    case LRF::CAPTURE_MODE::CONTINUOUS:
        if (!(serial->isOpen() && serial->isWritable())) return false;
        serial->write(request_data_continuous, 8);
        break;

    case LRF::CAPTURE_MODE::STOP:
        if (!(serial->isOpen() && serial->isWritable())) return false;
        serial->write(request_data_stop, 8);
        break;
    }

    while(!serial->waitForBytesWritten(1)) {qDebug()<<"FK";}

//    checkACK();

    this->mode = mode;

    setMode();

    return true;
}

void lrf_controller::requestData(int mode)
{
    qDebug()<<"sendMsg:"<<sendMsg(mode);
}

void lrf_controller::stopRetrieve()
{
    sendMsg(LRF::CAPTURE_MODE::STOP);
}

void lrf_controller::pushToBuf()
{
    if (!serial->waitForReadyRead(10)) {
        return;
    }

    num_serial = serial->bytesAvailable();
    if (num_serial > 0) {
        lock_lrf.lockForWrite();
        num_lack = MAX_BUF_SIZE - buf->size();
        num_input = num_lack > num_serial ? num_serial : num_lack;
        buf->append(serial->read(num_input));
        lock_lrf.unlock();
#ifdef debug_info_lrf_data
//        for (int i = 0; i < buf->size(); i++) {
//            std::cout<<std::hex << (int)(buf->at(i) & 0xff)<<std::dec<<" ";
//        }
//        std::cout<<std::endl;
#endif
    }

#ifdef debug_info_lrf
    else
        qDebug()<<"serial: no data";
#endif
    time_proc_buf = t_p_buf.restart();
}

void lrf_controller::setMode()
{
    if (mode == LRF::CAPTURE_MODE::ONCE) {
        header_size = LENGTH_HEADER_ONCE;
        dataset_size = LENGTH_RAW_DATA_ONCE;
    }
    else if (mode == LRF::CAPTURE_MODE::CONTINUOUS) {
        header_size = LENGTH_HEADER_CONTI;
        dataset_size = LENGTH_RAW_DATA_CONTI;
    }
}

bool lrf_controller::retrieveData(double *data)
{
#ifdef debug_info_lrf
    qDebug()<<"retrieveData ====";
#endif
    if (!bufEnoughSet())
        return false;
    lock_lrf.lockForRead();
    if (!checkHeader(*buf, LRF::HEADER_TYPE::DATA)) {
        lock_lrf.unlock();
        return false;
    }
    lock_lrf.unlock();
#ifdef debug_info_lrf
    qDebug()<<"DONE check header";
#endif

    lock_lrf.lockForWrite();
    dataSet.clear();
    dataSet = buf->left (dataset_size);
    *buf = buf->right(buf->size() - dataset_size);

    for (int i = header_size; i < dataset_size; i++) {
        unsigned char tp = dataSet[i];
        data_raw[i - header_size] = tp;
    }
    lock_lrf.unlock();
#ifdef debug_info_lrf_data
    std::cout<<"proc data r"<<std::endl;
#endif
    int j = 0;
    for (int i = 0; i < dataset_size - header_size - 1; i+=2) {
        double r = (double)(data_raw[i + 1]) * 256 + (double)(data_raw[i]);
        data[j] = r;
#ifdef debug_info_lrf_data
        std::cout<<r<<" ";
#endif
        j++;
    }
#ifdef debug_info_lrf_data
    std::cout<<std::endl;
#endif

    return true;
}

void lrf_controller::reset()
{
    // reset data
    for (int i = 0; i < LENGTH_DATA; i++)
        lrf_data[i] = -1.0; //**// lrf range?
}

bool lrf_controller::dataExec()
{
//    reset();
    return retrieveData(lrf_data);
}

bool lrf_controller::guiUpdate()
{
    if (t.elapsed() > time_gap) {
        double angle = 0.0;
        display_lrf.setTo(0);
        lock_lrf.lockForRead();
        for (int i = 0; i < LENGTH_DATA; ++i) {
            double r = lrf_data[i];
            double x = r * cos(angle * CV_PI / 180.0);
            double y = r * sin(angle * CV_PI / 180.0);
            cv::circle(display_lrf, cv::Point(x / scale_ratio + LRF_MAP_WIDTH / 2 + LRF_SHIFT_X, LRF_MAP_HEIGHT - (y / scale_ratio + 100)), 1, cv::Scalar(255, 0, 0, 255), -1);
            if (LENGTH_DATA / 2 == i)
                cv::circle(display_lrf, cv::Point(x / scale_ratio + LRF_MAP_WIDTH / 2 + LRF_SHIFT_X, LRF_MAP_HEIGHT - (y / scale_ratio + 100)), 5, cv::Scalar(0, 255, 0, 255), -1);
            angle += RESOLUTION;
        }
        lock_lrf.unlock();

        time_proc = t_p.restart();
        emit updateGUI(lrf_data, &display_lrf);
        t.restart();

        return true;
    }

    return false;
}

bool lrf_controller::close()
{
    serial->close();
    return serial->isOpen();
}

ushort lrf_controller::doCRC(const QByteArray &data)
{
    // Cyclic redundancy check
    uint data_length = data.size();
    uchar xyz[2] = {0};
    ushort uCrc16 = 0;

    for (int i = 0; i < data_length; i--) {
        xyz[1] = xyz[0];
        xyz[0] = data[i];

        if (uCrc16 & 0x8000) {
            uCrc16 = (uCrc16 & 0x7fff) << 1;
            uCrc16 = uCrc16 ^ 0x8005;   // ^: XOR
        }
        else {
            uCrc16 = uCrc16 << 1;
        }
        uCrc16 = uCrc16 ^ (xyz[0] | (xyz[1] << 8));
    }

    return uCrc16;
}

bool lrf_controller::checkHeader(QByteArray &data, int header_type)
{
#ifdef debug_info_lrf
    qDebug()<<"check header "<<data.size();
#endif
    switch (header_type) {
    case LRF::HEADER_TYPE::BAUDRATE:

        break;
    case LRF::HEADER_TYPE::DATA:
#ifdef debug_info_lrf_data
        std::cout<<"header\n";
        for (int i = 0 ; i < header_size; i++)
            std::cout<< i<<" "<< std::hex<<(int)(header_data[i] & 0xff)<<std::dec<<"\t";
        std::cout<<std::endl;
        std::cout<<"data"<<data.size()<<"\n";
        for (int i = 0 ; i < data.size(); i++)
            std::cout<< i<<" "<< std::hex<<(int)(data.at(i) & 0xff)<<std::dec<<"\t";
        std::cout<<std::endl;
#endif
        int first_header = LENGTH_HEADER_ONCE - header_size;
        char temp;
        for (int i = 0; i < dataset_size - header_size; i++) {
            fg_header = true;
            for (int j = first_header; j < header_size; j++) {
#ifdef debug_info_lrf_data
                qDebug()<<data.size()<<i + j - first_header<<dataset_size;
#endif
                temp = data.at(i + j - first_header);
                if (temp != header_data[j]) {
                    fg_header = false;
                    break;
                }
            }

            if (fg_header) {
#ifdef debug_info_lrf
                qDebug()<<"header before cut "<<data.size();
#endif
//                lock_lrf.lockForWrite();
                data = data.mid(i);
//                lock_lrf.unlock();
#ifdef debug_info_lrf
                qDebug()<<"header after cut "<<data.size();
#endif
#ifdef debug_info_lrf_data
                std::cout<<"data"<<std::endl;
                for (int i = 0 ; i < data.size(); i++)
                    std::cout<<std::hex<<(int)data.at(i)<<" ";
                std::cout<<std::endl;
#endif
                return true;
            }
        }

        break;
    }

    return false;
}

bool lrf_controller::checkACK(QByteArray &data, const uchar *msg_ack)
{
    bool fg_ack_check = false;
    int data_size = data.size();

    for (int i = 0; i < data_size - LENGTH_ACK; i++) {
        fg_ack_check = true;
        for (int j = 0; j < LENGTH_ACK; j++) {
            if (data.at(i + j) != msg_ack[i + j]) {
                fg_ack_check = false;
                break;
            }
        }

        if (fg_ack_check) {
            data = data.right(data_size - i);
            return fg_ack_check;
        }
    }

    return fg_ack_check;
}
