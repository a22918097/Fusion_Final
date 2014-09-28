#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Initialization
    lrf = new lrf_controller();
    sv = new stereo_vision();

    // Basic parameters
    // COM port
    for (int i = 1; i <= 20; i++)
        ui->comboBox_lrf_com->addItem("COM" + QString::number(i));
    for (int i = 0; i < 5; i++) {
        ui->comboBox_cam_com_L->addItem(QString::number(i));
        ui->comboBox_cam_com_R->addItem(QString::number(i));
    }

    // Baud rate
    QStringList list_baudrate;
    list_baudrate << "9600" << "19200" << "38400" << "76800" << "115200";
    ui->comboBox_lrf_baudRate->addItems(list_baudrate);

    // default settings
    // LRF
    ui->comboBox_lrf_com->setCurrentText("COM7");
    ui->comboBox_lrf_baudRate->setCurrentText("38400");
    lrf_status = false;
    lrfResetData();
    lrf_timer = new QTimer;
    connect(lrf_timer, SIGNAL(timeout()), this, SLOT(lrfReadData()));

    // SV
    ui->comboBox_cam_com_L->setCurrentText("0");
    ui->comboBox_cam_com_R->setCurrentText("2");

    fps_time = new QTime;
    connect(sv, SIGNAL(run(img_L, img_R, disp)), this, SLOT(displaying()));
}

MainWindow::~MainWindow()
{
    cv::destroyAllWindows();
    lrf->close();
    delete lrf;
    delete fps_time;
    lrf_timer->stop();
    delete lrf_timer;
    delete sv;
    delete ui;
}

void MainWindow::on_pushButton_lrf_open_clicked()
{
    lrf->open(ui->comboBox_lrf_com->currentText(), ui->comboBox_lrf_baudRate->currentText().toInt());
    if (!lrf->isOpen()) {
        ui->system_log->append("Error!  Port can NOT be open or under using.");
        QMessageBox::information(0, "Error!", "Port cannot be open or under using.");
        return;
    }
    else {
        ui->system_log->append("Port's opened.");
        lrf_timer->start(350);
    }
}

void MainWindow::lrfResetData()
{
    // reset data
    for (int i = 0; i < LENGTH_DATA; i++)
        lrf_data[i] = -1.0; //**// lrf range?
}

void MainWindow::lrfReadData()
{
    lrfResetData();

    if (lrf->acquireData(lrf_data) && lrf_status == false) {
        ui->system_log->append("data: acquired");
        lrf_status = true;
    }
    else {
        ui->system_log->append("data: lost");
        lrf_status = false;
    }

    // display
    cv::Mat display_lrf = cv::Mat::zeros(800, 800, CV_8UC3);
    double angle = 0.0;

    for (int i = 0; i < LENGTH_DATA; ++i) {
        double r = lrf_data[i];
        double x = r * cos(angle * CV_PI / 180.0);
        double y = r * sin(angle * CV_PI / 180.0);

        cv::circle(display_lrf, cv::Point(x / ui->spinBox_lrf_scale->value() + 400, y / ui->spinBox_lrf_scale->value() + 100), 1, cv::Scalar(0, 0, 255), -1);
        if (LENGTH_DATA / 2 == i)
            cv::circle(display_lrf, cv::Point(x / ui->spinBox_lrf_scale->value() + 400, y / ui->spinBox_lrf_scale->value() + 100), 5, cv::Scalar(0, 255, 0), -1);

        angle += RESOLUTION;
    }

    cv::imshow("image", display_lrf);
    cv::waitKey(1);

}

void MainWindow::on_pushButton_lrf_display_clicked()
{
    lrfReadData();
}

void MainWindow::camOpen()
{
    int L = ui->comboBox_cam_com_L->currentIndex();
    int R = ui->comboBox_cam_com_R->currentIndex();

    if (!sv->open(L, R)) {
        ui->system_log->append("Error!  Camera can NOT be opened.");
        QMessageBox::information(0, "Error!", "Camera can NOT be opened.");
        sv->close();
        return;
    }
    else {
        ui->system_log->append("Port's opened.");
    }
}

void MainWindow::displaying()
{
    ui->label_cam_img_L->setPixmap(QPixmap::fromImage(QImage::QImage(img_L.data, img_L.cols, img_L.rows, QImage::Format_RGB888)).scaled(IMG_W, IMG_H));
    ui->label_cam_img_R->setPixmap(QPixmap::fromImage(QImage::QImage(img_R.data, img_R.cols, img_R.rows, QImage::Format_RGB888)).scaled(IMG_W, IMG_H));
    cv::imshow("disp", disp);
//    ui->label_disp->setPixmap(QPixmap::fromImage(QImage::QImage(disp.data, disp.cols, disp.rows, QImage::Format_RGB888)).scaled(IMG_W, IMG_H));
}

void MainWindow::on_pushButton_cam_open_clicked()
{
    camOpen();
    stereoVision();
}

void MainWindow::on_pushButton_cam_step_clicked()
{
    stereoVision();
}

void MainWindow::on_pushButton_cam_capture_clicked()
{
    sv->start();
    sv->run(img_L, img_R, disp);
}

void MainWindow::on_pushButton_cam_stop_clicked()
{
    camStop();
}

void MainWindow::stereoVision()
{
    // count fps
    fps_time->restart();

    sv->camCapture(img_L, img_R);

    sv->stereoMatch(img_L, img_R, disp);

    // count fps
    int mSecPerFrame = fps_time->elapsed();
    fps = 1000.0 / (double)(mSecPerFrame);
    ui->statusBar->showMessage(QString::number(fps) + " fps");

    displaying();
}

void MainWindow::on_pushButton_3_clicked()
{

}

void MainWindow::on_pushButton_4_clicked()
{

}
