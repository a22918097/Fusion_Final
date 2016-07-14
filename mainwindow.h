#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// multi-thread processing
#define multi_thread

#include <QMainWindow>
#include <QApplication>
#include <QFileDialog>
#include <QString>
#include <QMessageBox>
#include <QTime>
#include <QLabel>
#include <QTableView>
#include <QKeyEvent>
#include <QKeySequence>
#include <QtConcurrent/QtConcurrent>
#include <QDir>
extern QDir project_path;

// recording
#include "recording/recording.h"
extern recording re;

#include <iostream>
#include <opencv2/opencv.hpp>

// Laser range finder controller
#include "lrf_controller.h"

// Stereo vision
#include "stereo_vision.h"
#include "stereomatchparamform.h"

// camera calibration
#include "calibrationform.h"

// Radar ESR controller
#include "radarcontroller.h"

// Sensor Information
#include "sensorinfo.h"

static bool fg_running;

namespace Ui {
class MainWindow;
}


namespace INPUT_TYPE {
enum {
    DEVICE,
    RECORDING
};
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    QLabel *label_file_loaded;

    void reportError(QString part, QString level, QString content);

    void report(QString);

    bool cwdIsProjectFolder();               // find the project folder for loading files

    // Read params ============
    bool fg_param_loaded;               // check wether the paramerters are loaded

    // file in & out
    stereo_vision::camParam* fin_cam_param;
    stereo_vision::matchParamSGBM* fin_SGBM;
    stereo_vision::matchParamBM* fin_BM;
    int fin_lrf_port, fin_lrf_baud_rate, fin_lrf_scale, fin_lrf_res;

    void paramRead();                   // read params from basic_param.yml

    void paramWrite();

    void paramUpdate();

    void readFromTxt(QString file_name, cv::Mat *output);   // read image data from text
    // ======================== End

    // Radar ESR ==============
    bool fg_retrieving;

    // tableView used
    QStandardItemModel* model_radar;

    bool radarDataIn();

    void radarDisplayTopViewBG();
    // ======================== End

    // Stereo vision ==========
    bool fg_capturing;

    bool svWarning();

    bool svDataIn();

    void camOpen();

    void svDisplayTopViewBG();
    // ======================== End

    // Stereo vision param ====
    stereoMatchParamForm *form_smp;

    bool fg_form_smp_alloc;

    void retrieveMatchParam();
    // ======================== End

    // Laser range finder =====
    bool fg_acquiring;

    bool fg_buffering;

    bool fg_lrf_record;
    bool fg_lrf_record_quit;

    FILE* fp1;

    void lrfClearData();

    bool lrfReadData();

    cv::Mat display_lrf;

    std::vector<double> lrf_temp;
    std::vector<std::vector<double> > display_lrf_3D;
    // ======================== End

    // SensorInfo =============
    SensorInfo* si;

    void initialFusedTopView();

    void updateFusedTopView();

    void dataFused();
    // ======================== End

    // Thread control =========
//    bool fg_running;
    QFuture<int> f_sv;
    QFuture<bool> f_lrf;
    QFuture<void> f_lrf_buf;
    QFuture<int> f_radar;
    QFuture<void> f_fused;
    int f_sv_status;
    bool f_lrf_status = false;
    int f_radar_status = false;
//    QFutureWatcher<void> fw_sv;
//    QFutureWatcher<void> fw_lrf;
//    QFutureWatcher<void> fw_lrf_buf;

    void exec();
    void threadCheck();
    void threadProcessing();
    // ======================== End

    // Camera calibration =====
    calibrationForm *form_calib;
    bool fg_form_calib_alloc;
    // ======================== End

    // Mouse control ==========
    QString mouse_info;

    int dis_xx, dis_yy;
    float dis_disp;
    cv::Point3i dis_pos3D;
    // ======================== End

    // Author =================
    QDialog *devForm;
    QLabel *icon;
    QLabel *text;

    bool fg_author;

    void releaseAuthor();
    // ======================== End

    // Recording ==============
    void inputType(int type);

    void loadData(int record_type);

    int sv_frame_count;
    // Recording ============== End

signals:
    // Camera calibration =====
    void sendBasicInfo(int focal_length, double base_line);
    void sendImage(cv::Mat *img);
    void sendImages(cv::Mat *img_L, cv::Mat *img_R);
    // ======================== End

    // Stereo vision param ====
    // ======================== End

private slots:
    void on_pushButton_start_all_clicked();

    void on_pushButton_stop_all_clicked();

    void on_actionShortcut_triggered();

    void on_actionAuthor_triggered();

    // Mouse control ==========
    void mouseXY(int x, int y);

    void wheelEvent(QWheelEvent *ev);

    void keyPressEvent(QKeyEvent *ev);
    // ======================== End

    // Laser range finder =====
    void on_pushButton_lrf_open_clicked();

    void lrfDisplay(double *lrf_data, cv::Mat *display_lrf);

    void on_pushButton_lrf_request_clicked();

    void on_pushButton_lrf_retrieve_clicked();

    void on_pushButton_lrf_stop_clicked();

    void on_pushButton_lrf_request_ONCE_clicked();

    void on_pushButton_lrf_record_data_clicked();

    void on_pushButton_lrf_record_stop_clicked();

    void on_pushButton_lrf_read_range_clicked();
    // ======================== End

    // Stereo vision ==========
    void on_pushButton_cam_open_clicked();

    void on_pushButton_cam_stop_clicked();

    void on_pushButton_cam_step_clicked();

    void on_pushButton_cam_capture_clicked();

    void on_checkBox_do_calibration_clicked(bool checked);

    void on_checkBox_do_depth_clicked(bool checked);

    void svDisplay(cv::Mat *img_L, cv::Mat *img_R, cv::Mat *disp, cv::Mat *disp_pseudo, cv::Mat *topview, cv::Mat *img_detected, int detected_obj, int current_frame_count);
    // ======================== End

    // Stereo vision param ====
    void closeFormSmp(void);

    void on_radioButton_BM_clicked();

    void on_radioButton_SGBM_clicked();

    void on_lineEdit_sv_focal_length_returnPressed();

    void on_checkBox_sv_topview_clicked(bool checked);

    void on_spinBox_topview_r_valueChanged(int arg1);

    void on_spinBox_topview_c_valueChanged(int arg1);

    void on_pushButton_stereo_match_param_clicked();

    void on_comboBox_camera_focal_length_currentIndexChanged(int index);

    void on_lineEdit_base_line_returnPressed();

    void on_checkBox_pseudo_color_clicked(bool checked);

    void on_checkBox_topview_plot_points_clicked(bool checked);

    void on_checkBox_sv_reproject_clicked(bool checked);

    void on_pushButton_sv_record_data_clicked();

    void on_pushButton_sv_read_images_clicked();

    void on_pushButton_sv_read_disp_clicked();
    // ======================== End

    // Camera calibration =====
    void on_pushButton_camera_calibration_clicked();

    void closeEvent(QCloseEvent *);

    void requestImage(char CCD);

    void closeFormCalib(void);
    // ======================== End

    // Radar ESR ==============
    void radarDisplay(int detected_obj, cv::Mat *img, cv::Mat *topview);

    void on_pushButton_radar_write_clicked();

    void on_pushButton_radar_bus_on_clicked();

    void on_pushButton_radar_bus_off_clicked();

    void on_pushButton_radar_open_clicked();

    void on_checkBox_radar_topview_clicked(bool checked);

    void on_spinBox_radar_topview_r_valueChanged(int arg1);

    void on_spinBox_radar_topview_c_valueChanged(int arg1);
    // ======================== End

    // SensorInfo =============
    void fusedDisplay(cv::Mat *fused_topview, cv::Mat *img_detected_display);
    // SensorInfo ============= End

    // 2014 BIOME =============
    void on_horizontalSlider_2_sliderReleased();

    void on_horizontalSlider_sliderReleased();

    void on_pushButton_lrf_read_range_2_clicked();

    void on_pushButton_lrf_request_2_clicked();

    void on_pushButton_lrf_retrieve_2_clicked();

    void on_pushButton_lrf_stop_2_clicked();
    // 2014 BIOME ============= End

    // Recording ==============
    void on_pushButton_sv_record_clicked();
    void on_pushButton_sv_load_data_clicked();
    void videoIsEnd();
    void dataIsEnd();
    void on_pushButton_radar_record_clicked();
    void on_pushButton_lrf_record_clicked();
    void on_pushButton_all_record_clicked();
    void on_pushButton_radar_load_data_clicked();
    void on_pushButton_lrf_load_data_clicked();
    void on_pushButton_all_load_data_clicked();
    void on_radioButton_input_device_clicked();
    void on_radioButton_input_recording_clicked();
    // Recording ============== End

    // GUI ====================
    void guiDisplay(int type, bool fg_on);
    // GUI ==================== End

    void on_lineEdit_sv_rig_height_returnPressed();
    void on_radioButton_vehicle_cart_clicked();
    void on_radioButton_vehicle_car_clicked();
    void on_checkBox_sv_ground_filter_clicked(bool checked);
    void on_pushButton_radar_step_clicked();
    void on_checkBox_fusion_sv_clicked(bool checked);
    void on_checkBox_fusion_radar_clicked(bool checked);
    void on_spinBox_lrf_scale_valueChanged(int arg1);
    void on_checkBox_ca_clicked(bool checked);
    void on_checkBox_ot_clicked(bool checked);
    void on_checkBox_ca_astar_clicked(bool checked);
    void on_horizontalSlider_ana_range_filter_min_valueChanged(int value);
    void on_horizontalSlider_ana_range_filter_max_valueChanged(int value);
    void on_checkBox_sv_matching_clicked(bool checked);
    void on_pushButton_step_all_clicked();
    void on_checkBox_ot_trajectory_clicked(bool checked);
    void on_radioButton_vehicle_tractor_clicked();
    void on_checkBox_avgfilter_clicked(bool checked);
};

// Mouse control ==========
class MouseLabel : public QLabel
{
    Q_OBJECT
public:
    MouseLabel(QWidget* parent = 0);

signals:
    void mXY(int x, int y);

protected:
    virtual void mouseMoveEvent(QMouseEvent *e);
};

#endif // MAINWINDOW_H
