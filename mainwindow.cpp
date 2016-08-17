#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    // default setting =========================
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<QVector<int>>("QVector<int>");

    // THREAD RUN
    fg_running = false;

    fg_param_loaded = false;

    // LRF
    fg_lrf_record = false;
    fg_lrf_record_quit = false;

    fg_acquiring = false;
    fg_buffering = false;

    // SV
    fg_capturing = false;

    // RADAR
    fg_retrieving = false;

    // camera calibration
    fg_form_calib_alloc = false;

    // Stereo vision param
    fg_form_smp_alloc = false;

    // Author
    fg_author = false;

    ui->setupUi(this);

    // Recording
    label_file_loaded = new QLabel();
    ui->statusBar->addWidget(label_file_loaded);

    // LRF
    // COM port
    for (int i = 1; i <= 20; i++)
        ui->comboBox_lrf_com->addItem("COM" + QString::number(i));
    // Baud rate
    QStringList list_baudrate;
    list_baudrate << "9600" << "19200" << "38400" << "500000";
    ui->comboBox_lrf_baudRate->addItems(list_baudrate);

    // SV
    // COM port
    for (int i = 0; i < 6; i++) {
        ui->comboBox_cam_device_index_L->addItem(QString::number(i));
        ui->comboBox_cam_device_index_R->addItem(QString::number(i));
    }

    // focal length
    ui->comboBox_camera_focal_length->addItem("16");
    ui->comboBox_camera_focal_length->addItem("12");
    ui->comboBox_camera_focal_length->addItem("4");

    fin_cam_param = new stereo_vision::camParam;
    fin_SGBM = new stereo_vision::matchParamSGBM;
    fin_BM = new stereo_vision::matchParamBM;

    if (!cwdIsProjectFolder())
        reportError("path", "Warning.", "Current folder is NOT \"Fusion\".");

    paramRead();
    // default setting ========================= End

    si = new SensorInfo();

    // GUI
    QImage gui_BG, gui_car, gui_left_oncoming, gui_right_oncoming, gui_approaching, gui_sign;
    gui_BG.load(":/icon/gui_BG_road.png");
    gui_car.load(":/icon/car1.png");
    gui_left_oncoming.load(":/icon/arrow_left_oncoming.png");
    gui_right_oncoming.load(":/icon/arrow_right_oncoming.png");
    gui_approaching.load(":/icon/arrow_approaching.png");
    gui_sign.load(":/icon/warning.png");

    ui->label_gui_BG->setPixmap(QPixmap::fromImage(gui_BG));
    ui->label_gui_vehicle->setPixmap(QPixmap::fromImage(gui_car));
    ui->label_gui_approaching->setPixmap(QPixmap::fromImage(gui_approaching));
    ui->label_gui_left_oncoming->setPixmap(QPixmap::fromImage(gui_left_oncoming));
    ui->label_gui_right_oncoming->setPixmap(QPixmap::fromImage(gui_right_oncoming));
    ui->label_gui_sign->setPixmap(QPixmap::fromImage(gui_sign));
    ui->label_gui_approaching->setVisible(false);
    ui->label_gui_left_oncoming->setVisible(false);
    ui->label_gui_right_oncoming->setVisible(false);
    ui->label_gui_sign->setVisible(false);

    QObject::connect(si, SIGNAL(guiDisplay(int, bool)), this, SLOT(guiDisplay(int, bool)));

    // Laser range finder ======================
    QObject::connect(si->lrf, SIGNAL(updateGUI(double *, cv::Mat *)), this, SLOT(lrfDisplay(double *, cv::Mat *)));
    
    // display
    ui->label_lrf_data_BG->setPixmap(QPixmap::fromImage(QImage::QImage(si->lrf->display_lrf_BG.data, si->lrf->display_lrf_BG.cols, si->lrf->display_lrf_BG.rows, si->lrf->display_lrf_BG.step, QImage::Format_RGB888)).scaled(LRF_MAP_WIDTH, LRF_MAP_HEIGHT));

    // 3D display
    display_lrf_3D.clear();
    lrf_temp.clear();

    // Laser range finder ====================== End

    // Stereo vision ===========================
#ifdef opencv_cuda
    ui->radioButton_SGBM->setEnabled(false);
    ui->radioButton_BM->setChecked(true);
    si->sv->match_mode = SV::STEREO_MATCH::BM;
#else
    ui->radioButton_BM->setEnabled(true);
    ui->radioButton_BM->setChecked(true);
    si->sv->match_mode = SV::STEREO_MATCH::BM;
    //    ui->radioButton_SGBM->setEnabled(true);
    //    ui->radioButton_SGBM->setChecked(true);
    //    match_mode = SV::STEREO_MATCH::SGBM;
#endif

    // Initialization
    retrieveMatchParam();

    ui->label_cam_img_L->setStyleSheet("background-color:silver");
    ui->label_cam_img_R->setStyleSheet("background-color:silver");
    ui->label_disp->setStyleSheet("background-color:silver");
    ui->label_sv_detected->setStyleSheet("background-color:silver");
    ui->label_sv_frame_count->setVisible(false);
    ui->label_sv_frame_count_2->setVisible(false);

    QObject::connect(si->sv, SIGNAL(updateGUI(cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, int, int)), this, SLOT(svDisplay(cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, int, int)));
    QObject::connect(si->sv, SIGNAL(videoEnd(void)), this, SLOT(videoIsEnd(void)));

    on_checkBox_pseudo_color_clicked(ui->checkBox_pseudo_color->isChecked());
    on_checkBox_sv_topview_clicked(ui->checkBox_sv_topview->isChecked());
    on_checkBox_sv_reproject_clicked(ui->checkBox_sv_reproject->isChecked());
    on_checkBox_topview_plot_points_clicked(ui->checkBox_topview_plot_points->isChecked());
    on_checkBox_do_calibration_clicked(ui->checkBox_do_calibration->isChecked());
    on_checkBox_do_depth_clicked(ui->checkBox_do_depth->isChecked());
    on_checkBox_avgfilter_clicked(ui->checkBox_avgfilter->isChecked());
    // top view
    svDisplayTopViewBG();
    // Stereo vision =========================== End

    // Radar ESR ===============================
    radarDisplayTopViewBG();

    model_radar = new QStandardItemModel(si->rc->objSize(), 1, this);
    model_radar->setHorizontalHeaderItem(0, new QStandardItem(QString("Range")));
    ui->tableView_radar->setModel(model_radar);
    QStringList labels;
    for (int i = 0 ; i < si->rc->objSize(); i++) {
        model_radar->setItem(i, &si->rc->item[i]);
        labels<<QString::number(i);
    }
    model_radar->setVerticalHeaderLabels(labels);

    on_checkBox_radar_topview_clicked(ui->checkBox_radar_topview->isChecked());

    QObject::connect(si->rc, SIGNAL(updateGUI(int, cv::Mat *, cv::Mat *)), this, SLOT(radarDisplay(int, cv::Mat *, cv::Mat *)));
    QObject::connect(si->rc, SIGNAL(dataEnd(void)), this, SLOT(dataIsEnd(void)));
    // Radar ESR =============================== End

    // Fusion ==================================
    initialFusedTopView();

    QObject::connect(si, SIGNAL(updateGUI(cv::Mat *, cv::Mat *)), this, SLOT(fusedDisplay(cv::Mat*,cv::Mat*)));
    // Fusion ================================== En

    // Pseudo color table ======================
    ui->label_color_table->setScaledContents(true) ;
    ui->label_color_table->setPixmap(QPixmap::fromImage(*si->sv->color_table));
    // ========================================= End

    // mouse control ===========================
    QObject::connect(ui->label_cam_img_L, SIGNAL(mXY(int, int)), this, SLOT(mouseXY(int, int)));
    QObject::connect(ui->label_disp, SIGNAL(mXY(int, int)), this, SLOT(mouseXY(int, int)));
    QObject::connect(ui->label_sv_detected, SIGNAL(mXY(int, int)), this, SLOT(mouseXY(int, int)));
    // ========================================= End

    // rig selection (before initialFusedTopView())
    if (ui->radioButton_vehicle_cart->isChecked())
        on_radioButton_vehicle_cart_clicked();
    else if (ui->radioButton_vehicle_car->isChecked())
        on_radioButton_vehicle_car_clicked();
    else if (ui->radioButton_vehicle_tractor->isChecked())
        on_radioButton_vehicle_tractor_clicked();
    else if (ui->radioButton_vehicle_robot->isChecked())
        on_radioButton_vehicle_robot_clicked();

    re.setParentFolder("data");
    ui->label_radar_data->setVisible(0);
    ui->label_radar_data_BG->setVisible(0);
    //DC Motor
    right_dcmotor = new dcmotor();
    left_dcmotor = new dcmotor();
    ui->pushButton_stop->setStyleSheet("color: rgb(255, 0, 0);");

}

MainWindow::~MainWindow()
{
#ifdef debug_info_main
    qDebug()<<"destructor";
#endif

    // If close mainwindow without clicking stop button since the camera has been opened.
    fg_running = false;
    paramUpdate();
    delete fin_cam_param;
    delete fin_SGBM;
    delete fin_BM;

    cv::destroyAllWindows();

    delete si;
    delete label_file_loaded;
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *)
{
    // If close mainwindow without clicking stop button since the camera has been opened.
    fg_running = false;
    if (fg_form_calib_alloc)
        if (form_calib->isVisible())
            form_calib->close();
    if (fg_form_smp_alloc)
        if (form_smp->isVisible())
            form_smp->close();

    releaseAuthor();
}

void MainWindow::keyPressEvent(QKeyEvent *ev)
{
    //    std::cout<<ev->key()<<std::endl;
    int index = ui->tabWidget_display->currentIndex();
    int index_1 = ui->tabWidget->currentIndex();

    // combination clicks
    int key_in = ev->key();
    if (ev->modifiers() & Qt::ControlModifier) {
        ui->centralWidget->setFocus();
        key_in += Qt::CTRL;
        //        std::cout<<key_in<<", "<<QKeySequence(key_in).toString(QKeySequence::NativeText).toStdString()<<std::endl;
        if (QKeySequence(key_in).matches(QKeySequence("Ctrl+Up")) == QKeySequence::ExactMatch)
            ui->tabWidget_display->setCurrentIndex(index - 1);
        else if (QKeySequence(key_in).matches(QKeySequence("Ctrl+Down")) == QKeySequence::ExactMatch)
            ui->tabWidget_display->setCurrentIndex(index + 1);
        else if (QKeySequence(key_in).matches(QKeySequence("Ctrl+Left")) == QKeySequence::ExactMatch)
            ui->tabWidget->setCurrentIndex(index_1 - 1);
        else if (QKeySequence(key_in).matches(QKeySequence("Ctrl+Right")) == QKeySequence::ExactMatch)
            ui->tabWidget->setCurrentIndex(index_1 + 1);
        else if (QKeySequence(key_in).matches(QKeySequence("Ctrl++")) == QKeySequence::ExactMatch) {
            si->zoomInFusedTopView();
            f_fused.waitForFinished();
            updateFusedTopView();
        }
        else if (QKeySequence(key_in).matches(QKeySequence("Ctrl+-")) == QKeySequence::ExactMatch) {
            si->zoomOutFusedTopView();
            f_fused.waitForFinished();
            updateFusedTopView();
        }
    }
}

MouseLabel::MouseLabel(QWidget * parent): QLabel(parent)
{
    this->setMouseTracking(true);
}

void MouseLabel::mouseMoveEvent(QMouseEvent *e)
{
    int x = e->x();
    int y = e->y();
    emit mXY(x, y);
}

void MainWindow::mouseXY(int x, int y)
{
    dis_xx = 2 * x;
    dis_yy = 2 * y;
    dis_disp = -1;
    dis_pos3D = cv::Point3i(-1, -1, -1);
    si->sv->getMouseCursorInfo(dis_yy, dis_xx, dis_disp, dis_pos3D);
    lock_sv_mouse.lockForRead();
    mouse_info.sprintf("(x,y) = (%d,%d), Disp. = %.0f, (X,Y,Z) = (%d,%d,%d)",
                       dis_xx, dis_yy, dis_disp, dis_pos3D.x, dis_pos3D.y, dis_pos3D.z); //**// real X, Y, Z
    lock_sv_mouse.unlock();
    ui->label_depth_info->setText(mouse_info);
}

void MainWindow::reportError(QString part, QString level, QString content)
{
    ui->system_log->append("<FONT COLOR = red>[" + part + "] " + level + "  " + content + "</FONT>");
    QMessageBox::information(0, level, content);
}

void MainWindow::report(QString content)
{
    ui->system_log->append(content);
}

bool MainWindow::cwdIsProjectFolder()
{
    project_path = QDir::current();
    QString current_folder = project_path.currentPath().section("/", -1, -1);
    if (current_folder == "release" || current_folder == "debug") {
        project_path.cdUp();
    }
    if (project_path.path().section("/", -1, -1) == "Fusion") {
        return true;
    }

    return false;
}

void MainWindow::paramRead()
{
    QFile fout(project_path.path() + "/basic_param.yml");
    if (!fout.exists()) {
        reportError("path", "Error!", "basic_param.yml is NOT existed.");
        return;
    }

    cv::FileStorage fs(project_path.path().toStdString() + "/basic_param.yml", cv::FileStorage::READ);

    cv::FileNode n;
    n = fs["stereoVision"];
    fin_cam_param->port_L = (int) n["port_L"];
    fin_cam_param->port_R = (int) n["port_R"];
    fin_cam_param->cam_focal_length = (int) n["cam_focal_length"];
    fin_cam_param->base_line = (double) n["base_line"];
    fin_cam_param->rig_height = (double) n["rig_height"];
    fin_cam_param->focal_length = (double) n["focal_length"];
    ui->comboBox_cam_device_index_L->setCurrentIndex(fin_cam_param->port_L);
    ui->comboBox_cam_device_index_R->setCurrentIndex(fin_cam_param->port_R);
    ui->comboBox_camera_focal_length->setCurrentIndex(fin_cam_param->cam_focal_length);
    ui->label_sv_rig_height->setText(QString::number(fin_cam_param->rig_height));
    ui->lineEdit_base_line->setText(QString::number(fin_cam_param->base_line));
    ui->label_sv_focal_length->setText(QString::number(fin_cam_param->focal_length));

    n = fs["topView"];
    ui->spinBox_topview_r->setValue((int) n["row_interval"]);
    ui->spinBox_topview_c->setValue((int) n["col_interval"]);
    ui->spinBox_radar_topview_r->setValue((int) n["row_interval_radar"]);
    ui->spinBox_radar_topview_c->setValue((int) n["col_interval_radar"]);

    n = fs["laserRangeFinder"];
    fin_lrf_port = (int) n["port"];
    fin_lrf_baud_rate = (int) n["baudRate"];
    fin_lrf_scale = (double) n["mapScale"];
    ui->comboBox_lrf_com->setCurrentIndex(fin_lrf_port);
    ui->comboBox_lrf_baudRate->setCurrentIndex(fin_lrf_baud_rate);
    ui->spinBox_lrf_scale->setValue(fin_lrf_scale);
    std::string res = (std::string) n["resolution"];
    if (res == "cm") {
        fin_lrf_res = 0;
        ui->radioButton_lrf_res_cm->setChecked(true);
    }
    else if (res == "mm") {
        fin_lrf_res = 1;
        ui->radioButton_lrf_res_mm->setChecked(true);
    }

    n = fs["stereoParamSGBM"];
    fin_SGBM->pre_filter_cap = (int)(n["pre_filter_cap"]);
    fin_SGBM->SAD_window_size = (int)(n["SAD_window_size"]);
    fin_SGBM->min_disp = (int)(n["min_disp"]);
    fin_SGBM->num_of_disp = (int)(n["num_of_disp"]);
    fin_SGBM->uniquenese_ratio = (int)(n["uniquenese_ratio"]);
    fin_SGBM->speckle_window_size = (int)(n["speckle_window_size"]);
    fin_SGBM->speckle_range = (int)(n["speckle_range"]);

    n = fs["stereoParamBM"];
    fin_BM->pre_filter_size = (int)(n["pre_filter_size"]);
    fin_BM->pre_filter_cap = (int)(n["pre_filter_cap"]);
    fin_BM->SAD_window_size = (int)(n["SAD_window_size"]);
    fin_BM->min_disp = (int)(n["min_disp"]);
    fin_BM->num_of_disp = (int)(n["num_of_disp"]);
    fin_BM->texture_thresh = (int)(n["texture_thresh"]);
    fin_BM->uniquenese_ratio = (int)(n["uniquenese_ratio"]);
    fin_BM->speckle_window_size = (int)(n["speckle_window_size"]);
    fin_BM->speckle_range = (int)(n["speckle_range"]);

    fs.release();
    fg_param_loaded = true;
}

void MainWindow::paramWrite()
{
    if (!fg_param_loaded)
        return;

    cv::FileStorage fs(project_path.path().toStdString() + "/basic_param.yml", cv::FileStorage::WRITE);

    fs << "stereoVision" << "{";
    fs << "port_L" << ui->comboBox_cam_device_index_L->currentIndex();
    fs << "port_R" << ui->comboBox_cam_device_index_R->currentIndex();
    fs << "cam_focal_length" << ui->comboBox_camera_focal_length->currentIndex();
    fs << "focal_length" << si->sv->cam_param->focal_length;
    fs << "rig_height" << si->sv->cam_param->rig_height;
    fs << "base_line" << ui->lineEdit_base_line->text().toDouble();
    fs << "}";

    fs << "topView" << "{";
    fs << "row_interval" << ui->spinBox_topview_r->value();
    fs << "col_interval" << ui->spinBox_topview_c->value();
    fs << "row_interval_radar" << ui->spinBox_radar_topview_r->value();
    fs << "col_interval_radar" << ui->spinBox_radar_topview_c->value();
    fs << "}";

    fs << "laserRangeFinder" << "{";
    fs << "port" << ui->comboBox_lrf_com->currentIndex();
    fs << "baudRate" << ui->comboBox_lrf_baudRate->currentIndex();
    fs << "mapScale" << ui->spinBox_lrf_scale->value();
    if (ui->radioButton_lrf_res_cm->isChecked())
        fs << "resolution" << "cm";
    else if (ui->radioButton_lrf_res_mm->isChecked())
        fs << "resolution" << "mm";
    fs << "}";

    fs << "stereoParamSGBM" << "{";
    fs << "pre_filter_cap" << si->sv->param_sgbm->pre_filter_cap;
    fs << "SAD_window_size" << si->sv->param_sgbm->SAD_window_size;
    fs << "min_disp" << si->sv->param_sgbm->min_disp;
    fs << "num_of_disp" << si->sv->param_sgbm->num_of_disp;
    fs << "uniquenese_ratio" << si->sv->param_sgbm->uniquenese_ratio;
    fs << "speckle_window_size" << si->sv->param_sgbm->speckle_window_size;
    fs << "speckle_range" << si->sv->param_sgbm->speckle_range;
    fs << "}";

    fs << "stereoParamBM" << "{";
    fs << "pre_filter_size" << si->sv->param_bm->pre_filter_size;
    fs << "pre_filter_cap" << si->sv->param_bm->pre_filter_cap;
    fs << "SAD_window_size" << si->sv->param_bm->SAD_window_size;
    fs << "min_disp" << si->sv->param_bm->min_disp;
    fs << "num_of_disp" << si->sv->param_bm->num_of_disp;
    fs << "texture_thresh" << si->sv->param_bm->texture_thresh;
    fs << "uniquenese_ratio" << si->sv->param_bm->uniquenese_ratio;
    fs << "speckle_window_size" << si->sv->param_bm->speckle_window_size;
    fs << "speckle_range" << si->sv->param_bm->speckle_range;
    fs << "}";

    fs.release();
}

void MainWindow::paramUpdate()
{
    // Unconsidered: topView, laserRangeFinder
    int cur_lrf_res;
    if (ui->radioButton_lrf_res_cm->isChecked())
        cur_lrf_res = 0;
    else if (ui->radioButton_lrf_res_mm->isChecked())
        cur_lrf_res = 1;

    if (fin_cam_param->port_L           != ui->comboBox_cam_device_index_L->currentIndex() ||
            fin_cam_param->port_R           != ui->comboBox_cam_device_index_R->currentIndex() ||
            fin_cam_param->cam_focal_length != ui->comboBox_camera_focal_length->currentIndex() ||
            fin_cam_param->focal_length     != ui->label_sv_focal_length->text().toDouble() ||
            fin_cam_param->rig_height       != ui->label_sv_rig_height->text().toDouble() ||
            fin_cam_param->base_line        != ui->lineEdit_base_line->text().toDouble() ||
            fin_SGBM->pre_filter_cap        != si->sv->param_sgbm->pre_filter_cap ||
            fin_SGBM->SAD_window_size       != si->sv->param_sgbm->SAD_window_size ||
            fin_SGBM->min_disp              != si->sv->param_sgbm->min_disp ||
            fin_SGBM->num_of_disp           != si->sv->param_sgbm->num_of_disp ||
            fin_SGBM->uniquenese_ratio      != si->sv->param_sgbm->uniquenese_ratio ||
            fin_SGBM->speckle_window_size   != si->sv->param_sgbm->speckle_window_size ||
            fin_SGBM->speckle_range         != si->sv->param_sgbm->speckle_range ||
            fin_BM->pre_filter_size         != si->sv->param_bm->pre_filter_size ||
            fin_BM->pre_filter_cap          != si->sv->param_bm->pre_filter_cap ||
            fin_BM->SAD_window_size         != si->sv->param_bm->SAD_window_size ||
            fin_BM->min_disp                != si->sv->param_bm->min_disp ||
            fin_BM->num_of_disp             != si->sv->param_bm->num_of_disp ||
            fin_BM->texture_thresh          != si->sv->param_bm->texture_thresh ||
            fin_BM->uniquenese_ratio        != si->sv->param_bm->uniquenese_ratio ||
            fin_BM->speckle_window_size     != si->sv->param_bm->speckle_window_size ||
            fin_BM->speckle_range           != si->sv->param_bm->speckle_range ||
            fin_lrf_port                    != ui->comboBox_lrf_com->currentIndex() ||
            fin_lrf_baud_rate               != ui->comboBox_lrf_baudRate->currentIndex() ||
            fin_lrf_scale                   != ui->spinBox_lrf_scale->value() ||
            fin_lrf_res                     != cur_lrf_res) {
        QMessageBox::StandardButton reply = QMessageBox::question(0, "New change", "Parameters were changed, save the new ones?", QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes)
            paramWrite();
    }
}

void MainWindow::initialFusedTopView()
{
    si->initialFusedTopView(ui->label_fusion->width() / 2);

    ui->label_fusion_sv_color->setPixmap(si->pic_sv);
    ui->label_fusion_sv_color->setPixmap(si->pic_sv);
    ui->label_fusion_radar_color->setPixmap(si->pic_radar);

    si->updateFusedTopView();
    ui->label_fusion_BG->setPixmap(QPixmap::fromImage(QImage::QImage(si->fused_topview_BG.data, si->fused_topview_BG.cols, si->fused_topview_BG.rows, si->fused_topview_BG.step, QImage::Format_RGBA8888)));
}

void MainWindow::updateFusedTopView()
{
    si->updateFusedTopView();
    ui->label_fusion_BG->setPixmap(QPixmap::fromImage(QImage::QImage(si->fused_topview_BG.data, si->fused_topview_BG.cols, si->fused_topview_BG.rows, si->fused_topview_BG.step, QImage::Format_RGBA8888)));
    si->updateFusedData();

    si->fg_proc = false;
    dataFused();
}

void MainWindow::on_radioButton_vehicle_cart_clicked()
{
    si->chooseVehicle(VEHICLE::CART);

    f_fused.waitForFinished();
    updateFusedTopView();
}

void MainWindow::on_radioButton_vehicle_car_clicked()
{
    si->chooseVehicle(VEHICLE::CAR);

    f_fused.waitForFinished();
    updateFusedTopView();
}

void MainWindow::on_radioButton_vehicle_tractor_clicked()
{
    si->chooseVehicle(VEHICLE::TRACTOR);

    f_fused.waitForFinished();
    updateFusedTopView();
}
void MainWindow::on_radioButton_vehicle_robot_clicked()
{
    si->chooseVehicle(VEHICLE::ROBOT);

    f_fused.waitForFinished();
    updateFusedTopView();
}

void MainWindow::wheelEvent(QWheelEvent *ev)
{
    // vertical middle button
    if (ev->orientation() == Qt::Vertical) {
        if (ev->delta() > 0) {
            si->zoomInFusedTopView();
        }
        if (ev->delta() < 0) {
            si->zoomOutFusedTopView();
        }
    }

    f_fused.waitForFinished();
    updateFusedTopView();
}

void MainWindow::dataFused()
{
    si->fg_sv = ui->checkBox_fusion_sv->isChecked() && si->sv->fusedTopview() && (fg_capturing || re.vr->fg_loaded);
    si->fg_sv_each_pixel = ui->checkBox_fused_sv_plot_every_pixel->isChecked();
    si->fg_radar = ui->checkBox_fusion_radar->isChecked() && si->rc->fusedTopview() && (fg_retrieving || re.tr->fg_loaded);
    si->fg_data_update = f_sv_status == SV::STATUS::OK || f_radar_status == RADAR::STATUS::OK;
    si->fg_fusion = ui->checkBox_fusion_data->isChecked() && (si->fg_sv && si->fg_radar);
    si->fg_ot = ui->checkBox_ot->isChecked();
    si->fg_ot_trajectory = si->fg_ot && ui->checkBox_ot_trajectory->isChecked();
    si->fg_ot_trajectory_raw = ui->checkBox_ot_trajectory->isChecked() && ui->checkBox_ot_trajectory_raw->isChecked();
    si->fg_ot_trajectory_kalman = ui->checkBox_ot_trajectory->isChecked() && ui->checkBox_ot_trajectory_kalman->isChecked();
    si->fg_ot_vel = si->fg_ot && ui->checkBox_ot_vel->isChecked();
    si->fg_ot_kf = si->fg_ot && ui->checkBox_ot_kf->isChecked();
    si->fg_ot_pf = si->fg_ot && ui->checkBox_ot_pf->isChecked();
    si->fg_ca_astar = ui->checkBox_ca->isChecked() && ui->checkBox_ca_astar->isChecked() && (si->fg_sv || si->fg_radar);
    si->fg_ca_vfh = ui->checkBox_ca->isChecked() && ui->checkBox_ca_vfh->isChecked();

    si->dataExec();
}

void MainWindow::fusedDisplay(cv::Mat *fused_topview, cv::Mat *img_detected_display)
{
    lock_f_sv.lockForRead();
    ui->label_sv_detected_display->setPixmap(QPixmap::fromImage(QImage::QImage(img_detected_display->data, img_detected_display->cols, img_detected_display->rows, img_detected_display->step, QImage::Format_RGB888)).scaled(IMG_DIS_W, IMG_DIS_H));
    lock_f_sv.unlock();
    lock_f_topview.lockForRead();
    ui->label_fusion->setPixmap(QPixmap::fromImage(QImage::QImage(fused_topview->data, fused_topview->cols, fused_topview->rows, fused_topview->step, QImage::Format_RGBA8888)));
    lock_f_topview.unlock();
    ui->label_sv_detected_display->update();
    ui->label_fusion->update();
    qApp->processEvents();
}

void MainWindow::radarDisplayTopViewBG()
{
    if (si->rc->isInitializedTopView()) {
        si->rc->drawTopViewLines(ui->spinBox_radar_topview_r->value(), ui->spinBox_radar_topview_c->value(), false);
        ui->label_top_view_radar_long_BG->setPixmap(QPixmap::fromImage(QImage::QImage(si->rc->topview_BG.data, si->rc->topview_BG.cols, si->rc->topview_BG.rows, si->rc->topview_BG.step, QImage::Format_RGBA8888)).scaled(900, 600));

        ui->label_radar_data_BG->setPixmap(QPixmap::fromImage(QImage::QImage(si->rc->img_radar_BG.data, si->rc->img_radar_BG.cols, si->rc->img_radar_BG.rows, si->rc->img_radar_BG.step, QImage::Format_RGB888)));
    }
    //    cv::imshow("rc", si->rc->topview_BG);
}

void MainWindow::on_spinBox_radar_topview_r_valueChanged(int arg1)
{
    radarDisplayTopViewBG();
}

void MainWindow::on_spinBox_radar_topview_c_valueChanged(int arg1)
{
    radarDisplayTopViewBG();
}

void MainWindow::svDisplayTopViewBG()
{
    if (si->sv->isInitializedTopView()) {
        si->sv->drawTopViewLines(ui->spinBox_topview_r->value(), ui->spinBox_topview_c->value(), true);
        ui->label_top_view_bg->setPixmap(QPixmap::fromImage(QImage::QImage(si->sv->topview_BG.data, si->sv->topview_BG.cols, si->sv->topview_BG.rows, si->sv->topview_BG.step, QImage::Format_RGBA8888)).scaled(270, 750));
    }
}

void MainWindow::on_spinBox_topview_r_valueChanged(int arg1)
{
    svDisplayTopViewBG();
}

void MainWindow::on_spinBox_topview_c_valueChanged(int arg1)
{
    svDisplayTopViewBG();
}

void MainWindow::on_pushButton_lrf_open_clicked()
{
    if (!si->lrf->open(ui->comboBox_lrf_com->currentText(), ui->comboBox_lrf_baudRate->currentText().toInt())) {
        reportError("lrf", "Error!", "Port cannot be open or under using.");
        return;
    }
    else {
        report("Port's opened.");
    }
}

void MainWindow::lrfDisplay(double *lrf_data, cv::Mat *display_lrf)
{
    // display
    ui->label_lrf_data->setPixmap(QPixmap::fromImage(QImage::QImage(display_lrf->data, display_lrf->cols, display_lrf->rows, display_lrf->step, QImage::Format_RGBA8888)).scaled(IMG_W, IMG_W));

    if (fg_lrf_record) {
        lock_lrf.lockForWrite();
        for (int i = 0; i < LENGTH_DATA; i++) {
            fprintf(fp1, "%d ", lrf_data[i]); //**// %f? %d?
        }
        fprintf(fp1, "\n");
        lock_lrf.unlock();
        if (fg_lrf_record_quit) {
            fclose(fp1);
            fg_lrf_record = false;
        }
    }
    ui->label_lrf_buf_proc->setText(QString::number(si->lrf->time_proc_buf));
    ui->label_lrf_proc->setText(QString::number(si->lrf->time_proc));

    ui->label_lrf_data->update();
    ui->label_lrf_buf_proc->update();
    ui->label_lrf_proc->update();
    qApp->processEvents();
}

void MainWindow::retrieveMatchParam()
{
    si->sv->cam_param->cam_focal_length     = fin_cam_param->cam_focal_length;
    si->sv->cam_param->base_line            = fin_cam_param->base_line;
    si->sv->cam_param->focal_length         = fin_cam_param->focal_length;
    si->sv->cam_param->rig_height           = fin_cam_param->rig_height;

    si->sv->param_sgbm->pre_filter_cap      = fin_SGBM->pre_filter_cap;
    si->sv->param_sgbm->SAD_window_size     = fin_SGBM->SAD_window_size;
    si->sv->param_sgbm->min_disp            = fin_SGBM->min_disp;
    si->sv->param_sgbm->num_of_disp         = fin_SGBM->num_of_disp;
    si->sv->param_sgbm->uniquenese_ratio    = fin_SGBM->uniquenese_ratio;
    si->sv->param_sgbm->speckle_window_size = fin_SGBM->speckle_window_size;
    si->sv->param_sgbm->speckle_range       = fin_SGBM->speckle_range;

    si->sv->param_bm->pre_filter_size     = fin_BM->pre_filter_size;
    si->sv->param_bm->pre_filter_cap      = fin_BM->pre_filter_cap;
    si->sv->param_bm->SAD_window_size     = fin_BM->SAD_window_size;
    si->sv->param_bm->min_disp            = fin_BM->min_disp;
    si->sv->param_bm->num_of_disp         = fin_BM->num_of_disp;
    si->sv->param_bm->texture_thresh      = fin_BM->texture_thresh;
    si->sv->param_bm->uniquenese_ratio    = fin_BM->uniquenese_ratio;
    si->sv->param_bm->speckle_window_size = fin_BM->speckle_window_size;
    si->sv->param_bm->speckle_range       = fin_BM->speckle_range;

    si->sv->updateParamsSmp();
}

void MainWindow::camOpen()
{
    int L = ui->comboBox_cam_device_index_L->currentIndex();
    int R = ui->comboBox_cam_device_index_R->currentIndex();

    if (!si->sv->open(L, R)) {
        reportError("sv", "Error!", "Camera can NOT be opened.");
        si->sv->close();
        return;
    }
    else {
        report("Port's opened.");
    }
}

void MainWindow::svDisplay(cv::Mat *img_L, cv::Mat *img_R, cv::Mat *disp, cv::Mat *disp_pseudo, cv::Mat *topview, cv::Mat *img_detected, int detected_obj, int current_frame_count)
{
    ui->label_cam_img_L->setPixmap(QPixmap::fromImage(QImage::QImage(img_L->data, img_L->cols, img_L->rows, img_L->step, QImage::Format_RGB888)).scaled(IMG_DIS_W, IMG_DIS_H));
    ui->label_cam_img_R->setPixmap(QPixmap::fromImage(QImage::QImage(img_R->data, img_R->cols, img_R->rows, img_R->step, QImage::Format_RGB888)).scaled(IMG_DIS_W, IMG_DIS_H));
    ui->label_sv_detected_obj->setText(QString::number(detected_obj));
    if (si->sv->input_mode == SV::INPUT_SOURCE::VIDEO) {
        ui->label_sv_frame_count->setText(QString::number(current_frame_count) + " / " + QString::number(sv_frame_count) + " frames");
        ui->label_sv_frame_count_2->setText(QString::number(current_frame_count) + " / " + QString::number(sv_frame_count) + " frames");
    }
    else if (si->sv->input_mode == SV::INPUT_SOURCE::CAM) {
        ui->label_sv_frame_count->setText(QString::number(current_frame_count) + " frames");
        ui->label_sv_frame_count_2->setText(QString::number(current_frame_count) + " frames");
    }

    if (ui->checkBox_do_depth->isChecked()) {
        lock_sv.lockForRead();
        if (ui->checkBox_pseudo_color->isChecked()) {
            ui->label_disp->setPixmap(QPixmap::fromImage(QImage::QImage(disp_pseudo->data, disp_pseudo->cols, disp_pseudo->rows, disp_pseudo->step, QImage::Format_RGB888)).scaled(IMG_DIS_DISP_W, IMG_DIS_DISP_H));
        }
        else {
            ui->label_disp->setPixmap(QPixmap::fromImage(QImage::QImage(disp->data, disp->cols, disp->rows, disp->step, QImage::Format_Indexed8)).scaled(IMG_DIS_DISP_W, IMG_DIS_DISP_H));
        }

        // update topview
        if (ui->checkBox_sv_topview->isChecked()) {
            ui->label_top_view_sv->setPixmap(QPixmap::fromImage(QImage::QImage(topview->data, topview->cols, topview->rows, topview->step, QImage::Format_RGBA8888)).scaled(270, 750));
            ui->label_sv_detected->setPixmap(QPixmap::fromImage(QImage::QImage(img_detected->data, img_detected->cols, img_detected->rows, img_detected->step, QImage::Format_RGB888)).scaled(IMG_DIS_W, IMG_DIS_H));
        }
        lock_sv.unlock();
    }
    ui->label_sv_proc->setText(QString::number(si->sv->time_proc));

    ui->label_cam_img_L->update();
    ui->label_cam_img_R->update();
    ui->label_sv_detected_obj->update();
    ui->label_sv_frame_count->update();
    ui->label_sv_frame_count_2->update();
    ui->label_disp->update();
    ui->label_top_view_sv->update();
    ui->label_sv_detected->update();
    ui->label_sv_detected_display->update();
    ui->label_sv_proc->update();
    qApp->processEvents();
}

void MainWindow::on_checkBox_sv_topview_clicked(bool checked)
{
    si->sv->fg_topview = checked;
    ui->label_top_view_sv->setVisible(checked);
    ui->checkBox_sv_reproject->setEnabled(checked);
    ui->checkBox_topview_plot_points->setEnabled(checked);
    if (checked)
        ui->label_top_view_sv->setPixmap(QPixmap::fromImage(QImage::QImage(si->sv->topview.data, si->sv->topview.cols, si->sv->topview.rows, si->sv->topview.step, QImage::Format_RGBA8888)).scaled(270, 750));
}

void MainWindow::on_pushButton_cam_open_clicked()
{
    camOpen();
    si->sv->input_mode = SV::INPUT_SOURCE::CAM;
    ui->label_sv_frame_count->setVisible(false);
    ui->label_sv_frame_count_2->setVisible(false);
    on_pushButton_cam_step_clicked();
}

void MainWindow::exec()
{
    if (!fg_running) {
        threadProcessing();
    }
}

bool MainWindow::radarDataIn()
{
    if (si->rc->input_mode == RADAR::INPUT_SOURCE::ESR && !si->rc->isOpened()) {
        reportError("radar", "Error!", "ESR haven't opened.");
        return false;
    }
    else if (si->rc->input_mode == RADAR::INPUT_SOURCE::TXT && !re.tr->fileExist()) {
        reportError("radar", "Error!", "No text file is loaded.");
        return false;
    }

    return true;
}

bool MainWindow::svDataIn()
{
    if (si->sv->input_mode == SV::INPUT_SOURCE::CAM && !si->sv->isOpened()) {
        reportError("sv", "Error!", "Cameras haven't opened.");
        return false;
    }
    else if (si->sv->input_mode == SV::INPUT_SOURCE::VIDEO && !re.vr->fileExist()) {
        reportError("sv", "Error!", "No video is loaded.");
        return false;
    }

    return true;
}

void MainWindow::on_pushButton_cam_step_clicked()
{
    if (fg_capturing) {
        reportError("sv", "Warning!", "The frames are capturing now.");
        return;
    }

    int stat = si->svDataExec();
    dataFused();
    if (stat == SV::STATUS::NO_INPUT) {
        reportError("sv", "Warning.", "No data is capturable.");
        fg_capturing = false;
        threadCheck();
    }
}

void MainWindow::on_pushButton_cam_capture_clicked()
{
    fg_capturing = true;

    exec();
}

void MainWindow::on_pushButton_cam_stop_clicked()
{
    fg_capturing = false;

    threadCheck();
}

void MainWindow::threadCheck()
{
    if (fg_running && !fg_acquiring && !fg_buffering && !fg_capturing && !fg_retrieving) {
        fg_running = false;
    }
}

void MainWindow::threadProcessing()
{
    fg_running = true;
    si->resetTrackingInfo();
#ifndef multi_thread
    bool fg_sv = false, fg_radar = false;
#endif
    while (fg_running) {
        // sv
#ifdef multi_thread
        if (fg_capturing && !f_sv.isRunning()) {
            f_sv = QtConcurrent::run(si, &SensorInfo::svDataExec);
            f_sv_status = f_sv.result();
        }
        if (fg_acquiring && !f_lrf.isRunning()) {
            f_lrf = QtConcurrent::run(si, &SensorInfo::lrfDataExec);
            f_lrf_status = f_lrf.result();
        }
        if (fg_buffering && si->lrf->bufNotFull() && !f_lrf_buf.isRunning()) {
            f_lrf_buf = QtConcurrent::run(si, &SensorInfo::lrfBufExec);
        }
        if (fg_retrieving && !f_radar.isRunning()) {
            f_radar = QtConcurrent::run(si, &SensorInfo::radarDataExec);
            f_radar_status = f_radar.result();
        }
        if (!f_fused.isRunning() && (fg_capturing || fg_retrieving )) {
            f_fused = QtConcurrent::run(this, &MainWindow::dataFused);
        }
#else
        if (fg_capturing) {
            int stat = si->svDataExec();
            if (!fg_sv)
                if (stat == SV::STATUS::OK)
                    fg_sv = true;
        }

        // lrf
        if (fg_acquiring)
            si->lrfDataExec();

        // lrf buffer
        if (fg_buffering)
            si->lrfBufExec();

        // Radar ESR
        if (fg_retrieving) {
            int stat = si->radarDataExec();
            if (!fg_radar)
                if (stat == RADAR::STATUS::OK)
                    fg_radar = true;
        }
        if (fg_sv && fg_radar) {
            dataFused();
            fg_sv = false;
            fg_radar = false;
        }
#endif

        qApp->sendPostedEvents();
        qApp->processEvents();
    }
}

void MainWindow::on_checkBox_do_calibration_clicked(bool checked)
{
    if (checked) {
        // load camera calibration files
        if (!si->sv->loadRemapFile(ui->comboBox_camera_focal_length->currentText().toInt(), ui->lineEdit_base_line->text().toDouble())) {
            reportError("cc", "Error!", "Calibration files can NOT be imported.");
            ui->checkBox_do_calibration->setChecked(false);
        }
        si->sv->fg_calib = true;
    }
    else
        si->sv->fg_calib = false;
}

void MainWindow::on_checkBox_do_depth_clicked(bool checked)
{
    if (checked) {
        // Though disparity is scaled by 16, GUI takes scaled pixels. If not, the topview looks weird.
        si->sv->cam_param->param_r = si->sv->cam_param->focal_length * si->sv->cam_param->base_line;

        ui->checkBox_pseudo_color->setEnabled(true);
        ui->checkBox_sv_topview->setEnabled(true);
        ui->checkBox_sv_reproject->setEnabled(true);
        si->sv->fg_stereoMatch = true;
    }
    else {
        ui->checkBox_pseudo_color->setEnabled(false);
        ui->checkBox_sv_topview->setEnabled(false);
        ui->checkBox_sv_reproject->setEnabled(false);
        si->sv->fg_stereoMatch = false;

    }
}
void MainWindow::on_checkBox_avgfilter_clicked(bool checked)
{
    if(checked)
        si->sv->fg_avgfilter=true;

    else
        si->sv->fg_avgfilter=false;

}

void MainWindow::on_checkBox_pseudo_color_clicked(bool checked)
{
    if (checked)
        si->sv->fg_pseudo = true;
    else
        si->sv->fg_pseudo = false;
}

void MainWindow::on_checkBox_sv_reproject_clicked(bool checked)
{
    if (checked)
        si->sv->fg_reproject = true;
    else
        si->sv->fg_reproject = false;
}

void MainWindow::on_checkBox_topview_plot_points_clicked(bool checked)
{
    if (checked)
        si->sv->fg_topview_plot_points = true;
    else
        si->sv->fg_topview_plot_points = false;
}

void MainWindow::on_pushButton_camera_calibration_clicked()
{
    if (fg_form_calib_alloc) {
        form_calib->raise();
        return;
    }
    form_calib = new calibrationForm();
    form_calib->move(1500, 500);
    fg_form_calib_alloc = true;
    QObject::connect(form_calib, SIGNAL(closed(void)), this, SLOT(closeFormCalib(void)));

    // send focal length & base line to form
    QObject::connect(this, SIGNAL(sendBasicInfo(int, double)), form_calib, SLOT(getBasicInfo(int, double)));
    QObject::connect(form_calib, SIGNAL(requestImage(char)), this, SLOT(requestImage(char)));
    QObject::connect(this, SIGNAL(sendImage(cv::Mat *)), form_calib, SLOT(saveImage(cv::Mat *)));
    QObject::connect(this, SIGNAL(sendImages(cv::Mat *, cv::Mat *)), form_calib, SLOT(saveImages(cv::Mat *, cv::Mat *)));

    emit sendBasicInfo(ui->comboBox_camera_focal_length->currentText().toInt(), ui->lineEdit_base_line->text().toDouble());

    form_calib->show();
}

void MainWindow::closeFormCalib(void)
{
    QObject::disconnect(form_calib, SIGNAL(closed(void)), this, SLOT(closeFormCalib(void)));
    delete form_calib;
    fg_form_calib_alloc = false;
}

void MainWindow::requestImage(char CCD)
{
    if (!svDataIn())
        return;

    switch (CCD) {
    case 'L':
#ifdef debug_info_cc
        qDebug()<<"send left image";
#endif
        emit sendImage(&si->sv->img_r_L);
        break;
    case 'R':
#ifdef debug_info_cc
        qDebug()<<"send right image";
#endif
        emit sendImage(&si->sv->img_r_R);
        break;
    case 'B':
#ifdef debug_info_cc
        qDebug()<<"send both images";
#endif
        emit sendImages(&si->sv->img_r_L, &si->sv->img_r_R);
        break;
    }

}

void MainWindow::on_radioButton_BM_clicked()
{
    if (!si->sv->modeChanged(SV::STEREO_MATCH::BM))
        return;
    report("Change to BM mathod.");
    si->sv->modeChange(SV::STEREO_MATCH::BM, fg_form_smp_alloc);
}

void MainWindow::on_radioButton_SGBM_clicked()
{
    if (!si->sv->modeChanged(SV::STEREO_MATCH::SGBM))
        return;
    report("Change to SGBM mathod.");
    si->sv->modeChange(SV::STEREO_MATCH::SGBM, fg_form_smp_alloc);
}

void MainWindow::on_pushButton_stereo_match_param_clicked()
{
    if (fg_form_smp_alloc) {
        form_smp->raise();
        return;
    }
    form_smp = new stereoMatchParamForm(0, si->sv->match_mode);
    form_smp->move(1500, 100);
    fg_form_smp_alloc = true;
    QObject::connect(form_smp, SIGNAL(closed(void)), this, SLOT(closeFormSmp(void)));

    // send cuurent stereo matching params to ui
    QObject::connect(si->sv, SIGNAL(updateForm(int, std::vector<int>)), form_smp, SLOT(updateParams(int, std::vector<int>)));

    // params connection
    QObject::connect(form_smp, SIGNAL(send_bm_pre_filter_size(int)),        si->sv, SLOT(change_bm_pre_filter_size(int)));
    QObject::connect(form_smp, SIGNAL(send_bm_pre_filter_cap(int)),         si->sv, SLOT(change_bm_pre_filter_cap(int)));
    QObject::connect(form_smp, SIGNAL(send_bm_sad_window_size(int)),        si->sv, SLOT(change_bm_sad_window_size(int)));
    QObject::connect(form_smp, SIGNAL(send_bm_min_disp(int)),               si->sv, SLOT(change_bm_min_disp(int)));
    QObject::connect(form_smp, SIGNAL(send_bm_num_of_disp(int)),            si->sv, SLOT(change_bm_num_of_disp(int)));
    QObject::connect(form_smp, SIGNAL(send_bm_texture_thresh(int)),         si->sv, SLOT(change_bm_texture_thresh(int)));
    QObject::connect(form_smp, SIGNAL(send_bm_uniqueness_ratio(int)),       si->sv, SLOT(change_bm_uniqueness_ratio(int)));
    QObject::connect(form_smp, SIGNAL(send_bm_speckle_window_size(int)),    si->sv, SLOT(change_bm_speckle_window_size(int)));
    QObject::connect(form_smp, SIGNAL(send_bm_speckle_range(int)),          si->sv, SLOT(change_bm_speckle_range(int)));
    QObject::connect(form_smp, SIGNAL(send_sgbm_pre_filter_cap(int)),       si->sv, SLOT(change_sgbm_pre_filter_cap(int)));
    QObject::connect(form_smp, SIGNAL(send_sgbm_sad_window_size(int)),      si->sv, SLOT(change_sgbm_sad_window_size(int)));
    QObject::connect(form_smp, SIGNAL(send_sgbm_min_disp(int)),             si->sv, SLOT(change_sgbm_min_disp(int)));
    QObject::connect(form_smp, SIGNAL(send_sgbm_num_of_disp(int)),          si->sv, SLOT(change_sgbm_num_of_disp(int)));
    QObject::connect(form_smp, SIGNAL(send_sgbm_uniqueness_ratio(int)),     si->sv, SLOT(change_sgbm_uniqueness_ratio(int)));
    QObject::connect(form_smp, SIGNAL(send_sgbm_speckle_window_size(int)),  si->sv, SLOT(change_sgbm_speckle_window_size(int)));
    QObject::connect(form_smp, SIGNAL(send_sgbm_speckle_range(int)),        si->sv, SLOT(change_sgbm_speckle_range(int)));

    si->sv->modeChange(si->sv->match_mode, fg_form_smp_alloc);

    form_smp->show();
}

void MainWindow::closeFormSmp(void)
{
    QObject::disconnect(form_smp, SIGNAL(closed(void)), this, SLOT(closeFormSmp(void)));
    QObject::disconnect(si->sv, SIGNAL(updateForm(int, std::vector<int>)), form_smp, SLOT(updateParams(int, std::vector<int>)));

    // params connection
    QObject::disconnect(form_smp, SIGNAL(send_bm_pre_filter_size(int)),        si->sv, SLOT(change_bm_pre_filter_size(int)));
    QObject::disconnect(form_smp, SIGNAL(send_bm_pre_filter_cap(int)),         si->sv, SLOT(change_bm_pre_filter_cap(int)));
    QObject::disconnect(form_smp, SIGNAL(send_bm_sad_window_size(int)),        si->sv, SLOT(change_bm_sad_window_size(int)));
    QObject::disconnect(form_smp, SIGNAL(send_bm_min_disp(int)),               si->sv, SLOT(change_bm_min_disp(int)));
    QObject::disconnect(form_smp, SIGNAL(send_bm_num_of_disp(int)),            si->sv, SLOT(change_bm_num_of_disp(int)));
    QObject::disconnect(form_smp, SIGNAL(send_bm_texture_thresh(int)),         si->sv, SLOT(change_bm_texture_thresh(int)));
    QObject::disconnect(form_smp, SIGNAL(send_bm_uniqueness_ratio(int)),       si->sv, SLOT(change_bm_uniqueness_ratio(int)));
    QObject::disconnect(form_smp, SIGNAL(send_bm_speckle_window_size(int)),    si->sv, SLOT(change_bm_speckle_window_size(int)));
    QObject::disconnect(form_smp, SIGNAL(send_bm_speckle_range(int)),          si->sv, SLOT(change_bm_speckle_range(int)));
    QObject::disconnect(form_smp, SIGNAL(send_sgbm_pre_filter_cap(int)),       si->sv, SLOT(change_sgbm_pre_filter_cap(int)));
    QObject::disconnect(form_smp, SIGNAL(send_sgbm_sad_window_size(int)),      si->sv, SLOT(change_sgbm_sad_window_size(int)));
    QObject::disconnect(form_smp, SIGNAL(send_sgbm_min_disp(int)),             si->sv, SLOT(change_sgbm_min_disp(int)));
    QObject::disconnect(form_smp, SIGNAL(send_sgbm_num_of_disp(int)),          si->sv, SLOT(change_sgbm_num_of_disp(int)));
    QObject::disconnect(form_smp, SIGNAL(send_sgbm_uniqueness_ratio(int)),     si->sv, SLOT(change_sgbm_uniqueness_ratio(int)));
    QObject::disconnect(form_smp, SIGNAL(send_sgbm_speckle_window_size(int)),  si->sv, SLOT(change_sgbm_speckle_window_size(int)));
    QObject::disconnect(form_smp, SIGNAL(send_sgbm_speckle_range(int)),        si->sv, SLOT(change_sgbm_speckle_range(int)));

    delete form_smp;
    fg_form_smp_alloc = false;
}

void MainWindow::on_comboBox_camera_focal_length_currentIndexChanged(int index)
{
    if (fg_capturing) {
        if (ui->checkBox_do_calibration->isChecked())
            on_checkBox_do_calibration_clicked(true);
        report("focal length has been changed.");
    }
}

void MainWindow::on_lineEdit_base_line_returnPressed()
{
    if (fg_capturing) {
        on_checkBox_do_calibration_clicked(true);
        report("base line has been changed.");
    }
}

void MainWindow::on_pushButton_lrf_request_ONCE_clicked()
{
    si->lrf->requestData(LRF::CAPTURE_MODE::ONCE);
    while (!si->lrf->bufEnoughSet()) {
        si->lrfBufExec();
    }
    si->lrfDataExec();
}

void MainWindow::on_pushButton_lrf_request_clicked()
{
    si->lrf->requestData(LRF::CAPTURE_MODE::CONTINUOUS);

    // push data to buffer //**// shouldn't be here
    fg_buffering = true;

    exec();
}

void MainWindow::on_pushButton_lrf_retrieve_clicked()
{
    fg_acquiring = true;

    exec();
}

void MainWindow::on_pushButton_lrf_stop_clicked()
{
    fg_buffering = false;
    fg_acquiring = false;

    si->lrf->stopRetrieve();

    threadCheck();

#ifdef debug_info_main
    qDebug()<<"stop done";
#endif
}

void MainWindow::on_lineEdit_sv_rig_height_returnPressed()
{
    si->sv->cam_param->rig_height = ui->lineEdit_sv_rig_height->text().toDouble();
    ui->label_sv_rig_height->setText(ui->lineEdit_sv_rig_height->text());
}

void MainWindow::on_lineEdit_sv_focal_length_returnPressed()
{
    si->sv->cam_param->focal_length = ui->lineEdit_sv_focal_length->text().toDouble();
    ui->label_sv_focal_length->setText(ui->lineEdit_sv_focal_length->text());
    on_checkBox_do_depth_clicked(true);
}

void MainWindow::on_pushButton_lrf_record_data_clicked()
{
    //    lrf_temp.clear();
    //    for (int i = 0; i < LENGTH_DATA; i++) {
    //        lrf_temp.push_back(lrf_data[i]);
    //    }
    //    display_lrf_3D.push_back(lrf_temp);

    fp1 = fopen("D_lrf.txt", "a");
    fg_lrf_record = true;
    //    FILE* fp1;
    //    fp1 = fopen("D_lrf.txt", "a");
    //    for (int i = 0; i < LENGTH_DATA; i++) {
    //        fprintf(fp1, "%f ", lrf_data[i]);
    //    }
    //    fprintf(fp1, "\n");
    //    fclose(fp1);
}

void MainWindow::on_pushButton_sv_record_data_clicked()
{
    FILE* fp;
    fp = fopen("D_sv.txt", "w");

    for (int r = 0; r < IMG_H; r++) {
        for (int c = 0; c < IMG_W; c++) {
            fprintf(fp, "%d ", si->sv->data[r][c].disp);
        }
        fprintf(fp, "\n");
    }

    fclose(fp);

    cv::Mat temp_L, temp_R;
    cv::cvtColor(si->sv->img_r_L, temp_L, cv::COLOR_RGB2BGR);
    cv::cvtColor(si->sv->img_r_R, temp_R, cv::COLOR_RGB2BGR);
    cv::imwrite("D_sv_L.jpg", temp_L);
    cv::imwrite("D_sv_R.jpg", temp_R);
}

void MainWindow::on_pushButton_lrf_record_stop_clicked()
{
    fg_lrf_record_quit = true;
}

void MainWindow::on_pushButton_sv_read_images_clicked()
{
    QStringList imgs_path = QFileDialog::getOpenFileNames(0, "Load images. Choose left and right sequentially.", ".", tr("Image files (*.jpg, *.png)"));
    if (imgs_path.empty())
        return;
    si->sv->input_mode = SV::INPUT_SOURCE::IMG;

    si->sv->img_L = cv::imread(imgs_path[0].toStdString());
    si->sv->img_R = cv::imread(imgs_path[1].toStdString());
    cv::Size2f ratio;
    ratio.width = 1.0 * IMG_W / si->sv->img_L.cols;
    ratio.height = 1.0 * IMG_H / si->sv->img_L.rows;
    double ratio_zoom = ratio.width < ratio.height ? ratio.width : ratio.height;
    cv::Size size_zoom = cv::Size(ratio_zoom * si->sv->img_L.cols, ratio_zoom * si->sv->img_L.rows);
    cv::resize(si->sv->img_L, si->sv->img_L, size_zoom);
    cv::resize(si->sv->img_R, si->sv->img_R, size_zoom);
    si->sv->dataExec();
}

void MainWindow::readFromTxt(QString file_name, cv::Mat *output)
{
    QFile file(file_name);
    file.open(QIODevice::ReadOnly);
    if(!file.isOpen())
        return;
    QTextStream in(&file);

    std::vector <std::vector <short int> > img_d;
    std::vector <short int> temp;
    QString content, value;
    while (!in.atEnd()) {
        temp.clear();
        content.clear();
        content = file.readLine();
        for (int i = 0; i < content.size(); i++) {
            QChar va = content.at(i);
            if (va != ' ' && va != '\t')
                value.append(va);
            else {
                temp.push_back(value.toInt());
                value.clear();
            }
        }
        img_d.push_back(temp);
    }
    file.close();

    int row = img_d.size();
    int col = img_d[0].size();
    output->create(row, col, CV_16S);
    for (int r = 0; r < row; r++) {
        short int* ptr = output->ptr<short int>(r);
        for (int c = 0; c < col; c++) {
            ptr[c] = img_d[r][c];
            //            std::cout<<ptr[c]<<" ";
        }
        //        std::cout<<std::endl;
    }
}

void MainWindow::on_pushButton_sv_read_disp_clicked()
{
    cv::Mat img_sv;
    cv::Mat img_sv_r;
    cv::Mat hist = cv::Mat::zeros(1, 3000, CV_32S);
    cv::Mat hist_id = cv::Mat(1, 3000, CV_16S);
    for (short int i = 0; i < 3000; i++)
        hist_id.ptr<short int>(0)[i] = i;
    QString file_name = "D_sv.txt";
    readFromTxt(file_name, &img_sv);

    QTime countt;
    countt.start();
    // display
    img_sv_r.create(img_sv.rows, img_sv.cols, CV_8UC1);
    for (int r = 0; r < img_sv.rows; r++) {
        uchar* ptr_1 = img_sv_r.ptr<uchar>(r);
        short int* ptr = img_sv.ptr<short int>(r);
        for (int c = 0; c < img_sv.cols; c++) {
            int va = ptr[c];
            int va_1 = 1.0 * va / 8.0;
            if (va_1 >= 256)
                ptr_1[c] = 255;
            else if (va_1 <= 0)
                ptr_1[c] = 0;
            else
                ptr_1[c] = va_1;
            if (va == -16) {
                hist.ptr<long int>(0)[0]++;
            }
            else {
                hist.ptr<long int>(0)[va]++;
            }
            //            std::cout<<ptr[c]<<" ";
        }
        //        std::cout<<std::endl;
    }
    cv::imshow("[sv] new", img_sv_r);
    cv::imwrite("D_sv_disp.jpg", img_sv_r);

    // sorting
    cv::Mat hist_sort = hist.clone();
    for (int i = 0; i < hist_sort.cols - 2; i++) {
        for (int j = i + 1; j < hist_sort.cols - 1; j++) {
            if (hist_sort.ptr<long int>(0)[i] < hist_sort.ptr<long int>(0)[j]) {
                long int tempp = hist_sort.ptr<long int>(0)[j];
                hist_sort.ptr<long int>(0)[j] = hist_sort.ptr<long int>(0)[i];
                hist_sort.ptr<long int>(0)[i] = tempp;

                int tempp1 = hist_id.ptr<short int>(0)[j];
                hist_id.ptr<short int>(0)[j] = hist_id.ptr<short int>(0)[i];
                hist_id.ptr<short int>(0)[i] = tempp1;
            }
            continue;
        }
    }

    //    int img_total = 0;
    //    for (int i = 0; i < 3000; i++) {
    //        img_total += hist_sort.ptr<long int>(0)[i];
    //    }

    cv::Mat img_sv_separate;
    img_sv_separate.create(img_sv.rows, img_sv.cols, CV_8UC1);
    for (int k = 0; k < 3000; k++) {
        img_sv_separate.setTo(0);
        for (int r = 0; r < img_sv.rows; r++) {
            uchar* ptr_1 = img_sv_separate.ptr<uchar>(r);
            short int* ptr = img_sv.ptr<short int>(r);
            for (int c = 0; c < img_sv.cols; c++) {
                int dev = 20;
                if ((int)(ptr[c]) >= (int)(hist_id.ptr<short int>(0)[k]) - dev &&
                        (int)(ptr[c]) <= (int)(hist_id.ptr<short int>(0)[k]) + dev &&
                        hist_id.ptr<short int>(0)[k] >= 20 && hist_id.ptr<short int>(0)[k] < 1800 &&
                        hist_sort.ptr<long int>(0)[k] > 300) {
                    ptr_1[c] = 255;//1.0 * ptr[c] / 8.0;
                }
            }
        }
        QString name = "D_sv_disp_" + QString::number(k) + ".jpg";
        cv::imshow(name.toStdString(), img_sv_separate);
        static char c;
        c = cv::waitKey(0);
        if (c == 'q') {
            cv::destroyAllWindows();
            break;
        }
        else if (c == 's')
            cv::imwrite(name.toStdString(), img_sv_separate);
    }
}

void MainWindow::on_pushButton_lrf_read_range_clicked()
{
    cv::Mat img_lrf;
    cv::Mat img_lrf_r;
    QString file_name = "D_lrf.txt";
    readFromTxt(file_name, &img_lrf);
    cv::transpose(img_lrf, img_lrf);
    cv::resize(img_lrf, img_lrf, cv::Size(img_lrf.cols, (int)(2.2 * img_lrf.rows)));
    img_lrf_r.create(img_lrf.rows, img_lrf.cols, CV_8UC1);
    for (int r = 0; r < img_lrf.rows; r++) {
        uchar* ptr_1 = img_lrf_r.ptr<uchar>(r);
        short int* ptr = img_lrf.ptr<short int>(r);
        for (int c = 0; c < img_lrf.cols; c++) {
            ptr_1[c] = 1.0 * ptr[c] / 35.0;
        }
    }
    cv::imshow("[lrf] new", img_lrf_r);
    cv::imwrite("D_lrf.jpg", img_lrf_r);

    cv::Mat img_lrf_proc = cv::Mat::zeros(img_lrf.rows, img_lrf.cols, CV_8UC1);
    int pxl_dev = 1;
    for (int r = 0; r < img_lrf.rows - pxl_dev; r++) {
        short int* ptr = img_lrf.ptr<short int>(r);
        uchar* ptr_p = img_lrf_proc.ptr<uchar>(r);
        for (int c = 0; c < img_lrf.cols - pxl_dev; c++) {
            int neighbor = ptr[c + pxl_dev];
            int smaller = ptr[c] > neighbor ? neighbor : ptr[c];
            int bigger = ptr[c] > neighbor ? ptr[c] : neighbor;
            double ratio = 1.0 * (bigger - smaller) / smaller;
            neighbor = img_lrf.ptr<short int>(r + pxl_dev)[c];
            smaller = ptr[c] > neighbor ? neighbor : ptr[c];
            bigger = ptr[c] > neighbor ? ptr[c] : neighbor;
            double ratio_1 = 1.0 * (bigger - smaller) / smaller;
            if (ratio > 0.5 || ratio_1 > 0.5) {
                ptr_p[c] = 255;
            }
            //            std::cout<<ratio<<" ";
        }
        //        std::cout<<std::endl;
    }

    cv::imshow("proc lrf", img_lrf_proc);
    cv::imwrite("D_lrf_edge.jpg", img_lrf_proc);
}

void MainWindow::on_pushButton_lrf_read_range_2_clicked()
{
    cv::Mat img_lrf;
    cv::Mat img_lrf_r;
    QString file_name = "D_lrf.txt";
    readFromTxt(file_name, &img_lrf);
    cv::transpose(img_lrf, img_lrf);
    cv::resize(img_lrf, img_lrf, cv::Size(img_lrf.cols * 2, img_lrf.rows * 2));
    img_lrf_r.create(img_lrf.rows, img_lrf.cols, CV_8UC1);
    double minn = 10000.0;
    double maxx = 0.0;
    for (int r = 0; r < img_lrf.rows; r++) {
        short int* ptr = img_lrf.ptr<short int>(r);
        for (int c = 0; c < img_lrf.cols; c++) {
            if (ptr[c] < minn)
                minn = ptr[c];
            if (ptr[c] > maxx)
                maxx = ptr[c];
        }
    }
    for (int r = 0; r < img_lrf.rows; r++) {
        uchar* ptr_1 = img_lrf_r.ptr<uchar>(r);
        short int* ptr = img_lrf.ptr<short int>(r);
        for (int c = 0; c < img_lrf.cols; c++) {
            int p = ptr[c];
            double va = ((255.0 * (p - minn)) / (1.0 * (maxx - minn)));
            if (va > 255.0)
                va = 255;
            else if (va < 0.0)
                va = 0;
            ptr_1[c] = (int)(va);
            //            std::cout<<va<<"/"<<p<<" ";

        }
        //        std::cout<<std::endl;
    }
    //    std::cout<<std::endl<<minn<<", "<<maxx<<std::endl;

    cv::imshow("[lrf] new", img_lrf_r);
    cv::imwrite("D_lrf.jpg", img_lrf_r);
    cv::Mat img_lrf_edge;
    cv::Canny(img_lrf_r, img_lrf_edge, ui->horizontalSlider->value(), ui->horizontalSlider_2->value());
    cv::imshow("edge", img_lrf_edge);
    cv::imwrite("D_lrf_edge.jpg", img_lrf_edge);
}

void MainWindow::on_horizontalSlider_2_sliderReleased()
{
    on_pushButton_lrf_read_range_2_clicked();
}

void MainWindow::on_horizontalSlider_sliderReleased()
{
    on_pushButton_lrf_read_range_2_clicked();
}

void MainWindow::on_pushButton_lrf_request_2_clicked()
{
    si->lrf->requestData(LRF::CAPTURE_MODE::CONTINUOUS);

    fg_buffering = true;
    fg_acquiring = true;
}

void MainWindow::on_pushButton_lrf_retrieve_2_clicked()
{
    while (fg_acquiring) {
        si->lrfDataExec();
        si->lrfBufExec();

        qApp->processEvents();
    }
}

void MainWindow::on_pushButton_lrf_stop_2_clicked()
{
    fg_buffering = false;
    fg_acquiring = false;

    si->lrf->stopRetrieve();
    threadCheck();
}

void MainWindow::on_pushButton_radar_open_clicked()
{
    if (si->rc->open())
        report("Radar's Opened");
    else
        reportError("radar", "Error!", "Failed to open");
}

void MainWindow::on_pushButton_radar_write_clicked()
{
    if (si->rc->write())
        report("send cmd complete, data start!");
    else
        reportError("radar", "Error!", "Failed to send cmd to radar.");
}

void MainWindow::on_pushButton_radar_bus_on_clicked()
{
    si->rc->busOn();

    fg_retrieving = true;

    exec();
}

void MainWindow::on_pushButton_radar_bus_off_clicked()
{
    fg_retrieving = false;

    si->rc->busOff();
    threadCheck();
}

void MainWindow::on_pushButton_radar_step_clicked()
{
    int stat;
    while (stat != RADAR::STATUS::OK) {
        stat = si->radarDataExec();
        if (stat == RADAR::STATUS::NO_INPUT)
            reportError("radar", "Warning.", "No data is retrievable.");
    }
}

void MainWindow::radarDisplay(int detected_obj, cv::Mat *img, cv::Mat *topview)
{
    ui->label_radar_detected_obj->setText(QString::number(detected_obj));
    lock_radar.lockForRead();
    ui->label_radar_data->setPixmap(QPixmap::fromImage(QImage::QImage(img->data, img->cols, img->rows, QImage::Format_RGBA8888)));
    lock_radar.unlock();

    // update topview
    if (ui->checkBox_radar_topview->isChecked() && si->rc->current_count >= si->rc->update_count) {
        lock_radar.lockForRead();
        ui->label_top_view_radar_long->setPixmap(QPixmap::fromImage(QImage::QImage(topview->data, topview->cols, topview->rows, topview->step, QImage::Format_RGBA8888)).scaled(900, 600));
        lock_radar.unlock();
    }

    // computing time
    ui->label_radar_proc->setText(QString::number(si->rc->time_proc));

    ui->label_radar_detected_obj->update();
    ui->label_radar_data->update();
    ui->label_top_view_radar_long->update();
    ui->label_radar_proc->update();
    qApp->processEvents();
}

void MainWindow::on_checkBox_radar_topview_clicked(bool checked)
{
    si->rc->fg_topview = checked;
    ui->label_top_view_radar_long->setVisible(checked);
    if (checked)
        ui->label_top_view_radar_long->setPixmap(QPixmap::fromImage(QImage::QImage(si->rc->topview.data, si->rc->topview.cols, si->rc->topview.rows, si->rc->topview.step, QImage::Format_RGBA8888)).scaled(900, 600));
}

void MainWindow::on_pushButton_start_all_clicked()
{
    if (!svDataIn() || !radarDataIn())
        return;

    if (si->sv->input_mode == SV::INPUT_SOURCE::CAM && si->rc->input_mode == RADAR::INPUT_SOURCE::ESR) {
        inputType(INPUT_TYPE::DEVICE);

        if (!fg_capturing)
            on_pushButton_cam_open_clicked();

        if (!fg_retrieving) {
            on_pushButton_radar_open_clicked();
            si->rc->busOn();
            on_pushButton_radar_write_clicked();
        }
    }
    else
        inputType(INPUT_TYPE::RECORDING);

    if (!fg_capturing) {
        fg_capturing = true;
    }
    if (!fg_retrieving) {
        fg_retrieving = true;
    }

    exec();
}

void MainWindow::on_pushButton_stop_all_clicked()
{
    on_pushButton_cam_stop_clicked();
    on_pushButton_radar_bus_off_clicked();
    //    re.tr->file.close();
}

void MainWindow::on_actionShortcut_triggered()
{

}

void MainWindow::on_actionAuthor_triggered()
{
    int l_widget = 400;
    int l_icon = 200;
    int x, y;
    x = y = (l_widget - l_icon) / 2;

    devForm = new QDialog;
    icon = new QLabel(devForm);
    text = new QLabel(devForm);
    devForm->setGeometry(100, 100, l_widget, l_widget);
    icon->setGeometry(x, y, l_icon, l_icon);
    icon->setPixmap(QPixmap(":/icon/iconES.png", 0, Qt::AutoColor).scaled(l_icon, l_icon));
    text->setGeometry(0, l_icon * 1.7, l_widget, l_icon / 2);
    text->setText("Contributors: Li-Kang Weng, An-Chih Tsai, Kai-Chung Chuang.");
    text->setAlignment(Qt::AlignHCenter);
    devForm->show();

    fg_author = true;
}

void MainWindow::releaseAuthor()
{
    if (fg_author) {
        devForm->close();
        delete icon;
        delete text;
        delete devForm;
    }
}

void MainWindow::on_pushButton_sv_record_clicked()
{
    // record goes on
    if (!re.vr->fg_record) {
        ui->pushButton_sv_record->setIcon(QIcon(":/icon/record_on.png"));
        ui->label_sv_frame_count->setText("0");
        ui->label_sv_frame_count->setVisible(true);
        ui->label_sv_frame_count_2->setText("0");
        ui->label_sv_frame_count_2->setVisible(true);

        re.start(RECORD_TYPE::VIDEO);
    }
    // record goes off
    else {
        ui->pushButton_sv_record->setIcon(QIcon(":/icon/record_off.png"));
        re.stop();
        ui->label_sv_frame_count->setVisible(false);
        ui->label_sv_frame_count_2->setVisible(false);
    }
}

void MainWindow::on_pushButton_radar_record_clicked()
{
    // record goes on
    if (!re.tr->fg_record) {
        ui->pushButton_radar_record->setIcon(QIcon(":/icon/record_on.png"));
        re.start(RECORD_TYPE::TXT);
    }
    // record goes off
    else {
        ui->pushButton_radar_record->setIcon(QIcon(":/icon/record_off.png"));
        re.stop();
    }
}

void MainWindow::on_pushButton_lrf_record_clicked()
{

}

void MainWindow::on_pushButton_all_record_clicked()
{
    // record goes on
    if (!(re.tr->fg_record && re.vr->fg_record)) {
        if (fg_capturing)
            ui->pushButton_sv_record->setIcon(QIcon(":/icon/record_on.png"));
        if (fg_retrieving)
            ui->pushButton_radar_record->setIcon(QIcon(":/icon/record_on.png"));
        ui->pushButton_all_record->setIcon(QIcon(":/icon/all_record_on.png"));
        ui->label_sv_frame_count->setText("0");
        ui->label_sv_frame_count->setVisible(true);
        ui->label_sv_frame_count_2->setText("0");
        ui->label_sv_frame_count_2->setVisible(true);
        re.start(RECORD_TYPE::ALL);
    }
    // record goes off
    else {
        ui->pushButton_sv_record->setIcon(QIcon(":/icon/record_off.png"));
        ui->pushButton_radar_record->setIcon(QIcon(":/icon/record_off.png"));
        ui->pushButton_all_record->setIcon(QIcon(":/icon/all_record_off.png"));
        re.stop();
        ui->label_sv_frame_count->setVisible(false);
        ui->label_sv_frame_count_2->setVisible(false);
    }
}

void MainWindow::loadData(int record_type)
{
    re.setRecordType(record_type);
    inputType(INPUT_TYPE::RECORDING);
    re.vr->fg_data_end = false;
    re.tr->fg_data_end = false;
    QString file_loaded = re.loadData();
    if (record_type == RECORD_TYPE::VIDEO || record_type == RECORD_TYPE::ALL) {
        sv_frame_count = re.vr->frame_count;
        ui->label_sv_frame_count->setText("0 / " + QString::number(sv_frame_count) + " frames");
        ui->label_sv_frame_count->setVisible(true);
        ui->label_sv_frame_count_2->setText("0 / " + QString::number(sv_frame_count) + " frames");
        ui->label_sv_frame_count_2->setVisible(true);
    }

    label_file_loaded->setText(file_loaded);
}

void MainWindow::on_pushButton_sv_load_data_clicked()
{
    loadData(RECORD_TYPE::VIDEO);
}

void MainWindow::on_pushButton_radar_load_data_clicked()
{
    loadData(RECORD_TYPE::TXT);
}

void MainWindow::on_pushButton_lrf_load_data_clicked()
{

}

void MainWindow::on_pushButton_all_load_data_clicked()
{
    loadData(RECORD_TYPE::ALL);
}

void MainWindow::videoIsEnd()
{
    if (re.vr->fg_data_end)
        return;

    on_pushButton_cam_stop_clicked();
    report("Video is end.");

    re.vr->fg_data_end = true;
}

void MainWindow::dataIsEnd()
{
    if (re.tr->fg_data_end)
        return;

    on_pushButton_radar_bus_off_clicked();

    report("Data is end.");

    re.tr->file.close();
    re.tr->fg_data_end = true;
}

void MainWindow::inputType(int type)
{
    switch (type) {
    case INPUT_TYPE::DEVICE:
        si->sv->input_mode = SV::INPUT_SOURCE::CAM;
        si->rc->input_mode = RADAR::INPUT_SOURCE::ESR;
        ui->radioButton_input_device->setChecked(true);
        break;
    case INPUT_TYPE::RECORDING:
        si->sv->input_mode = SV::INPUT_SOURCE::VIDEO;
        si->rc->input_mode = RADAR::INPUT_SOURCE::TXT;
        ui->radioButton_input_recording->setChecked(true);
        break;
    }
}

void MainWindow::on_radioButton_input_device_clicked()
{
    inputType(INPUT_TYPE::DEVICE);
}

void MainWindow::on_radioButton_input_recording_clicked()
{
    inputType(INPUT_TYPE::RECORDING);
}

void MainWindow::on_checkBox_sv_ground_filter_clicked(bool checked)
{
    if (checked)
        si->sv->setGroundFilter(true);
    else
        si->sv->setGroundFilter(false);
}

void MainWindow::on_checkBox_fusion_sv_clicked(bool checked)
{
    if (checked && ui->checkBox_fusion_radar->isChecked()) {
        ui->checkBox_fusion_data->setEnabled(checked);
        ui->checkBox_fusion_data->setChecked(checked);
    }
    else {
        ui->checkBox_fusion_data->setEnabled(checked);
        ui->checkBox_fusion_data->setChecked(checked);
    }
}

void MainWindow::on_checkBox_fusion_radar_clicked(bool checked)
{
    if (checked && ui->checkBox_fusion_sv->isChecked()) {
        ui->checkBox_fusion_data->setEnabled(checked);
        ui->checkBox_fusion_data->setChecked(checked);
    }
    else {
        ui->checkBox_fusion_data->setEnabled(checked);
        ui->checkBox_fusion_data->setChecked(checked);
    }
}

void MainWindow::on_spinBox_lrf_scale_valueChanged(int arg1)
{
    if (fg_acquiring)
        si->lrf->scale_ratio = arg1;
}

void MainWindow::on_checkBox_ca_clicked(bool checked)
{
    ui->checkBox_ca_astar->setEnabled(checked);
    ui->checkBox_ca_vfh->setEnabled(checked);
}

void MainWindow::on_checkBox_ot_clicked(bool checked)
{
    ui->checkBox_ot_trajectory->setEnabled(checked);
    ui->checkBox_ot_trajectory_raw->setEnabled(checked);
    ui->checkBox_ot_trajectory_kalman->setEnabled(checked);
    ui->checkBox_ot_vel->setEnabled(checked);
    ui->checkBox_ot_kf->setEnabled(checked);
    ui->checkBox_ot_pf->setEnabled(checked);

#ifdef debug_info_sv_object_matching_img
    cv::namedWindow("Comp", cv::WINDOW_AUTOSIZE);
#endif
}

void MainWindow::on_checkBox_sv_matching_clicked(bool checked)
{
    si->sv->fg_tracking = checked;
}

void MainWindow::on_checkBox_ca_astar_clicked(bool checked)
{

}

void MainWindow::on_horizontalSlider_ana_range_filter_min_valueChanged(int value)
{
    si->range_filter_min = value;
}

void MainWindow::on_horizontalSlider_ana_range_filter_max_valueChanged(int value)
{
    si->range_filter_max = value;
}


void MainWindow::on_pushButton_step_all_clicked()
{
    if (!svDataIn() || !radarDataIn())
        return;

    if (si->sv->input_mode == SV::INPUT_SOURCE::CAM && si->rc->input_mode == RADAR::INPUT_SOURCE::ESR) {
        inputType(INPUT_TYPE::DEVICE);

        if (!fg_capturing)
            on_pushButton_cam_open_clicked();

        if (!fg_retrieving) {
            on_pushButton_radar_open_clicked();
            si->rc->busOn();
            on_pushButton_radar_write_clicked();
        }
    }
    else
        inputType(INPUT_TYPE::RECORDING);

    if (!fg_capturing) {
        fg_capturing = true;
    }
    if (!fg_retrieving) {
        fg_retrieving = true;
    }

    if (fg_capturing) {
        f_sv_status = si->svDataExec();
    }
    if (fg_acquiring) {
        f_lrf_status = si->lrfDataExec();
    }
    if (fg_buffering && si->lrf->bufNotFull()) {
        si->lrfBufExec();
    }
    if (fg_retrieving) {
        f_radar_status = si->radarDataExec();
    }
    if (fg_capturing || fg_retrieving ) {
        dataFused();
    }
}

void MainWindow::on_checkBox_ot_trajectory_clicked(bool checked)
{
    ui->checkBox_ot_trajectory_raw->setEnabled(checked);
    ui->checkBox_ot_trajectory_kalman->setEnabled(checked);
}

void MainWindow::guiDisplay(int type, bool fg_on)
{
    ui->label_gui_sign->setVisible(fg_on);
    switch (type) {
    case GUI::APPROACHING:
        ui->label_gui_approaching->setVisible(fg_on);
        ui->label_gui_approaching->update();
        break;
    case GUI::LEFT_ONCOMING:
        ui->label_gui_left_oncoming->setVisible(fg_on);
        ui->label_gui_left_oncoming->update();
        break;
    case GUI::RIGHT_ONCOMING:
        ui->label_gui_right_oncoming->setVisible(fg_on);
        ui->label_gui_right_oncoming->update();
        break;
    }
    ui->label_gui_sign->update();
    qApp->sendPostedEvents();
    qApp->processEvents();
}


void MainWindow::on_pushButton_left_clicked()
{
    left_dcmotor->Open(ui->comboBox_COMport_left->currentText(),ui->comboBox_baudrate_left->currentText().toInt());


    if (left_dcmotor->isOpen())
        ui->textBrowser->append(
                    "Left DC Motor: "
                    + ui->comboBox_COMport_left->currentText()
                    + " Baud:" + ui->comboBox_baudrate_left->currentText()
                    + " is opened");
    else
        ui->textBrowser->append("[Error] Left DC Motor: cannot open " +
                                ui->comboBox_COMport_left->currentText()
                                + " Baud:" + ui->comboBox_baudrate_left->currentText());
}

void MainWindow::on_pushButton_right_clicked()
{
    right_dcmotor->Open(ui->comboBox_COMport_right->currentText(),ui->comboBox_baudrate_right->currentText().toInt());
    if (right_dcmotor->isOpen())
        ui->textBrowser->append(
                    "Right DC Motor: "
                    + ui->comboBox_COMport_right->currentText()
                    + " Baud:" + ui->comboBox_baudrate_right->currentText()
                    + " is opened");
    else
        ui->textBrowser->append("[Error] Right DC Motor: cannot open " +
                                ui->comboBox_COMport_right->currentText()
                                + " Baud:" + ui->comboBox_baudrate_right->currentText());
}

void MainWindow::on_pushButton_initial_clicked()
{
    ui->pushButton_left->click();
    ui->pushButton_right->click();
    if (right_dcmotor->isOpen() && left_dcmotor->isOpen())
    {
        left_dcmotor->SetHome();
        right_dcmotor->SetHome();
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(right_dcmotor->isOpen())
                                .arg(left_dcmotor->isOpen()));
    }
}

void MainWindow::on_pushButton_R_clicked()
{
    if (right_dcmotor->isOpen() && left_dcmotor->isOpen())
    {
        right_dcmotor->D(ui->doubleSpinBox_velocity->value()*-1);
        left_dcmotor->D(ui->doubleSpinBox_velocity->value());
        ui->textBrowser->append(QString("Run"));
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(right_dcmotor->isOpen())
                                .arg(left_dcmotor->isOpen()));
    }
}

void MainWindow::on_pushButton_D_clicked()
{
    if (right_dcmotor->isOpen() && left_dcmotor->isOpen())
    {
        right_dcmotor->D(ui->doubleSpinBox_velocity->value());
        left_dcmotor->D(ui->doubleSpinBox_velocity->value()*-1);
        ui->textBrowser->append(QString("Run"));
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(right_dcmotor->isOpen())
                                .arg(left_dcmotor->isOpen()));
    }
}

void MainWindow::on_pushButton_stop_clicked()
{
    if (right_dcmotor->isOpen() && left_dcmotor->isOpen())
    {
        right_dcmotor->stop();
        left_dcmotor->stop();
        ui->textBrowser->append(QString("Stop"));
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(right_dcmotor->isOpen())
                                .arg(left_dcmotor->isOpen()));
    }
}

void MainWindow::on_pushButton_front_clicked()
{
    if (right_dcmotor->isOpen() && left_dcmotor->isOpen())
    {
        ui->textBrowser->append(QString("Go Forward %1m").arg(ui->doubleSpinBox_distance->value()));
        GoForward(ui->doubleSpinBox_distance->value(),ui->doubleSpinBox_velocity->value());
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(right_dcmotor->isOpen())
                                .arg(left_dcmotor->isOpen()));
    }
}

void MainWindow::GoForward(double distance_m, double speed)
{
    right_dcmotor->SetMaxVelocity(speed);
    left_dcmotor->SetMaxVelocity(-speed);
    double pi=3.14159265;
    // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
    // calibration coeffient: y = 0.9487x
    int value = static_cast<int>((4096*3.333*14)*(distance_m/0.325)/(pi)/0.9487);
    //    std::cout << "value = " << value << std::endl;
    right_dcmotor->RotateRelativeDistancce(value);
    left_dcmotor->RotateRelativeDistancce(-value);
}

void MainWindow::GoBackward(double distance_m, double speed)
{
    right_dcmotor->SetMaxVelocity(-speed);
    left_dcmotor->SetMaxVelocity(speed);
    double pi=3.14159265;
    // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
    // calibration coeffient: y = 0.9487x
    int value = static_cast<int>((4096*3.333*14)*(distance_m/0.325)/(pi)/0.9487);
    //    std::cout << "value = " << value << std::endl;
    right_dcmotor->RotateRelativeDistancce(-value);
    left_dcmotor->RotateRelativeDistancce(value);
}

void MainWindow::TurnLeft(double angle, double speed)
{
    if (angle > 0 && angle <= 360)
    {
        right_dcmotor->SetMaxVelocity(speed);
        left_dcmotor->SetMaxVelocity(speed);

        // distance between wheels = 0.57m
        // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
        // calibration coeffient: y = 0.9487x
        int value = static_cast<int>((4096*3.333*14)*(0.57/0.325)*angle/360/0.9487);
        right_dcmotor->RotateRelativeDistancce(value);
        left_dcmotor->RotateRelativeDistancce(value);
    }
}

void MainWindow::TurnRight(double angle, double speed)
{
    if (angle > 0 && angle <= 360)
    {
        right_dcmotor->SetMaxVelocity(-speed);
        left_dcmotor->SetMaxVelocity(-speed);

        // distance between wheels = 0.57m
        // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
        // calibration coeffient: y = 0.9487x
        int value = static_cast<int>((4096*3.333*14)*(0.57/0.325)*angle/360/0.9487);
        right_dcmotor->RotateRelativeDistancce(-value);
        left_dcmotor->RotateRelativeDistancce(-value);
    }
}

void MainWindow::TurnLeftOneLeg(double angle, double speed)
{
    if (angle > 0 && angle <= 360)
    {
        right_dcmotor->SetMaxVelocity(speed);
        left_dcmotor->stop();

        // distance between wheels = 0.57m
        // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
        // calibration coeffient: y = 0.9487x
        int value = static_cast<int>((4096*3.333*14)*(0.57/0.325)*angle/360/0.9487*2);
        right_dcmotor->RotateRelativeDistancce(value);
    }
}

void MainWindow::TurnRightOneLeg(double angle, double speed)
{
    if (angle > 0 && angle <= 360)
    {
        right_dcmotor->stop();
        left_dcmotor->SetMaxVelocity(speed);

        // distance between wheels = 0.57m
        // [encoder:4096]  [motor Gearhead:14]  [wheel gear:3.333] [wheel diameter:0.325m]
        // calibration coeffient: y = 0.9487x
        int value = static_cast<int>((4096*3.333*14)*(0.57/0.325)*angle/360/0.9487*2);
        left_dcmotor->RotateRelativeDistancce((-1)*value);
    }
}

void MainWindow::on_pushButton_back_clicked()
{
    if (right_dcmotor->isOpen() && left_dcmotor->isOpen())
    {
        ui->textBrowser->append(QString("Go Backward %1m").arg(ui->doubleSpinBox_distance->value()));
        GoBackward(ui->doubleSpinBox_distance->value(),ui->doubleSpinBox_velocity->value());}
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(right_dcmotor->isOpen()).arg(left_dcmotor->isOpen()));
    }
}

void MainWindow::on_pushButton_turn_left_clicked()
{
    if (right_dcmotor->isOpen() && left_dcmotor->isOpen())
    {
        ui->textBrowser->append(QString("Turn Left %1 degrees").arg(ui->doubleSpinBox_angle->value()));
        TurnLeft(ui->doubleSpinBox_angle->value(),
                 ui->doubleSpinBox_velocity->value());
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(right_dcmotor->isOpen())
                                .arg(left_dcmotor->isOpen()));
    }
}
void MainWindow::on_pushButton_trun_right_clicked()
{
    if (right_dcmotor->isOpen() && left_dcmotor->isOpen())
    {
        ui->textBrowser->append(QString("Turn Right %1 degrees").arg(ui->doubleSpinBox_angle->value()));
        TurnRight(ui->doubleSpinBox_angle->value(),ui->doubleSpinBox_velocity->value());
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(right_dcmotor->isOpen())
                                .arg(left_dcmotor->isOpen()));
    }
}

void MainWindow::on_pushButton_turn_one_left_clicked()
{
    if (right_dcmotor->isOpen() && left_dcmotor->isOpen())
    {
        ui->textBrowser->append(QString("One Left %1 degrees").arg(ui->doubleSpinBox_angle->value()));
        TurnLeftOneLeg(ui->doubleSpinBox_angle->value(),ui->doubleSpinBox_velocity->value());
    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(right_dcmotor->isOpen())
                                .arg(left_dcmotor->isOpen()));
    }
}

void MainWindow::on_pushButton_trun_one_right_clicked()
{
    if (right_dcmotor->isOpen() && left_dcmotor->isOpen())
    {
        ui->textBrowser->append(QString("One Right %1 degrees").arg(ui->doubleSpinBox_angle->value()));
        TurnRightOneLeg(ui->doubleSpinBox_angle->value(),ui->doubleSpinBox_velocity->value());

    }
    else
    {
        ui->textBrowser->append(QString("[Error] Please Open the both dcmotor first right: %1, left: %2")
                                .arg(right_dcmotor->isOpen())
                                .arg(left_dcmotor->isOpen()));
    }
}

void MainWindow::on_pushButton_clearlog_clicked()
{
    ui->textBrowser->clear();
}

