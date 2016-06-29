#include "stereo_vision.h"

stereo_vision::stereo_vision() : TopView(20, 200, 3000, 19.8, 1080, 750, 270, 125, 100)
{
    device_index_L = -1;
    device_index_R = -1;
    fg_cam_L = false;
    fg_cam_R = false;
    fg_cam_opened = false;
    fg_calib_loaded = false;
    fg_calib = false;
    fg_stereoMatch = false;
    fg_reproject = false;
    fg_tracking = true;
    fg_topview_plot_points = false;
    fg_ground_filter = true;

    // blob
    obj_nums = 256;

    // ground filtering
    thresh_ground_cand = 10;
    thresh_ground_y_max = 50;
    thresh_rho_min = 15.0;
    thresh_dist = 1.0;
    angle_min = 165 * CV_PI / 180.0;
    angle_max = 178 * CV_PI / 180.0;
    weight_now = 0.2;
    weight_prev = 0.8;

    // thickness of frawing paint for objects on the image
    thick_obj_rect = 2;
    radius_obj_point = 3;

    ground_mean_guess = 0.0;

    cam_param = new camParam;
    param_sgbm = new matchParamSGBM;
    param_bm = new matchParamBM;

    objects = new objectInfo[obj_nums];
    objects_display = new objectInfo[obj_nums];

    data = new StereoData * [IMG_H];
    for (int r = 0; r < IMG_H; r++) {
        data[r] = new StereoData[IMG_W];
    }

    createLUT();

    // input source
    input_mode = SV::INPUT_SOURCE::CAM;

    // Initialization images for displaying
    img_L = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    img_R = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    disp_pseudo = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
#ifdef opencv_cuda
    bm = cv::cuda::createStereoBM(16, 9);
#else
    //bm = cv::createStereoBM(16, 9);
    bm = cv::StereoBM::create(16, 9);
    //sgbm = cv::createStereoSGBM(0, 16, 3);
    sgbm = cv::StereoSGBM::create(0, 16, 3);
#endif
    img_detected = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
    img_detected_display = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);

    matchParamInitialize(match_mode);

    time_gap = 50;
    t.start();

#ifdef debug_info_sv_time_proc
    std::cout<<"dataIn\tMatch\tDepth\tvDisp\tTopview\tBlob\tImage\tOMatch"<<std::endl;
#endif
}

stereo_vision::~stereo_vision()
{
    delete cam_param;
    delete param_sgbm;
    delete param_bm;

    for (int i = 0; i < IMG_H; i++) {
        delete[] data[i];
    }
    delete[] data;

    delete[] objects;
    delete[] objects_display;

    delete[] LUT_grid_row;
    delete[] LUT_grid_col;

    delete[] LUT_depth;
    delete[] LUT_img_col;

    close();
#ifndef opencv_cuda
    sgbm.release();
#endif
    bm.release();
}

void stereo_vision::createLUT()
{
    LUT_grid_row = new int[max_distance + 1];
    LUT_grid_col = new int[IMG_W];

    for (int m = 0; m < max_distance + 1; m++)
        LUT_grid_row[m] = 1.0 * log10(1.0 * m / (1.0 * min_distance)) / log10(1.0 + k);

    for (int m = 0; m < IMG_W; m++)
        LUT_grid_col[m] = 1.0 * m / c;

    LUT_depth = new int[img_row];
    LUT_img_col = new int[img_col];

    for (int m = 0; m < img_row; m++)
        LUT_depth[m] = 1.0 * min_distance * pow((double)(1.0 + k), m);

    for (int n = 0; n < img_col; n++)
        LUT_img_col[n] = 1.0 * n * c;
}

int stereo_vision::corrGridRow(int k)
{
    int m = k > max_distance ? max_distance : k;
    if (m <= 0)
        return -1;
    return LUT_grid_row[m];
}

int stereo_vision::corrGridCol(int k)
{
    int m = k > IMG_W ? IMG_W : k;
    return LUT_grid_col[m];
}

void stereo_vision::resetOpen(int device_index_L, int device_index_R)
{

    if (this->device_index_L != device_index_L) {
        cam_L.release();
        fg_cam_L = false;
    }
    if (this->device_index_R != device_index_R) {
        cam_R.release();
        fg_cam_R = false;
    }
}

bool stereo_vision::open(int device_index_L, int device_index_R)
{/*
    cv::VideoCapture cap(0);
    cv::Mat frame;
    for(;;){
        cap>>frame;
        imshow("frame",frame);
        cv::waitKey(33);

    }*/
//    qDebug() << cam_R.isOpened();
    resetOpen(device_index_L, device_index_R);
    if (fg_cam_L && fg_cam_R)
        return true;
    if (device_index_L == device_index_R || device_index_L < 0 || device_index_R < 0)
        return false;
    if (!cam_L.isOpened()) {
        if (cam_L.open(device_index_L)) {
            if (cam_L.isOpened()) {
                cam_L.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
                cam_L.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);
                fg_cam_L = true;
                this->device_index_L = device_index_L;
#ifdef debug_info_sv
                qDebug()<<"open L";
#endif
            }
        }
        else {
#ifdef debug_info_sv
            qDebug()<<"fail L";
#endif
        }
    }
    if (!cam_R.isOpened()) {
        if (cam_R.open(device_index_R)) {
            if (cam_R.isOpened()) {
                cam_R.set(cv::CAP_PROP_FRAME_WIDTH, IMG_W);
                cam_R.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_H);
                fg_cam_R = true;
                this->device_index_R = device_index_R;
#ifdef debug_info_sv
                qDebug()<<"open R";
#endif
            }
        }
        else {
#ifdef debug_info_sv
            qDebug()<<"fail R";
#endif
        }
    }
    if (fg_cam_L && fg_cam_R)
        fg_cam_opened = true;
    else
        fg_cam_opened = false;

    return fg_cam_opened;
}

void stereo_vision::close()
{
    if (cam_L.isOpened())
        cam_L.release();
    if (cam_R.isOpened())
        cam_R.release();
}

void stereo_vision::matchParamInitialize(int cur_mode)
{
    // default initialization
    int SAD_window_size = 0, number_disparity = 128;  // 0
    switch (cur_mode) {
    case SV::STEREO_MATCH::BM:
        //        bm->setROI1(roi1);
        //        bm->setROI2(roi2);
        bm->setPreFilterCap(31);
        bm->setBlockSize(SAD_window_size > 0 ? SAD_window_size : 9);
        bm->setMinDisparity(0);
        bm->setNumDisparities(number_disparity);
        bm->setTextureThreshold(10);
        bm->setUniquenessRatio(15);
        bm->setSpeckleWindowSize(100);
        bm->setSpeckleRange(32);
        bm->setDisp12MaxDiff(1);
        break;
#ifndef opencv_cuda
    case SV::STEREO_MATCH::SGBM:
        SAD_window_size = 0; // odd number, usually from 3 to 11
        number_disparity = number_disparity > 0 ? number_disparity : ((IMG_W / 8) + 15) & -16;

        sgbm->setPreFilterCap(63);
        int sgbm_win_size = SAD_window_size > 0 ? SAD_window_size : 5;
        sgbm->setBlockSize(sgbm_win_size);

        // channel
        cn = img_r_L.channels();

        sgbm->setP1(8 * cn * sgbm_win_size * sgbm_win_size);
        sgbm->setP2(32 * cn * sgbm_win_size * sgbm_win_size);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(number_disparity);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
        break;
#endif
    }

    match_mode = cur_mode;
}

void stereo_vision::modeChange(int cur_mode, bool fg_form_smp_update)
{
    match_mode = cur_mode;

    updateParamsSmp();

    if (fg_form_smp_update)
        updateFormParams();
}

void stereo_vision::updateParamsSmp()
{
    switch(match_mode) {
    case SV::STEREO_MATCH::BM:
        bm->setPreFilterSize(param_bm->pre_filter_size);
        bm->setPreFilterCap(param_bm->pre_filter_cap);
        bm->setBlockSize(param_bm->SAD_window_size);
        bm->setMinDisparity(param_bm->min_disp);
        bm->setNumDisparities(param_bm->num_of_disp);
        bm->setTextureThreshold(param_bm->texture_thresh);
        bm->setUniquenessRatio(param_bm->uniquenese_ratio);
        bm->setSpeckleWindowSize(param_bm->speckle_window_size);
        bm->setSpeckleRange(param_bm->speckle_range);
        break;
#ifndef opencv_cuda
    case SV::STEREO_MATCH::SGBM:
        sgbm->setPreFilterCap(param_sgbm->pre_filter_cap);
        sgbm->setBlockSize(param_sgbm->SAD_window_size);
        sgbm->setMinDisparity(param_sgbm->min_disp);
        sgbm->setNumDisparities(param_sgbm->num_of_disp);
        sgbm->setUniquenessRatio(param_sgbm->uniquenese_ratio);
        sgbm->setSpeckleWindowSize(param_sgbm->speckle_window_size);
        sgbm->setSpeckleRange(param_sgbm->speckle_range);
        break;
#endif
    }
}

void stereo_vision::updateFormParams()
{
    match_param.clear();
    switch(match_mode) {
    case SV::STEREO_MATCH::BM:
        match_param.push_back(bm->getPreFilterSize());
        match_param.push_back(bm->getPreFilterCap());
        match_param.push_back(bm->getBlockSize());
        match_param.push_back(bm->getMinDisparity());
        match_param.push_back(bm->getNumDisparities());
        match_param.push_back(bm->getTextureThreshold());
        match_param.push_back(bm->getUniquenessRatio());
        match_param.push_back(bm->getSpeckleWindowSize());
        match_param.push_back(bm->getSpeckleRange());
        break;
#ifndef opencv_cuda
    case SV::STEREO_MATCH::SGBM:
        match_param.push_back(sgbm->getPreFilterCap());
        match_param.push_back(sgbm->getBlockSize());
        match_param.push_back(sgbm->getMinDisparity());
        match_param.push_back(sgbm->getNumDisparities());
        match_param.push_back(sgbm->getUniquenessRatio());
        match_param.push_back(sgbm->getSpeckleWindowSize());
        match_param.push_back(sgbm->getSpeckleRange());
        break;
#endif
    }

    emit updateForm(match_mode, match_param);
}

bool stereo_vision::camCapture()
{
    if (cam_L.isOpened()) {
        cam_L >> cap_L;
        cv::cvtColor(cap_L, img_L, cv::COLOR_BGR2RGB);
    }
    else
        return false;

    if (cam_R.isOpened()) {
        cam_R >> cap_R;
        cv::cvtColor(cap_R, img_R, cv::COLOR_BGR2RGB);
    }
    else
        return false;

    return true;
}

bool stereo_vision::loadRemapFile(int cam_focal_length, double base_line)
{
    // The folder of calibration files should be placed under project's folder
    // check whether the file has been loaded
    if (fg_calib_loaded && this->cam_param->cam_focal_length == cam_focal_length && this->cam_param->base_line == base_line)
        return fg_calib_loaded;

    // find files under which folder and find the folder with calibration files
    remap_folder = "calibrationImgs";
    remap_file = QString("My_Data_" + QString::number(cam_focal_length) + "_" + QString::number(base_line) + ".yml");
    remap_path = project_path;
    if (!remap_path.exists(remap_folder))
        return fg_calib_loaded;
    remap_path.cd(remap_folder);

#ifdef debug_info_sv
    qDebug()<<"path exist: "<<remap_path.exists()<<"path: "<<remap_path.path();
#endif
    if (!remap_path.exists())
        return fg_calib_loaded;

    cv::FileStorage fs(QString(remap_path.path() + "/" + remap_file).toStdString().c_str(), cv::FileStorage::READ);

    if (!fs.isOpened())
        return fg_calib_loaded;

    // rewrite params
    fs["reMapLx"] >> rmapLx;
    fs["reMapLy"] >> rmapLy;
    fs["reMapRx"] >> rmapRx;
    fs["reMapRy"] >> rmapRy;
    fs["ROI0x"] >> calibROI[0].x;
    fs["ROI0y"] >> calibROI[0].y;
    fs["ROI0w"] >> calibROI[0].width;
    fs["ROI0h"] >> calibROI[0].height;
    fs["ROI1x"] >> calibROI[1].x;
    fs["ROI1y"] >> calibROI[1].y;
    fs["ROI1w"] >> calibROI[1].width;
    fs["ROI1h"] >> calibROI[1].height;
    fs.release();
    this->cam_param->cam_focal_length = cam_focal_length;
    this->cam_param->base_line = base_line;
    fg_calib_loaded = true;

    return fg_calib_loaded;
}

bool stereo_vision::rectifyImage()
{
    if (fg_calib_loaded) {
        cv::remap(img_L, img_r_L, rmapLx, rmapLy, cv::INTER_LINEAR);
        cv::remap(img_R, img_r_R, rmapRx, rmapRy, cv::INTER_LINEAR);
        return true;
    }

    return false;
}

void stereo_vision::stereoMatch()
{
    // pre-processing
    cv::cvtColor(img_r_L, img_match_L, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_r_R, img_match_R, cv::COLOR_BGR2GRAY);

    cv::equalizeHist(img_match_L, img_match_L);
    cv::equalizeHist(img_match_R, img_match_R);

    cv::GaussianBlur(img_match_L, img_match_L, cv::Size(7, 7), 0, 0);
    cv::GaussianBlur(img_match_R, img_match_R, cv::Size(7, 7), 0, 0);
#ifdef opencv_cuda
    d_L.upload(img_match_L);
    d_R.upload(img_match_R);
    bm->compute(d_L, d_R, d_disp);
    d_disp.download(disp_raw);
#else
    if (match_mode == SV::STEREO_MATCH::BM)
        bm->compute(img_match_L, img_match_R, disp_raw);
    else if (match_mode == SV::STEREO_MATCH::SGBM)
        sgbm->compute(img_match_L, img_match_R, disp_raw);
#endif

    disp_raw.convertTo(disp, CV_8UC1);

    // depth calculation of points from disp [merge into stereo_vision::depthCalculation]
}


void stereo_vision::depthCalculation()
{
    if (fg_pseudo)
        disp_pseudo.setTo(0);
    uchar* ptr_color = color_table->scanLine(0);

    lock_sv_data.lockForWrite();
    for (int r = 0; r < IMG_H; r++) {
#ifdef opencv_cuda
        uchar* ptr_raw = (uchar*)(disp_raw.data + r * disp_raw.step);
#else
        short int* ptr_raw = (short int*)(disp_raw.data + r * disp_raw.step);
#endif

        uchar* ptr = (uchar*)(disp_pseudo.data + r * disp_pseudo.step);
        for (int c = 0; c < IMG_W; c++) {
            // non-overlapping part
            if (c < param_sgbm->num_of_disp / 2 && input_mode == SV::STEREO_MATCH::SGBM)
                continue;
            else if (c < param_bm->num_of_disp / 2 && input_mode == SV::STEREO_MATCH::BM)
                continue;
            // Depth calculation
            float val = ptr_raw[c];
#ifdef opencv_cuda
            data[r][c].disp = val;
#else
            data[r][c].disp = val / 16.0;
#endif

            if (data[r][c].disp > 0) {
                data[r][c].Z = cam_param->param_r / data[r][c].disp;
                data[r][c].X = (c - IMG_W_HALF) * data[r][c].Z / cam_param->focal_length;
                data[r][c].Y = (IMG_H_HALF - r) * data[r][c].Z / cam_param->focal_length + cam_param->rig_height;

                // pseudo color transform
                if (fg_pseudo) {
                    int z_est;
                    z_est = data[r][c].Z;
                    //                        std::cout<<z_est<<" ";
                    if (z_est >= min_distance && z_est <= max_distance) {
                        int jj = z_est - min_distance;
                        ptr[3 * c + 0] = ptr_color[3 * jj + 0];
                        ptr[3 * c + 1] = ptr_color[3 * jj + 1];
                        ptr[3 * c + 2] = ptr_color[3 * jj + 2];
                    }
                    // out of max_distance
                    else if (z_est > max_distance) {
                        ptr[3 * c + 0] = 140;
                        ptr[3 * c + 1] = 0;
                        ptr[3 * c + 2] = 168;
                    }
                    // below the min_distance
                    else {
                        ptr[3 * c + 0] = 0;
                        ptr[3 * c + 1] = 0;
                        ptr[3 * c + 2] = 0;
                    }
                }
            }
            // unmatched
            else {
                data[r][c].Z = -1;
                data[r][c].X = -1;
                data[r][c].Y = -1;
                ptr[3 * c + 0] = 100;
                ptr[3 * c + 1] = 100;
                ptr[3 * c + 2] = 100;
                //                    std::cout<<"0 ";
            }
        }
        //            std::cout<<std::endl;
    }
    lock_sv_data.unlock();
}

bool stereo_vision::vDispCalculation()
{
    int img_w;
    switch (match_mode) {
    case SV::STEREO_MATCH::BM:
        img_w = param_bm->num_of_disp;
        break;
    case SV::STEREO_MATCH::SGBM:
        img_w = param_sgbm->num_of_disp;
        break;
    }

    v_disp.release();
    v_disp_norm.release();
    v_disp_display.release();
    v_disp_bi.release();
    // v-disparity generation
    v_disp = cv::Mat::zeros(IMG_H, img_w, CV_16SC1);
    for (int r = 0; r < IMG_H; r++) {
        short int* ptr_v = v_disp.ptr<short int>(r);
        for (int c = 0 ; c < IMG_W; c++) {
            short int val_raw = data[r][c].disp;
            short int val = 1.0 * val_raw + 0.5;
            if (val == -1) continue;
            ptr_v[val]++;
        }
    }

    v_disp_display = cv::Mat::zeros(v_disp.rows, v_disp.cols, CV_8UC3);
    cv::normalize(v_disp, v_disp_norm, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
    for (int r = 0; r < v_disp_norm.rows; r++) {
        short int *ptr = v_disp_norm.ptr<short int>(r);
        uchar *ptr_o = v_disp_display.ptr<uchar>(r);
        for (int c = 0; c < v_disp_norm.cols; c++) {
            ptr_o[3* c + 0] = ptr[c];
            ptr_o[3* c + 1] = ptr[c];
            ptr_o[3* c + 2] = ptr[c];
        }
    }
#ifdef debug_info_sv_ground_filter_v_disp
    cv::imshow("V-disp", v_disp_display);
#endif

    v_disp_bi = cv::Mat::zeros(v_disp.rows, v_disp.cols, CV_8UC1);
    for (int r = 0; r < v_disp.rows; r++) {
        short int *ptr = v_disp.ptr<short int>(r);
        uchar *ptr_o = v_disp_bi.ptr<uchar>(r);
        for (int c = 0; c < v_disp.cols; c++) {
            if (ptr[c] > 50)
                ptr_o[c] = 255;
        }
    }
#ifdef debug_info_sv_ground_filter_v_disp
    cv::imshow("Canny", v_disp_bi);
    qDebug()<<"rho theta";
#endif

    lines.clear();
    cv::HoughLines(v_disp_bi, lines, 1, CV_PI / 180, 75, 0, 0);
    if (lines.empty()) return false;
    lines_fitted_pt.clear();
    lines_fitted_pt.resize(lines.size(), 0);

    for (int i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
#ifdef debug_info_sv_ground_filter_v_disp
        qDebug()<<rho<<theta;
#endif
        if (rho >= thresh_rho_min && theta >= angle_min && theta <= angle_max) {
            for (int c = 0; c < v_disp.cols; c++) {
                int disp_y = c * (-1 * cos(theta) / sin (theta)) + rho / sin(theta);
                if (c >= v_disp.cols || c < 0 || disp_y >= v_disp.rows || disp_y < 0)
                    continue;
                if (v_disp_bi.ptr<uchar>(disp_y)[c] == 255)
                    lines_fitted_pt[i]++;
            }
        }
    }

    int fitted_line_id = -1;
    int amount_fitted_pt = 0;
    for (int k = 0 ; k < lines.size(); k++) {
        if (amount_fitted_pt < lines_fitted_pt[k]) {
            amount_fitted_pt = lines_fitted_pt[k];
            fitted_line_id = k;
        }
    }

    if (fitted_line_id == -1) return false;
    fitted_line = lines[fitted_line_id];

#ifdef debug_info_sv_ground_filter_v_disp
    for (int i = 0 ; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        pt1.x = 0;
        pt1.y = pt1.x * (-1 * cos(theta) / sin (theta)) + rho / sin(theta);
        pt2.x = v_disp.cols;
        pt2.y = pt2.x * (-1 * cos(theta) / sin (theta)) + rho / sin(theta);
        cv::line(v_disp_display, pt1, pt2, cv::Scalar(0, 0, 255), 1, 8, 0);
    }
    float rho = fitted_line[0], theta = fitted_line[1];
    cv::Point pt1, pt2;
    pt1.x = 0;
    pt1.y = pt1.x * (-1 * cos(theta) / sin (theta)) + rho / sin(theta);
    pt2.x = v_disp.cols;
    pt2.y = pt2.x * (-1 * cos(theta) / sin (theta)) + rho / sin(theta);
    cv::line(v_disp_display, pt1, pt2, cv::Scalar(0, 255, 255), 1, 8, 0);

    cv::imshow("Lines", v_disp_display);
    cv::Mat img = img_r_L.clone();
#endif

    float ground_avg_y = 0.0;
    int avg_disp_count = 0;
    if (fitted_line_id != -1) {
        for (int r = 0; r < IMG_H; r++) {
            for (int c = 0 ; c < IMG_W; c++) {
                float rho = fitted_line[0];
                float theta = fitted_line[1];
                if (data[r][c].disp == -1 || data[r][c].Y > thresh_ground_y_max) {
                    data[r][c].ground_cand = false;
                    continue;
                }
                cv::Point coor_v_disp = cv::Point(data[r][c].disp, r);
                cv::Point line_1, line_2;
                line_1.x = 0;
                line_1.y = line_1.x * (-1 * cos(theta) / sin (theta)) + rho / sin(theta);
                line_2.x = v_disp.cols;
                line_2.y = line_2.x * (-1 * cos(theta) / sin (theta)) + rho / sin(theta);
                float dist = point2Line(coor_v_disp, line_1, line_2);
                if (dist < thresh_dist) {
                    avg_disp_count += 1.0;
                    ground_avg_y += 1.0 * (data[r][c].Y - ground_avg_y) / avg_disp_count;
                    data[r][c].ground_cand = true;
#ifdef debug_info_sv_ground_filter_v_disp
                    cv::circle(img, cv::Point(c, r), 1, cv::Scalar(0, 0, 255), 1, 8, 0);
#endif
                }
            }
        }
    }
#ifdef debug_info_sv_ground_filter_v_disp
    qDebug()<<"ground_mean_guess: "<<ground_mean_guess<<"\tavg Y: "<<ground_avg_y;
    cv::imshow("det", img);
#endif

    if (avg_disp_count == 0.0)
        return false;
    ground_mean_guess = ground_avg_y * weight_now + ground_mean_guess * weight_prev;
    return true;
}

float stereo_vision::point2Line(cv::Point pt, cv::Point line_1, cv::Point line_2)
{
    float a, b, c, dis;
    a = line_2.y - line_1.y;
    b = line_1.x - line_2.x;
    c = line_2.x * line_1.y - line_1.x * line_2.y;
    dis = abs(a * pt.x + b * pt.y + c) / sqrt(a * a + b * b);

    return dis;
}

void stereo_vision::HoughLine()
{
    std::vector<cv::Vec2f> lines;
    QString file = QFileDialog::getOpenFileName();
    cv::Mat img = cv::imread(file.toStdString());
    cv::Mat img_bi;
    cv::Canny(img, img_bi, 50, 255, 3);
    cv::imshow("canny", img_bi);
    cv::HoughLines(img_bi, lines, 1, CV_PI/180, 100, 0, 0);

    for (int i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        cv::line(img, pt1, pt2, cv::Scalar(0,0,255), 1, cv::LINE_AA);
    }

    cv::imshow("Lines", img);
}

bool stereo_vision::dataIn()
{
    switch (input_mode) {
    // camera capturing
    case SV::INPUT_SOURCE::CAM:
        if (!camCapture())
            return false;

        if (re.vr->fg_record) {
            cv::Mat img_merge = cv::Mat(IMG_H, 2 * IMG_W, CV_8UC3);
            re.vr->combineTwoImages(&img_merge, img_L, img_R, cv::Size(img_L.cols, img_L.rows));
            re.recordData(img_merge);
        }
        break;
    case SV::INPUT_SOURCE::VIDEO:
        // For synchronization replay
        if (re.tr->fg_loaded && re.vr->fg_loaded)
            if (!re.tr->fg_data_end && re.tr->current_frame_count < re.vr->current_frame_count) {
                //                std::cout<<re.tr->fg_data_end<<" "<<re.tr->current_frame_count<<" "<<re.vr->current_frame_count;
                return false;
            }
        if (!re.vr->segmentTwoImages(&img_L, &img_R, cv::Size(IMG_W, IMG_H))) {
            emit videoEnd();
            return false;
        }
        break;
    case SV::INPUT_SOURCE::IMG:
        //        img_L = ;
        //        img_R = ;
        break;
    }

    re.vr->current_frame_count++;

    return true;
}

int stereo_vision::dataExec()
{
#ifdef debug_info_sv
    qDebug()<<"run";
#endif
#ifdef debug_info_sv_time_proc
    QTime ttt; ttt.restart();
#endif

    if (!dataIn()) {
        return SV::STATUS::NO_INPUT;
    }

    // camera calibration
    if (fg_calib) {
        if (!rectifyImage())
            return SV::STATUS::NO_RECTIFYIMAGE;
    }
    else {
        img_r_L = img_L.clone();
        img_r_R = img_R.clone();
    }
#ifdef debug_info_sv_time_proc
    std::cout<<ttt.restart()<<"\t";
#endif

    detected_obj = 0;

    // stereo matching
    if (fg_stereoMatch) {
        stereoMatch();
#ifdef debug_info_sv_time_proc
        std::cout<<ttt.restart()<<"\t";
#endif
        depthCalculation();
#ifdef debug_info_sv_time_proc
        std::cout<<ttt.restart()<<"\t";
#endif

        // ground filtering
        if (fg_ground_filter)
            fg_vDisp = vDispCalculation();
        else
            fg_vDisp = false;
#ifdef debug_info_sv_time_proc
        std::cout<<ttt.restart()<<"\t";
#endif

        if (fg_topview) {
            // object detection
            pointProjectTopView();
#ifdef debug_info_sv_time_proc
            std::cout<<ttt.restart()<<"\t";
#endif
            blob(3000);
#ifdef debug_info_sv_time_proc
            std::cout<<ttt.restart()<<"\t";
#endif

            if (fg_reproject) {
                pointProjectImage();
#ifdef debug_info_sv_time_proc
                std::cout<<ttt.restart()<<"\t";
#endif

                //                if (fg_recognition)
                //                    objectRecognition();

#ifdef debug_info_sv_time_proc
                std::cout<<ttt.restart()<<"\t";
#endif
            }
        }
    }
    else {
        disp.setTo(0);
        disp_pseudo.setTo(0);
        img_detected.setTo(0);
    }
#ifdef debug_info_sv_time_proc
    std::cout<<std::endl;
#endif
#ifdef debug_info_sv
    qDebug()<<"run"<<&img_L<<"emit"<<&img_r_L;
#endif

    updateDataForDisplay();

    return SV::STATUS::OK;
}

int stereo_vision::guiUpdate()
{
    if (t.elapsed() > time_gap) {
        emit updateGUI(&img_r_L, &img_r_R, &disp, &disp_pseudo, &topview, &img_detected, detected_obj, re.vr->current_frame_count);
        t.restart();
        time_proc = t_p.restart();
        return SV::STATUS::OK;
    }
    return SV::STATUS::NO_UPDATE;
}

void stereo_vision::getMouseCursorInfo(int y, int x, float &disp, cv::Point3i &pos3D)
{
    lock_sv_mouse.lockForRead();
    disp = data[y][x].disp;
    pos3D = cv::Point3i(data[y][x].X, data[y][x].Y, data[y][x].Z);
    lock_sv_mouse.unlock();
}

void stereo_vision::pointProjectTopView()
{
    resetTopView();
    int grid_row, grid_col;
    for (int r = 0; r < IMG_H; r++) {
        for (int c = 0; c < IMG_W; c++) {
            // reset
            data[r][c].grid_id = std::pair<int, int>(-1, -1);

            // porject each 3D point onto a topview
            if (data[r][c].Z >= min_distance && data[r][c].Z <= max_distance &&
                    data[r][c].Y <= 300) {
                // ground filtering
                if (fg_ground_filter) {
                    if (fg_vDisp) {
                        if (data[r][c].ground_cand)
                            continue;
                    }
                    else {
                        if (abs(data[r][c].Y - ground_mean_guess) < 10)
                            continue;
                    }
                }

                grid_row = corrGridRow(data[r][c].Z);
                grid_col = corrGridCol(c);
                //                grid_row = 1.0 * log10(1.0 * data[r][c].Z / min_distance) / log10(1.0 + k);
                ////                grid_col = 360.0 * img_col_half * atan((c / (double)(IMG_W / img_col) - img_col_half) / data[r][c].Z) / (view_angle * CV_PI) + img_col_half;
                ////                grid_col = 1.0 * c * ratio_col; //**// old
                //                grid_col = 1.0 * c / this->c;

                // display each point on topview
                if (fg_topview_plot_points)
                    if (grid_row >= 0 && grid_row < img_row + 1 &&
                            grid_col >= 0 && grid_col < img_col + 1)
                        cv::circle(topview, pointT(img_grid[grid_row][grid_col]), 1, cv::Scalar(0, 0, 255, 255), -1, 8, 0);

                // mark each point belongs to which cell
                int grid_row_t = img_row - grid_row - 1;
                int grid_col_t = grid_col;
                if (grid_row_t >= 0 && grid_row_t < img_row &&
                        grid_col_t >= 0 && grid_col_t < img_col) {
                    lock_sv_data.lockForWrite();
                    // count the amount of point
                    grid_map[grid_row_t][grid_col_t].pts_num++;
                    // average the depth
                    grid_map[grid_row_t][grid_col_t].avg_Z += 1.0 * (data[r][c].Z - grid_map[grid_row_t][grid_col_t].avg_Z) / grid_map[grid_row_t][grid_col_t].pts_num;
                    grid_map[grid_row_t][grid_col_t].avg_X += 1.0 * (data[r][c].X - grid_map[grid_row_t][grid_col_t].avg_X) / grid_map[grid_row_t][grid_col_t].pts_num;
                    grid_map[grid_row_t][grid_col_t].avg_Y += 1.0 * (data[r][c].Y - grid_map[grid_row_t][grid_col_t].avg_Y) / grid_map[grid_row_t][grid_col_t].pts_num;

                    // label the point to the belonging cell
                    data[r][c].grid_id = std::pair<int, int>(grid_row_t, grid_col_t);
                    lock_sv_data.unlock();
                }
            }
        }
    }

    // check whether the cell is satisfied as an object [merge into stereo_vision::blob]
}

void stereo_vision::resetBlob()
{
    for (int i = 0; i < obj_nums; i++)
        resetMatchedInfo(objects[i]);
}

void stereo_vision::blob(int thresh_pts_num)
{
    // objects starts from 0
    int mask_size = 3;
    int offset = (mask_size - 1) / 2;
    int cur_label = 0;
    resetBlob();
    for (int r = 0; r < img_row; r++) {
        for (int c = 0; c < img_col; c++) {
            // blob labeling
            if (grid_map[r][c].pts_num >= thresh_free_space && grid_map[r][c].obj_label == -1) {
                if (cur_label == obj_nums) {
                    qDebug()<<"Objects are out of defined.";
                    break;
                }

                int count = 1;

                std::stack<std::pair<int, int> > neighbors;
                neighbors.push(std::pair<int, int>(r, c));
                lock_sv_object.lockForWrite();
                grid_map[r][c].obj_label = cur_label;
                objects[cur_label].pts_num += grid_map[r][c].pts_num;
                objects[cur_label].avg_Z += 1.0 * (grid_map[r][c].avg_Z - objects[cur_label].avg_Z) / count;
                objects[cur_label].avg_X += 1.0 * (grid_map[r][c].avg_X - objects[cur_label].avg_X) / count;
                objects[cur_label].avg_Y += 1.0 * (grid_map[r][c].avg_Y - objects[cur_label].avg_Y) / count;

                lock_sv_object.unlock();

                while (!neighbors.empty()) {
                    std::pair<int, int> cur_pos = neighbors.top();
                    neighbors.pop();

                    int r_now, c_now;
                    for (int rr = - offset; rr <= offset; rr++) {
                        r_now = cur_pos.first + rr;
                        for (int cc = - offset; cc <= offset; cc++) {
                            c_now = cur_pos.second + cc;

                            // out of boundary
                            if (r_now < 0 || r_now >= img_row ||
                                    c_now < 0 || c_now >= img_col) {
                                continue;
                            }
                            if (grid_map[r_now][c_now].pts_num >= thresh_free_space &&
                                    grid_map[r_now][c_now].obj_label == -1) {
                                neighbors.push(std::pair<int, int>(r_now, c_now));
                                lock_sv_object.lockForWrite();
                                grid_map[r_now][c_now].obj_label = cur_label;
                                objects[cur_label].pts_num += grid_map[r_now][c_now].pts_num;
                                objects[cur_label].avg_Z += 1.0 * (grid_map[r][c].avg_Z - objects[cur_label].avg_Z) / count;
                                objects[cur_label].avg_X += 1.0 * (grid_map[r][c].avg_X - objects[cur_label].avg_X) / count;
                                objects[cur_label].avg_Y += 1.0 * (grid_map[r][c].avg_Y - objects[cur_label].avg_Y) / count;
                                lock_sv_object.unlock();
                            }
                        }
                    }

                }
                cur_label++;
                count++;
            }
        }
        if (cur_label == obj_nums)
            break;
    }

    for (int i= 0; i < obj_nums; i++) {
        lock_sv_object.lockForRead();
        if (objects[i].pts_num >= thresh_pts_num) {
            objects[i].labeled = true;
            detected_obj++;
        }
        lock_sv_object.unlock();
    }

#ifdef debug_info_sv_blob_data
    std::cout<<"objs: "<<cur_label - 1<<std::endl;

    for (int i = 0; i < obj_nums; i++) {
        std::cout<<i<<" "<<objects[i].pts_num<<std::endl;
    }

    // blob data of grid map
    for (int r = 0; r < img_row; r++) {
        for (int c = 0; c < img_col; c++) {
            std::cout << grid_map[r][c].obj_label << " ";
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
#endif

    // filter out trivial objects by amount of points & check whether the cell is satisfied as an object
    cv::Point pts[4];
    uchar* ptr_color = color_table->scanLine(0);
    int p;
    int gap = 0;
    for (int r = 0; r < img_row; r++) {
        for (int c = 0; c < img_col; c++) {
            // filtering
            int grid_obj_label = grid_map[r][c].obj_label;
            if (grid_obj_label >= 0 && objects[grid_obj_label].labeled) {
                // plot points onto topview
                int row = img_row - r - gap > 0 ? img_row - r - gap : 0;
                int row_1 = img_row - (r + 1) + gap <= img_row ? img_row - (r + 1) + gap : img_row;
                int col = c - gap > 0 ? c - gap : 0;
                int col_1 = c + 1 + gap <= img_col ? c + 1 + gap : img_col;

                pts[0] = pointT(img_grid[row][col]);
                pts[1] = pointT(img_grid[row_1][col]);
                pts[2] = pointT(img_grid[row_1][col_1]);
                pts[3] = pointT(img_grid[row][col_1]);

                p = (max_distance - 0.5 * (img_grid[row][col].y + img_grid[row_1][col].y)) - min_distance;
                objects[grid_obj_label].color = cv::Scalar(ptr_color[3 * p + 0], ptr_color[3 * p + 1], ptr_color[3 * p + 2], 255);
                cv::fillConvexPoly(topview, pts, thick_polygon, objects[grid_obj_label].color, 8, 0);

                // find rect of object in topview
                if (objects[grid_obj_label].rect_tl.x > c) objects[grid_obj_label].rect_tl.x = c;
                if (objects[grid_obj_label].rect_tl.y > r) objects[grid_obj_label].rect_tl.y = r;
                if (objects[grid_obj_label].rect_br.x < c) objects[grid_obj_label].rect_br.x = c;
                if (objects[grid_obj_label].rect_br.y < r) objects[grid_obj_label].rect_br.y = r;
            }
        }
    }

    for (int k = 0 ; k < obj_nums; k++) {
        int tl_depth = LUT_depth[img_row - objects[k].rect_tl.y - 1];
        int br_depth = LUT_depth[img_row - objects[k].rect_br.y - 1];
        int tl_x = (LUT_img_col[objects[k].rect_tl.x] - IMG_W_HALF) * tl_depth / cam_param->focal_length;
        int br_x = (LUT_img_col[objects[k].rect_br.x] - IMG_W_HALF) * tl_depth / cam_param->focal_length;;
        cv::Point rect_tl = cv::Point(tl_x, tl_depth);
        cv::Point rect_br = cv::Point(br_x, br_depth);
        objects[k].rect = cv::Rect(rect_tl.x, rect_tl.y, rect_br.x - rect_tl.x, abs(rect_br.y - rect_tl.y));
    }

    // find the boundary of objects [merge into stereo_vision::pointProjectImage]
}

void stereo_vision::pointProjectImage()
{
    // remap a blob objects from topview to label
    lock_sv.lockForWrite();
    img_detected.setTo(0);
    lock_sv.unlock();
    for (int r = 0; r < IMG_H; r++) {
        uchar *ptr_o = img_r_L.ptr<uchar>(r);
        uchar *ptr_d = img_detected.ptr<uchar>(r);
        for (int c = 0; c < IMG_W; c++) {
            int grid_r, grid_c;
            lock_sv_data.lockForRead();
            grid_r = data[r][c].grid_id.first;
            grid_c = data[r][c].grid_id.second;
            lock_sv_data.unlock();
            if (grid_r == -1 || grid_c == -1 || data[r][c].disp <= 0)
                continue;
            int label = grid_map[grid_r][grid_c].obj_label;
            if (grid_map[grid_r][grid_c].obj_label >= 0 && objects[label].labeled && data[r][c].disp > 0) {
                // find the boundary of objects
                if (objects[label].br == std::pair<int, int>(-1, -1) &&
                        objects[label].tl == std::pair<int, int>(-1, -1)) {
                    objects[label].br = std::pair<int, int>(r, c);
                    objects[label].tl = std::pair<int, int>(r, c);
                }
                else {
                    lock_sv_object.lockForWrite();
                    if (objects[label].br.first < r)
                        objects[label].br.first = r;
                    if (objects[label].br.second < c)
                        objects[label].br.second = c;
                    if (objects[label].tl.first > r)
                        objects[label].tl.first = r;
                    if (objects[label].tl.second > c)
                        objects[label].tl.second = c;
                    lock_sv_object.unlock();
                }

                // draw points
                ptr_d[3 * c + 0] = ptr_o[3 * c + 0];
                ptr_d[3 * c + 1] = ptr_o[3 * c + 1];
                ptr_d[3 * c + 2] = ptr_o[3 * c + 2];
            }
        }
    }

    // sort objects using depth for displaying color
    for (int i = 0; i < obj_nums; i++) {
        for (int j = i + 1; j < obj_nums; j++) {
            if (objects[i].avg_Z >= objects[j].avg_Z) {
                obj_temp = objects[j];
                objects[j] = objects[i];
                objects[i] = obj_temp;
            }
        }
    }

    // draw region of detected objects
    int range_precision = 3;

    lock_f_sv.lockForRead();
    cv::Mat img_r_L_tmp = img_r_L.clone();
    lock_f_sv.unlock();
    for (int i = 0; i < obj_nums; i++) {
        lock_sv_object.lockForWrite();
        if (objects[i].labeled && objects[i].br != std::pair<int, int>(-1, -1) && objects[i].tl != std::pair<int, int>(-1, -1)) {
            // find center of rect
            objects[i].center = std::pair<int, int>(0.5 * (objects[i].tl.first + objects[i].br.first), 0.5 * (objects[i].tl.second + objects[i].br.second));
            objects[i].pc.angle = atan((objects[i].avg_X) / objects[i].avg_Z) * 180.0 / CV_PI;
            objects[i].pc.range = sqrt(pow((double)(objects[i].avg_Z), 2) + pow((double)(objects[i].avg_X), 2));
            cv::Rect rect_tmp = cv::Rect(objects[i].tl.second, objects[i].tl.first, (objects[i].br.second - objects[i].tl.second), (objects[i].br.first - objects[i].tl.first));
            objects[i].img = img_r_L_tmp(rect_tmp);

            cv::rectangle(img_detected, rect_tmp, objects[i].color, thick_obj_rect, 8, 0);
            cv::circle(img_detected, cv::Point(objects[i].center.second, objects[i].center.first), radius_obj_point, cv::Scalar(0, 255, 0), -1, 8, 0);
            std::string distance_tag = QString::number(objects[i].avg_Z / 100.0, 'g', range_precision).toStdString() + " M";
            cv::putText(img_detected, distance_tag, cv::Point(objects[i].tl.second, objects[i].br.first - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, cv::Scalar(0, 0, 255), 2);
        }
        lock_sv_object.unlock();
    }
}

void stereo_vision::resetMatchedInfo(objectInfo &src)
{
    src.pts_num = 0;
    src.labeled = false;
    src.avg_X = 0.0;
    src.avg_Y = 0.0;
    src.avg_Z = 0.0;
    src.rect_tl = cv::Point(img_col, img_row);
    src.rect_br = cv::Point(0, 0);
    src.tl = std::pair<int, int>(-1, -1);
    src.br = std::pair<int, int>(-1, -1);
    src.center = std::pair<int, int>(-1, -1);
    src.img.release();
}

void stereo_vision::connectMatchedInfo(objectInfo &src, objectInfo &dst)
{
    dst.labeled        = src.labeled;
    dst.avg_X          = src.avg_X;
    dst.avg_Y          = src.avg_Y;
    dst.avg_Z          = src.avg_Z;
    dst.rect_tl.x      = src.rect_tl.x;
    dst.rect_tl.y      = src.rect_tl.y;
    dst.rect_br.x      = src.rect_br.x;
    dst.rect_br.y      = src.rect_br.y;
    dst.tl.first       = src.tl.first;
    dst.tl.second      = src.tl.second;
    dst.br.first       = src.br.first;
    dst.br.second      = src.br.second;
    dst.center.first   = src.center.first;
    dst.center.second  = src.center.second;
    dst.color          = cv::Scalar(src.color);
    dst.img.release();
    dst.img            = src.img.clone();
    dst.pc.range       = src.pc.range;
    dst.pc.angle       = src.pc.angle;
    dst.rect           = cv::Rect(src.rect.tl().x, src.rect.tl().y, src.rect.width, src.rect.height);
    dst.rect_f         = cv::Rect(src.rect_f.tl().x, src.rect_f.tl().y, src.rect_f.width, src.rect_f.height);
    dst.plot_pt_f.x    = src.plot_pt_f.x;
    dst.plot_pt_f.y    = src.plot_pt_f.y;
    dst.rect_world     = cv::Rect(src.rect_world.tl().x, src.rect_world.tl().y, src.rect_world.width, src.rect_world.height);
    dst.pc_world.range = src.pc_world.range;
    dst.pc_world.angle = src.pc_world.angle;

    resetMatchedInfo(*&src);
}

void stereo_vision::updateDataForDisplay()
{
    lock_sv_object.lockForWrite();
    for (int i = 0; i < obj_nums; i++)
        connectMatchedInfo(objects[i], objects_display[i]);
    lock_sv_object.unlock();
}

void stereo_vision::change_bm_pre_filter_size(int value)
{
    bm->setPreFilterSize(value);
    param_bm->pre_filter_size = value;
#ifdef debug_info_sv_param
    qDebug()<<bm->getPreFilterSize();
#endif
}

void stereo_vision::change_bm_pre_filter_cap(int value)
{
    bm->setPreFilterCap(value);
    param_bm->pre_filter_cap = value;
#ifdef debug_info_sv_param
    qDebug()<<bm->getPreFilterCap();
#endif
}

void stereo_vision::change_bm_sad_window_size(int value)
{
    bm->setBlockSize(value);
    param_bm->SAD_window_size = value;
#ifdef debug_info_sv_param
    qDebug()<<bm->getBlockSize();
#endif
}

void stereo_vision::change_bm_min_disp(int value)
{
    bm->setMinDisparity(value);
    param_bm->min_disp = value;
#ifdef debug_info_sv_param
    qDebug()<<bm->getMinDisparity();
#endif
}

void stereo_vision::change_bm_num_of_disp(int value)
{
    bm->setNumDisparities(value);
    param_bm->num_of_disp = value;
#ifdef debug_info_sv_param
    qDebug()<<bm->getNumDisparities();
#endif
}

void stereo_vision::change_bm_texture_thresh(int value)
{
    bm->setTextureThreshold(value);
    param_bm->texture_thresh = value;
#ifdef debug_info_sv_param
    qDebug()<<bm->getTextureThreshold();
#endif
}

void stereo_vision::change_bm_uniqueness_ratio(int value)
{
    bm->setUniquenessRatio(value);
    param_bm->uniquenese_ratio = value;
#ifdef debug_info_sv_param
    qDebug()<<bm->getUniquenessRatio();
#endif
}

void stereo_vision::change_bm_speckle_window_size(int value)
{
    bm->setSpeckleWindowSize(value);
    param_bm->speckle_window_size = value;
#ifdef debug_info_sv_param
    qDebug()<<bm->getSpeckleWindowSize();
#endif
}

void stereo_vision::change_bm_speckle_range(int value)
{
    bm->setSpeckleRange(value);
    param_bm->speckle_range = value;
#ifdef debug_info_sv_param
    qDebug()<<bm->getSpeckleRange();
#endif
}

void stereo_vision::change_sgbm_pre_filter_cap(int value)
{
#ifndef opencv_cuda
    sgbm->setPreFilterCap(value);
    param_sgbm->pre_filter_cap = value;
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getPreFilterCap();
#endif
#endif
}

void stereo_vision::change_sgbm_sad_window_size(int value)
{
#ifndef opencv_cuda
    sgbm->setBlockSize(value);
    sgbm->setP1(8 * cn * value * value);
    sgbm->setP2(32 * cn * value * value);
    param_sgbm->SAD_window_size = value;
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getBlockSize();
#endif
#endif
}

void stereo_vision::change_sgbm_min_disp(int value)
{
#ifndef opencv_cuda
    sgbm->setMinDisparity(value);
    param_sgbm->min_disp = value;
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getMinDisparity();
#endif
#endif
}

void stereo_vision::change_sgbm_num_of_disp(int value)
{
#ifndef opencv_cuda
    sgbm->setNumDisparities(value);
    param_sgbm->num_of_disp = value;
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getNumDisparities();
#endif
#endif
}

void stereo_vision::change_sgbm_uniqueness_ratio(int value)
{
#ifndef opencv_cuda
    sgbm->setUniquenessRatio(value);
    param_sgbm->uniquenese_ratio = value;
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getUniquenessRatio();
#endif
#endif
}

void stereo_vision::change_sgbm_speckle_window_size(int value)
{
#ifndef opencv_cuda
    sgbm->setSpeckleWindowSize(value);
    param_sgbm->speckle_window_size = value;
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getSpeckleWindowSize();
#endif
#endif
}

void stereo_vision::change_sgbm_speckle_range(int value)
{
#ifndef opencv_cuda
    sgbm->setSpeckleRange(value);
    param_sgbm->speckle_range = value;
#ifdef debug_info_sv_param
    qDebug()<<sgbm->getSpeckleRange();
#endif
#endif
}
