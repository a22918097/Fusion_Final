#include "sensorinfo.h"

SensorInfo::SensorInfo()
{
    //**// You may CRASH if you add lots of sensors but you don't revise the number of the array
    // sensor information
    sensors = new sensorInformation[3];

    // Initialization
    sv = new stereo_vision();
    rc = new RadarController();
    lrf = new lrf_controller();
    size_data_fused = sv->objSize() + rc->objSize();
    data_fused = new ObjectTracking::objectTrackingInfo[size_data_fused];
    data_fused_tmp = new ObjectTracking::objectTrackingInfo[size_data_fused];
#ifdef debug_info_object_matching_fusion_img
    data_fused_prev = new ObjectTracking::objectTrackingInfo[size_data_fused];
#endif

    fg_fusion = false;

    fg_ca_astar = false;

    ot_fused = new ObjectTracking();
    // max. size of detected objects
    //    ot_fused->initialze(sv->objSize() + rc->objSize());

    // object macthing
    initializeObjectMatching(size_data_fused);
    setErrorThresholdX(100);
    setErrorThresholdZ(500, sv->max_distance);
    setThresholdBha(0.3);

    ca = new CollisionAvoidance();

    pic_sv = QPixmap(20, 20);
    pic_radar = QPixmap(20, 20);

    range_precision = 4;

    t.restart();
    time_gap = 30;

    // fusion params
    var_sv = pow(0.2260, 2);
    var_radar = pow(0.0204, 2);
    var_fused = var_sv * var_radar / (var_sv + var_radar);

#ifdef debug_info_ca_astar
    cv::namedWindow("A* result", cv::WINDOW_AUTOSIZE);
#endif
#ifdef debug_info_object_tracking
    cv::namedWindow("Trajectory", cv::WINDOW_AUTOSIZE);
#endif
#ifdef debug_info_object_matching_fusion_img
    cv::namedWindow("fusion matched comparision", cv::WINDOW_AUTOSIZE);
#endif
}

SensorInfo::~SensorInfo()
{
    delete sv;
    delete rc;
    delete lrf;
    delete[] data_fused;
    delete[] data_fused_tmp;
#ifdef debug_info_object_matching_fusion_img
    delete[] data_fused_prev;
#endif
    delete ot_fused;
    delete ca;
    delete[] sensors;
}

void SensorInfo::initialFusedTopView(int range_pixel)
{
    // vehicle & fused topview information
    detection_range_pixel = range_pixel;

    detection_range = sv->max_distance;

    ratio = 1.0 * detection_range_pixel / sv->max_distance;

    vehicle.width_pixel = vehicle.width * ratio;
    vehicle.length_pixel = vehicle.length * ratio;

    vehicle.head_pos = vehicle.length / 2;
    vehicle.head_pos_pixel = vehicle.head_pos * ratio;

    // find max & min detection range in all sensors, so this function shall be called after initialization of all sensors
    max_detection_range = rc->max_distance;

    min_detection_range = rc->min_distance + sqrt(pow(0.5 * vehicle.length, 2) + pow(0.5 * vehicle.width, 2));

    fused_topview.create(2 * detection_range_pixel, 2 * detection_range_pixel, CV_8UC4);
    fused_topview_BG.create(2 * detection_range_pixel, 2 * detection_range_pixel, CV_8UC4);

    thickness = 2;
    font = cv::FONT_HERSHEY_PLAIN;
    font_size = 1;
    font_thickness = 1;

    // sv
    int sensor = SENSOR::SV;
    sensors[sensor].color = cv::Scalar(255, 0, 0, 255);

    sensors[sensor].angle_half_fov = sv->view_angle * 0.5;

    QColor pic_color_sv = QColor(sensors[sensor].color[0], sensors[sensor].color[1], sensors[sensor].color[2]);
    pic_sv.fill(pic_color_sv);

    // radar
    sensor = SENSOR::RADAR;
    sensors[sensor].color = cv::Scalar(0, 0, 255, 255);

    sensors[sensor].angle_half_fov = rc->view_angle * 0.5;

    QColor pic_color_radar = QColor(sensors[sensor].color[0], sensors[sensor].color[1], sensors[sensor].color[2]);
    pic_radar.fill(pic_color_radar);
}

void SensorInfo::updateFusedTopView()
{
    ratio = 1.0 * detection_range_pixel / detection_range;

    vehicle.VCP = cv::Point(detection_range_pixel, detection_range_pixel);

    vehicle.width_pixel = vehicle.width * ratio;

    vehicle.length_pixel = vehicle.length * ratio;

    vehicle.head_pos_pixel = vehicle.head_pos * ratio;

    vehicle.rect = cv::Rect(vehicle.VCP.x - (int)(vehicle.width_pixel / 2.0), vehicle.VCP.y - (int)(vehicle.length_pixel / 2.0),
                            vehicle.width_pixel, vehicle.length_pixel);

    vehicle.color = cv::Scalar(0, 255, 0, 255);

    fused_topview_BG.setTo(cv::Scalar(0, 0, 0, 0));

    // topview
    cv::circle(fused_topview_BG, vehicle.VCP, detection_range_pixel, rc->color_BG, -1, 8, 0);

    // max detection range [now]
    cv::putText(fused_topview_BG, QString::number((int)detection_range).toStdString() + " cm", cv::Point(vehicle.VCP.x + 0.65 * detection_range_pixel, vehicle.VCP.y  - 0.8 * detection_range_pixel), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, rc->color_line, 1);

    // sensor information
    int device = SENSOR::SV;
    sensors[device].pos_pixel = cv::Point(sensors[device].location.pos.x * ratio, sensors[device].location.pos.y * ratio);
    device = SENSOR::RADAR;
    sensors[device].pos_pixel = cv::Point(sensors[device].location.pos.x * ratio, sensors[device].location.pos.y * ratio);

    // FOV - Stereo vision
    cv::Point sensor_pos;
    cv::Point pos_fov_r, pos_fov_l;
    int shift_x, shift_y;
    device = SENSOR::SV;
    sensor_pos = cv::Point(vehicle.VCP.x + sensors[device].pos_pixel.x, vehicle.VCP.y - sensors[device].pos_pixel.y);
    shift_x = detection_range_pixel * sin(sensors[device].angle_half_fov * CV_PI / 180.0);
    shift_y = detection_range_pixel * cos(sensors[device].angle_half_fov * CV_PI / 180.0);
    pos_fov_r = cv::Point(sensor_pos.x + shift_x, sensor_pos.y - shift_y);
    pos_fov_l = cv::Point(sensor_pos.x - shift_x, sensor_pos.y - shift_y);
    cv::line(fused_topview_BG, sensor_pos, pos_fov_r, sensors[device].color_fov, 1, 8, 0);
    cv::line(fused_topview_BG, sensor_pos, pos_fov_l, sensors[device].color_fov, 1, 8, 0);

    // FOV - ESR
    device = SENSOR::RADAR;
    sensor_pos = cv::Point(vehicle.VCP.x + sensors[device].pos_pixel.x, vehicle.VCP.y - sensors[device].pos_pixel.y);
    shift_x = detection_range_pixel * sin(sensors[device].angle_half_fov * CV_PI / 180.0);
    shift_y = detection_range_pixel * cos(sensors[device].angle_half_fov * CV_PI / 180.0);
    pos_fov_r = cv::Point(sensor_pos.x + shift_x, sensor_pos.y - shift_y);
    pos_fov_l = cv::Point(sensor_pos.x - shift_x, sensor_pos.y - shift_y);
    cv::line(fused_topview_BG, sensor_pos, pos_fov_r, sensors[device].color_fov, 1, 8, 0);
    cv::line(fused_topview_BG, sensor_pos, pos_fov_l, sensors[device].color_fov, 1, 8, 0);

    // 30 m
    float distance = (sv->max_distance + vehicle.length / 2) * ratio;
    if (distance <= detection_range_pixel)
        cv::circle(fused_topview_BG, vehicle.VCP, distance, rc->color_line, 1, 8, 0);

    // 5 m
    distance = (500 + vehicle.length / 2)* ratio;
    if (distance <= detection_range_pixel)
        cv::circle(fused_topview_BG, vehicle.VCP, distance, rc->color_line, 1, 8, 0);

    // vehicle
    cv::rectangle(fused_topview_BG, vehicle.rect, vehicle.color, -1, 8, 0);
}

void SensorInfo::updateFusedData()
{
    lock_ot_fused.lockForWrite();
    for (int i = 0; i < ot_fused->ti.size(); i++) {
        if (ot_fused->ti[i].track_status == TRACK_STATUS::NO_TARGET) continue;
        for (int j = 0; j < ot_fused->ti[i].info.size(); j++) {
            ot_fused->ti[i].info[j].plot_pt_f = point2FusedTopView(ot_fused->ti[i].info[j].pc);
            //**// rect won't change as wellr? 20150623
        }
    }
    lock_ot_fused.unlock();
}

void SensorInfo::chooseVehicle(int vehicle_type)
{
    switch (vehicle_type) {
    case VEHICLE::CART:
        // choose cart
        vehicle.width = 43;//cm
        vehicle.length = 43;
        vehicle.head_pos = vehicle.length / 2;

        sensors[SENSOR::SV].location.pos = cv::Point2f(0, 29.5);
        sensors[SENSOR::SV].location.theta = 0.0;
        sensors[SENSOR::RADAR].location.pos = cv::Point2f(0, vehicle.head_pos);
        sensors[SENSOR::RADAR].location.theta = 0.0;
        break;
    case VEHICLE::CAR:
        // choose car: TOYOTA Camry
        vehicle.width = 183;
        vehicle.length = 485;
        vehicle.head_pos = vehicle.length / 2;

        sensors[SENSOR::SV].location.pos = cv::Point2f(0, 75);
        sensors[SENSOR::SV].location.theta = 0.0;
        sensors[SENSOR::RADAR].location.pos = cv::Point2f(0, vehicle.head_pos);
        sensors[SENSOR::RADAR].location.theta = 0.0;
        break;
    case VEHICLE::TRACTOR:
        vehicle.width = 125;
        vehicle.length = 295;
        vehicle.head_pos = vehicle.length / 2;

        sensors[SENSOR::SV].location.pos = cv::Point2f(-16.5, 302 - vehicle.length / 2);
        sensors[SENSOR::SV].location.theta = 0.0;
        sensors[SENSOR::RADAR].location.pos = cv::Point2f(0, 301 - vehicle.length / 2);
        sensors[SENSOR::RADAR].location.theta = 0.0;
        break;


    }

    vehicle.width_pixel = vehicle.width * ratio;
    vehicle.length_pixel = vehicle.length * ratio;
    // find max & min detection range in all sensors, so this function shall be called after initialization of all sensors
    max_detection_range = rc->max_distance;
    min_detection_range = rc->min_distance + sqrt(pow(0.5 * vehicle.length, 2) + pow(0.5 * vehicle.width, 2));

    updateFusedTopView();
}

void SensorInfo::zoomOutFusedTopView()
{
    // zoom out (scrolling forward)
    if (detection_range == min_detection_range) {
        int count = min_detection_range / gap;
        if (count == 0)
            count = 1;
        detection_range = ceil(count) * gap;
    }
    else
        detection_range = detection_range + gap < max_detection_range ?
                    detection_range + gap : max_detection_range;
}

void SensorInfo::zoomInFusedTopView()
{
    // zoom in (scrolling backward)
    if (detection_range == max_detection_range) {
        int count = max_detection_range / gap;
        detection_range = floor(count) * gap;
    }
    else
        detection_range = detection_range - gap > min_detection_range ?
                    detection_range - gap : min_detection_range;
}

void SensorInfo::resetFusion()
{
    for (int p = 0; p < size_data_fused; p++)
        resetMatchedInfo(data_fused[p]);
}

void SensorInfo::resetTrackingInfo()
{
    lock_ot_fused.lockForWrite();
    ot_fused->ti.clear();
    lock_ot_fused.unlock();
}

void SensorInfo::dataExec()
{
    if (fg_proc && fg_data_update) {
        resetFusion();

        dataProcess(fg_sv, fg_radar);

        if (fg_ot) {
            dataMatching();

            dataTracking();
        }

        dataCollisionAvoidance();
    }

    drawFusedTopView(fg_sv, fg_radar, fg_sv_each_pixel, fg_ot_trajectory, fg_ot_trajectory_raw, fg_ot_trajectory_kalman, fg_ot_kf);
    fg_proc = true;
}

void SensorInfo::resetMatchedInfo(ObjectTracking::objectTrackingInfo &src)
{
    src.det_mode = DETECT_MODE::NO_DETECT;
    src.ids.now = -1;
    src.img.release();
    src.H_hist.release();
    src.pc = SensorBase::PC(0.0, 0.0);
    src.pos = cv::Point2f(-1.0, -1.0);
}

void SensorInfo::connectMatchedInfo(ObjectTracking::objectTrackingInfo &src, ObjectTracking::objectTrackingInfo &dst)
{
    dst.det_mode       = src.det_mode;
    dst.ids.now      = src.ids.now;
    dst.img.release();
    dst.H_hist.release();
    dst.img            = src.img.clone();
    dst.H_hist         = src.H_hist.clone();
    dst.rect_f         = cv::Rect(src.rect_f.tl().x, src.rect_f.tl().y, src.rect_f.width, src.rect_f.height);
    dst.plot_pt_f.x    = src.plot_pt_f.x;
    dst.plot_pt_f.y    = src.plot_pt_f.y;
    dst.pc.range       = src.pc.range;
    dst.pc.angle       = src.pc.angle;
    dst.pos.x          = src.pos.x;
    dst.pos.y          = src.pos.y;
    dst.ids.prev       = src.ids.prev;

    resetMatchedInfo(*&src);
}

void SensorInfo::dataMatching()
{
    // Object matching: a) Bha. dist. of H color space, b) bias of X & Z of WCS location
    // Comparison of H color space image using Bhattacharyya distance with Bubble search
    resetObjMatching();
    // Extract histogram of H color space
    lock_data_fused.lockForRead();
    for (int i = 0; i < size_data_fused; i++) {
        if (data_fused[i].det_mode != DETECT_MODE::NO_DETECT) {
            fg_om_existed = true;
            om[i].labeled = true;
            om_obj_num++;
            om[i].pc = data_fused[i].pc;
            switch (data_fused[i].det_mode) {
            case DETECT_MODE::RADAR_ONLY:
                om[i].match_type = MATCH_TYPE::RANGE_ONLY;
                om[i].center = std::pair<int, int>(0, 0);
                break;
            case DETECT_MODE::SV_ONLY:
            case DETECT_MODE::SV_RADAR:
                om[i].match_type = MATCH_TYPE::RANGE_BHA;
                om[i].center = data_fused[i].center;
                cv::Mat img_hsv = cv::Mat(data_fused[i].img.rows, data_fused[i].img.cols, CV_8UC3);
                om[i].img = data_fused[i].img.clone();
                cv::cvtColor(data_fused[i].img, img_hsv, cv::COLOR_BGR2HSV);
                splitOneOut(0, img_hsv, &om[i].H_img);
                cv::calcHist(&om[i].H_img, 1, 0, cv::Mat(), om[i].H_hist, 1, &hist_size, &hist_ranges, true, false);
                cv::normalize(om[i].H_hist, om[i].H_hist, hist_ranges[0], hist_ranges[1], cv::NORM_MINMAX, -1, cv::Mat());
                data_fused[i].H_hist = om[i].H_hist.clone();
                break;
            }
        }
    }
    lock_data_fused.unlock();

#ifdef debug_info_object_matching_img
    comp = sv->img_detected.clone();
#endif

    matching_result.clear();
    matching_result = Matching();

    for (int k = 0; k < matching_result.size(); k++) {
        int id = matching_result[k].id;
        data_fused[id].ids.prev = matching_result[k].prev_id;
#ifdef debug_info_object_matching_fusion_others
        qDebug()<<"Matched now"<<id<<"prev"<<matching_result[k].prev_id;
#endif
    }
#ifdef debug_info_object_matching_fusion_img
    cv::Mat fused_topview_cp = fused_topview.clone();
    for (int p = 0; p < size_data_fused; p++) {
        if (data_fused[p].det_mode != DETECT_MODE::NO_DETECT && data_fused_prev[p].det_mode != DETECT_MODE::NO_DETECT) {
            if (data_fused[p].ids.prev != -1) {
                cv::circle(fused_topview_cp, data_fused_prev[p].plot_pt_f, 3, cv::Scalar(0, 0, 255, 255), -1, 8, 0);
                cv::line(fused_topview_cp, data_fused[p].plot_pt_f, data_fused_prev[data_fused[p].ids.prev].plot_pt_f, cv::Scalar(255, 255, 255, 255), 2, 8, 0);
            }
            else
                cv::circle(fused_topview_cp, data_fused[p].plot_pt_f, 2, cv::Scalar(0, 255, 255, 255), -1, 8, 0);
        }
    }
    cv::imshow("fusion matched comparision", fused_topview_cp);

    for (int p = 0; p < size_data_fused; p++) {
        data_fused_prev[p].det_mode      = data_fused[p].det_mode;
        data_fused_prev[p].ids.now     = data_fused[p].ids.now;
        data_fused_prev[p].img.release();
        data_fused_prev[p].img           = data_fused[p].img.clone();
        data_fused_prev[p].rect_f        = cv::Rect(data_fused[p].rect_f.tl().x, data_fused[p].rect_f.tl().y, data_fused[p].rect_f.width, data_fused[p].rect_f.height);
        data_fused_prev[p].plot_pt_f.x   = data_fused[p].plot_pt_f.x;
        data_fused_prev[p].plot_pt_f.y   = data_fused[p].plot_pt_f.y;
        data_fused_prev[p].pc.range      = data_fused[p].pc.range;
        data_fused_prev[p].pc.angle      = data_fused[p].pc.angle;
        data_fused_prev[p].pos.x         = data_fused[p].pos.x;
        data_fused_prev[p].pos.y         = data_fused[p].pos.y;
    }
#endif
}

void SensorInfo::dataTracking()
{
    lock_ot_fused.lockForWrite();
    for (int m = 0; m < ot_fused->ti.size(); m++)
        ot_fused->ti[m].fg_update = false;
    lock_ot_fused.unlock();
    for (int p = 0 ; p < size_data_fused; p++) {
        lock_data_fused.lockForRead();
        if (data_fused[p].det_mode == DETECT_MODE::NO_DETECT) continue;
        bool fg_new_obj = true;
        int conti_id = -1;
        lock_data_fused.unlock();
        lock_ot_fused.lockForWrite();
#ifdef debug_info_object_matching_fusion_others
        qDebug()<<"================"<<p<<data_fused[p].ids.prev;
#endif
        // new or non-continuous obstacle
        if (data_fused[p].ids.prev == -1) {
            // new: go ahead & non-continous obstacle: compare again with missed_count is grater than 0.
            for (int m = 0; m < ot_fused->ti.size(); m++) {
                if (ot_fused->ti[m].track_status == TRACK_STATUS::NO_TARGET || ot_fused->ti[m].fg_update)
                    continue;
                int last = ot_fused->ti[m].info.size() - 1;
                if (last >= 0) {
                    std::pair<double, double> pc, pc_prev;
                    pc_prev.first = ot_fused->ti[m].info[last].pc.range * sin(ot_fused->ti[m].info[last].pc.angle * CV_PI / 180.0);
                    pc_prev.second = ot_fused->ti[m].info[last].pc.range * cos(ot_fused->ti[m].info[last].pc.angle * CV_PI / 180.0);
                    lock_data_fused.lockForRead();
                    pc.first = data_fused[p].pc.range * sin(data_fused[p].pc.angle * CV_PI / 180.0);
                    pc.second = data_fused[p].pc.range * cos(data_fused[p].pc.angle * CV_PI / 180.0);
                    double Bha;
                    bool fg_Bha_check = false;
                    // check if H_hist existed
                    if (!ot_fused->ti[m].info[last].H_hist.empty() && !data_fused[p].H_hist.empty()) {
                        fg_Bha_check = true;
                        Bha = cv::compareHist(ot_fused->ti[m].info[last].H_hist, data_fused[p].H_hist, cv::HISTCMP_BHATTACHARYYA);
                    }
                    lock_data_fused.unlock();
#ifdef debug_info_object_matching_fusion_others
                    qDebug()<<"dist"<<fabs((pc.first - pc_prev.first))<<fabs((pc.second - pc_prev.second));
#endif
                    if (fabs((pc.first - pc_prev.first)) <= max_err_x &&
                            fabs((pc.second - pc_prev.second)) <= max_err_z * pc.second / max_distance_z) {
                        if (fg_Bha_check) {
                            if (Bha <= thresh_Bha) {
#ifdef debug_info_object_matching_fusion_others
                                qDebug()<<"new / non-conti";
#endif
                                fg_new_obj = false;
                                conti_id = m;
                                break;
                            }
                        }
                        else {
                            fg_new_obj = false;
#ifdef debug_info_object_matching_fusion_others
                            qDebug()<<"new / non-conti";
#endif
                            conti_id = m;
                            break;
                        }
                    }
                }
            }
        }
        // matched obstacle
        else {
            for (int m = 0; m < ot_fused->ti.size(); m++) {
                if (ot_fused->ti[m].track_status == TRACK_STATUS::NO_TARGET || ot_fused->ti[m].fg_update)
                    continue;
                int cur_obj = ot_fused->ti[m].info.size() - 1;
                lock_data_fused.lockForRead();
                if (cur_obj >= 0 && ot_fused->ti[m].info[cur_obj].ids.now == data_fused[p].ids.prev) {
                    //**// radar data will randomly labeled @@ 20150623
                    std::pair<double, double> pc, pc_prev;
                    pc_prev.first = ot_fused->ti[m].info[cur_obj].pc.range * sin(ot_fused->ti[m].info[cur_obj].pc.angle * CV_PI / 180.0);
                    pc_prev.second = ot_fused->ti[m].info[cur_obj].pc.range * cos(ot_fused->ti[m].info[cur_obj].pc.angle * CV_PI / 180.0);
                    lock_data_fused.lockForRead();
                    pc.first = data_fused[p].pc.range * sin(data_fused[p].pc.angle * CV_PI / 180.0);
                    pc.second = data_fused[p].pc.range * cos(data_fused[p].pc.angle * CV_PI / 180.0);
                    if (fabs((pc.first - pc_prev.first)) > max_err_x ||
                            fabs((pc.second - pc_prev.second)) > max_err_z * pc.second / max_distance_z)
                        continue;
                    fg_new_obj = false;
#ifdef debug_info_object_matching_fusion_others
                    qDebug()<<"matched";
#endif
                    conti_id = m;
                    break;
                }
                lock_data_fused.unlock();
            }
        }
        lock_ot_fused.unlock();

        lock_data_fused.lockForRead();
        ObjectTracking::objectTrackingInfo info_new;
        info_new.ids.now = data_fused[p].ids.now;
        info_new.det_mode = data_fused[p].det_mode;
        info_new.img = data_fused[p].img.clone();
        info_new.H_hist = data_fused[p].H_hist.clone();
        info_new.rect_f = data_fused[p].rect_f;
        info_new.plot_pt_f = data_fused[p].plot_pt_f;
        info_new.pc = data_fused[p].pc;
        info_new.pos = data_fused[p].pos;
        lock_data_fused.unlock();
        // update object
        if (!fg_new_obj) {
#ifdef debug_info_object_matching_fusion_others
            qDebug()<<"update"<<p<<ot_fused->ti[conti_id].info[ot_fused->ti[conti_id].info.size() - 1].ids.now<<data_fused[p].ids.now;
#endif
            lock_ot_fused.lockForWrite();
            ot_fused->ti[conti_id].info.push_back(info_new);
            ot_fused->ti[conti_id].track_status = TRACK_STATUS::UPDATE_TARGET;
            ot_fused->ti[conti_id].fg_update = true;
            ot_fused->ti[conti_id].trajectory.push_back(SensorBase::PC(info_new.pc.range, info_new.pc.angle));
#ifdef debug_info_object_tracking_trajectory
            if (conti_id == 0 && !fg_ot_kf)
                qDebug()<<"Raw traj."<<re.vr->current_frame_count<<SensorBase::polar2Cart(info_new.pc).x<<SensorBase::polar2Cart(info_new.pc).y;
#endif

            // raw data's velocity and acceleration estimation
            int last = ot_fused->ti[conti_id].info.size() - 1;
            if (last >= 2) {
                // velocity & acceleration estimation
                // (cm), (m/s), (m/s^2)
                cv::Point2f pt_pre2t = ot_fused->ti[conti_id].info[last - 2].pos;
                cv::Point2f pt_pre1t = ot_fused->ti[conti_id].info[last - 1].pos;
                cv::Point2f pt_now = ot_fused->ti[conti_id].info[last].pos;
                //                qDebug()<<"size"<<last<<pt_pre2t.x<<pt_pre2t.y<<pt_pre1t.x<<pt_pre1t.y;
                //                for(int i = 0; i < last + 1; i++) {
                //                    qDebug()<<ot_fused->ti[conti_id].info[i].pos.x<<
                //                              ot_fused->ti[conti_id].info[i].pos.y;
                //                }
                float time_proc = 1.0 * sv->time_proc;
                cv::Point2f vel_prev2t_1t = SensorBase::velEstimation(pt_pre1t, pt_pre2t, time_proc, VEL_ESTI::CM_PER_FRAME2M_PER_SEC);
                cv::Point2f vel_prev1t_now = SensorBase::velEstimation(pt_now, pt_pre1t, time_proc, VEL_ESTI::CM_PER_FRAME2M_PER_SEC);
                cv::Point2f acc = cv::Point2f((vel_prev1t_now.x - vel_prev2t_1t.x) / time_proc, (vel_prev1t_now.y - vel_prev2t_1t.y) / time_proc);
                ot_fused->ti[conti_id].info[last].vel.x = vel_prev1t_now.x;
                ot_fused->ti[conti_id].info[last].vel.y = vel_prev1t_now.y;
                ot_fused->ti[conti_id].info[last].acc.x = acc.x;
                ot_fused->ti[conti_id].info[last].acc.y = acc.y;
#ifdef debug_info_object_tracking
                std::cout<<conti_id<<std::endl;
                std::cout<<"pt_pre1t: "<<pt_pre1t.x<<","<<pt_pre1t.y<<"\tpt_pre2t: "<<
                           pt_pre2t.x<<","<<pt_pre2t.y<<"\t\tvel_prev2t_1t: "<<
                           vel_prev2t_1t.x<<","<<vel_prev2t_1t.y<<"\tvel_prev1t_now: "<<
                           vel_prev1t_now.x<<","<<vel_prev1t_now.y<<std::endl;
#endif


            }
            // Kalman filter and its velocity and acceleration estimation
            if (fg_ot_kf) {
                if (last >= 0) {
                    ot_fused->ti[conti_id].kf.prediction = ot_fused->ti[conti_id].kf.kf_core.predict();
                    ot_fused->ti[conti_id].kf.predictPt = cv::Point((int)(ot_fused->ti[conti_id].kf.prediction.at<float>(0)), (int)(ot_fused->ti[conti_id].kf.prediction.at<float>(1)));
#ifdef debug_info_object_tracking
                    std::cout<<"predictPt: "<<ot_fused->ti[conti_id].kf.predictPt.x<<","<<ot_fused->ti[conti_id].kf.predictPt.y<<std::endl;
#endif

                    ot_fused->ti[conti_id].kf.measurement.at<float>(0) = ot_fused->ti[conti_id].info[last].pos.x;
                    ot_fused->ti[conti_id].kf.measurement.at<float>(1) = ot_fused->ti[conti_id].info[last].pos.y;
                    ot_fused->ti[conti_id].kf.measurement += ot_fused->ti[conti_id].kf.kf_core.measurementMatrix * ot_fused->ti[conti_id].kf.state;
                    ot_fused->ti[conti_id].kf.estimated = ot_fused->ti[conti_id].kf.kf_core.correct(ot_fused->ti[conti_id].kf.measurement);
                    int x = ot_fused->ti[conti_id].kf.estimated.at<float>(0);
                    int y = ot_fused->ti[conti_id].kf.estimated.at<float>(1);
                    ot_fused->ti[conti_id].kf.statePt = cv::Point((int)(ot_fused->ti[conti_id].kf.estimated.at<float>(0)), (int)(ot_fused->ti[conti_id].kf.estimated.at<float>(1)));
                    //**// sometimes the value will overflow 20150717
                    ot_fused->ti[conti_id].trajectory_kf.push_back(ot_fused->ti[conti_id].kf.statePt);
#ifdef debug_info_object_tracking_trajectory
                    if (conti_id == 0 && fg_ot_kf)
                        qDebug()<<"Kalman traj."<<re.vr->current_frame_count<<ot_fused->ti[conti_id].kf.statePt.x<<ot_fused->ti[conti_id].kf.statePt.y;
#endif
                    int last_kf = ot_fused->ti[conti_id].trajectory_kf.size() - 1;
                    // velocity & acceleration estimation
                    if (last_kf >= 2) {
                        // (cm), (m/s), (m/s^2)
                        cv::Point2f pt_pre2t_kf = cv::Point2f(ot_fused->ti[conti_id].trajectory_kf[last_kf - 2]);
                        cv::Point2f pt_pre1t_kf = cv::Point2f(ot_fused->ti[conti_id].trajectory_kf[last_kf - 1]);
                        cv::Point2f pt_now_kf = cv::Point2f(ot_fused->ti[conti_id].trajectory_kf[last_kf]);
                        float time_proc = 1.0 * sv->time_proc;
                        cv::Point2f vel_prev2t_1t = SensorBase::velEstimation(pt_pre1t_kf, pt_pre2t_kf, time_proc, VEL_ESTI::CM_PER_FRAME2M_PER_SEC);
                        cv::Point2f vel_prev1t_now = SensorBase::velEstimation(pt_now_kf, pt_pre1t_kf, time_proc, VEL_ESTI::CM_PER_FRAME2M_PER_SEC);
                        cv::Point2f acc = cv::Point2f((vel_prev1t_now.x - vel_prev2t_1t.x) / time_proc, (vel_prev1t_now.y - vel_prev2t_1t.y) / time_proc);
                        ot_fused->ti[conti_id].info[last].vel.x = vel_prev1t_now.x;
                        ot_fused->ti[conti_id].info[last].vel.y = vel_prev1t_now.y;
                        ot_fused->ti[conti_id].info[last].acc.x = acc.x;
                        ot_fused->ti[conti_id].info[last].acc.y = acc.y;
                    }
                    randn(ot_fused->ti[conti_id].kf.processNoise, cv::Scalar(0), cv::Scalar::all(sqrt(ot_fused->ti[conti_id].kf.kf_core.processNoiseCov.at<float>(0, 0))));
                    ot_fused->ti[conti_id].kf.state = ot_fused->ti[conti_id].kf.kf_core.transitionMatrix * ot_fused->ti[conti_id].kf.state + ot_fused->ti[conti_id].kf.processNoise;
                }
            }

#ifdef debug_info_fusion
            int id_need = -1;
            for (int k = 0; k < ot_fused->ti[conti_id].info.size(); k++) {
                if (ot_fused->ti[conti_id].info[k].id.y != -1) {
                    id_need = k;
                    break;
                }
            }
            if (id_need != -1 && ot_fused->ti[conti_id].info[id_need].id.y != -1 &&
                    ot_fused->ti[conti_id].info[ot_fused->ti[conti_id].info.size() - 1].id.y != -1) {
                cv::Mat img_1 = ot_fused->ti[conti_id].info[id_need].img;
                cv::Mat img_2 = ot_fused->ti[conti_id].info[ot_fused->ti[conti_id].info.size() - 1].img;

                cv::namedWindow("first", cv::WINDOW_AUTOSIZE);
                cv::namedWindow("now", cv::WINDOW_AUTOSIZE);
                cv::imshow("first", img_1);
                cv::imshow("now", img_2);
                char c;
                cv::waitKey(c);
                cv::destroyWindow("first");
                cv::destroyWindow("now");
            }
            qDebug()<<"update"<<info_new.id.x<<info_new.id.y<<info_new.id.z;
#endif
            lock_ot_fused.unlock();
        }
        // new object
        else {
            ObjectTracking::objectTracking ti_new;
            ti_new.info.push_back(info_new);
#ifdef debug_info_object_matching_fusion_others
            qDebug()<<"push"<<p<<info_new.ids.now<<ot_fused->ti.size();
#endif
            ti_new.track_status = TRACK_STATUS::NEW_TARGET;
            ti_new.fg_update = true;
            int r = rng.uniform(0, 256);
            int g = rng.uniform(0, 256);
            int b = rng.uniform(0, 256);
            ti_new.color_trajectory = cv::Scalar(b, g, r, 255);
            ti_new.trajectory.clear();
            ti_new.trajectory.push_back(SensorBase::PC(info_new.pc.range, info_new.pc.angle));
            ti_new.trajectory_kf.clear();
            ti_new.kf.kf_core.statePre.at<float>(0) = info_new.pos.x;
            ti_new.kf.kf_core.statePre.at<float>(1) = info_new.pos.y;
            ti_new.kf.kf_core.statePre.at<float>(2) = 0.0;
            ti_new.kf.kf_core.statePre.at<float>(3) = 0.0;
            ti_new.kf.kf_core.transitionMatrix = (cv::Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
            cv::setIdentity(ti_new.kf.kf_core.measurementMatrix);
            cv::setIdentity(ti_new.kf.kf_core.processNoiseCov, cv::Scalar::all(1e-5));
            cv::setIdentity(ti_new.kf.kf_core.measurementNoiseCov, cv::Scalar::all(1e-3));
            cv::setIdentity(ti_new.kf.kf_core.errorCovPost, cv::Scalar::all(1));
            ot_fused->ti.push_back(ti_new);
#ifdef debug_info_fusion
            qDebug()<<"new"<<info_new.id.x<<info_new.id.y<<info_new.id.z;
#endif
        }
    }

    // predict non-update obstacle
    //    for (int m = 0; m < ot_fused->ti.size(); m++) {
    //        if (ot_fused->ti[m].track_status != TRACK_STATUS::NO_TARGET && !ot_fused->ti[m].fg_update) {
    //            qDebug()<<"predict"<<m;
    //            lock_data_fused.lockForRead();
    //            int last = ot_fused->ti[m].info.size() - 1;
    //            if (last < 0 || (ot_fused->ti[m].info[last].vel.x == 0.0 && ot_fused->ti[m].info[last].vel.y == 0.0))
    //                continue;
    //            ObjectTracking::objectTrackingInfo info_new;
    //            info_new.ids.now   = ot_fused->ti[m].info[last].ids.now;
    //            info_new.det_mode  = ot_fused->ti[m].info[last].det_mode;
    //            info_new.img       = ot_fused->ti[m].info[last].img.clone();
    //            info_new.H_hist    = ot_fused->ti[m].info[last].H_hist.clone();
    //            info_new.pos       = cv::Point2f(ot_fused->ti[m].info[last].pos.x + 100.0 * ot_fused->ti[m].info[last].vel.x,
    //                                             ot_fused->ti[m].info[last].pos.y + 100.0 * ot_fused->ti[m].info[last].vel.y);
    //            qDebug()<<ot_fused->ti[m].info[last].pos.x<<ot_fused->ti[m].info[last].pos.y<<
    //                      info_new.pos.x<<info_new.pos.y;
    //            info_new.pc        = SensorBase::cart2Polarf(ot_fused->ti[m].info[last].pos);
    //            info_new.plot_pt_f = point2FusedTopView(info_new.pc);
    //            info_new.rect_f    = cv::Rect(info_new.plot_pt_f.x - 0.5 * ot_fused->ti[m].info[last].rect_f.width,
    //                                          info_new.plot_pt_f.y - 0.5 * ot_fused->ti[m].info[last].rect_f.height,
    //                                          ot_fused->ti[m].info[last].rect_f.width, ot_fused->ti[m].info[last].rect_f.height);
    //            lock_data_fused.unlock();
    //            // update object
    //            lock_ot_fused.lockForWrite();
    //            ot_fused->ti[m].info.push_back(info_new);
    //            ot_fused->ti[m].track_status = TRACK_STATUS::PREDICT_TARGET;
    //            ot_fused->ti[m].fg_update = true;
    //            ot_fused->ti[m].trajectory.push_back(SensorBase::PC(info_new.pc.range, info_new.pc.angle));

    //            // Kalman filter
    //            last = ot_fused->ti[m].info.size() - 1;
    //            if (last >= 2) {
    //                // (cm), (m/s), (m/s^2)
    //                cv::Point2f pt_pre2t = ot_fused->ti[m].info[last - 2].pos;
    //                cv::Point2f pt_pre1t = ot_fused->ti[m].info[last - 1].pos;
    //                cv::Point2f pt_now = ot_fused->ti[m].info[last].pos;
    //                float time_proc = 1.0 * sv->time_proc;
    //                cv::Point2f vel_prev2t_1t = SensorBase::velEstimation(pt_pre1t, pt_pre2t, time_proc, VEL_ESTI::CM_PER_FRAME2M_PER_SEC);
    //                cv::Point2f vel_prev1t_now = SensorBase::velEstimation(pt_now, pt_pre1t, time_proc, VEL_ESTI::CM_PER_FRAME2M_PER_SEC);
    //                cv::Point2f acc = cv::Point2f((vel_prev1t_now.x - vel_prev2t_1t.x) / time_proc, (vel_prev1t_now.y - vel_prev2t_1t.y) / time_proc);
    //                ot_fused->ti[m].info[last].vel.x = vel_prev1t_now.x;
    //                ot_fused->ti[m].info[last].vel.y = vel_prev1t_now.y;
    //                ot_fused->ti[m].info[last].acc.x = acc.x;
    //                ot_fused->ti[m].info[last].acc.y = acc.y;

    //                ot_fused->ti[m].kf.prediction = ot_fused->ti[m].kf.kf_core.predict();
    //                ot_fused->ti[m].kf.predictPt = cv::Point((int)(ot_fused->ti[m].kf.prediction.at<float>(0)), (int)(ot_fused->ti[m].kf.prediction.at<float>(1)));

    //                ot_fused->ti[m].kf.measurement.at<float>(0) = pt_now.x;
    //                ot_fused->ti[m].kf.measurement.at<float>(1) = pt_now.y;
    //                ot_fused->ti[m].kf.measurement += ot_fused->ti[m].kf.kf_core.measurementMatrix * ot_fused->ti[m].kf.state;

    //                ot_fused->ti[m].kf.estimated = ot_fused->ti[m].kf.kf_core.correct(ot_fused->ti[m].kf.measurement);
    //                ot_fused->ti[m].kf.statePt = cv::Point(ot_fused->ti[m].kf.estimated.at<float>(0), ot_fused->ti[m].kf.estimated.at<float>(1));
    //                ot_fused->ti[m].trajectory_kf.push_back(ot_fused->ti[m].kf.statePt);
    //                int last_kf = ot_fused->ti[m].trajectory_kf.size() - 1;
    //                if (last_kf >= 2) {
    //                    // (cm), (m/s), (m/s^2)
    //                    cv::Point2f pt_pre2t = cv::Point2f(ot_fused->ti[m].trajectory_kf[last_kf - 2]);
    //                    cv::Point2f pt_pre1t = cv::Point2f(ot_fused->ti[m].trajectory_kf[last_kf - 1]);
    //                    cv::Point2f pt_now = cv::Point2f(ot_fused->ti[m].trajectory_kf[last_kf]);
    //                    float time_proc = 1.0 * sv->time_proc;
    //                    cv::Point2f vel_prev2t_1t = SensorBase::velEstimation(pt_pre1t, pt_pre2t, time_proc, VEL_ESTI::CM_PER_FRAME2M_PER_SEC);
    //                    cv::Point2f vel_prev1t_now = SensorBase::velEstimation(pt_now, pt_pre1t, time_proc, VEL_ESTI::CM_PER_FRAME2M_PER_SEC);
    //                    cv::Point2f acc = cv::Point2f((vel_prev1t_now.x - vel_prev2t_1t.x) / time_proc, (vel_prev1t_now.y - vel_prev2t_1t.y) / time_proc);
    //                    ot_fused->ti[m].info[last].vel.x = vel_prev1t_now.x;
    //                    ot_fused->ti[m].info[last].vel.y = vel_prev1t_now.y;
    //                    ot_fused->ti[m].info[last].acc.x = acc.x;
    //                    ot_fused->ti[m].info[last].acc.y = acc.y;
    //                }
    //                randn(ot_fused->ti[m].kf.processNoise, cv::Scalar(0), cv::Scalar::all(sqrt(ot_fused->ti[m].kf.kf_core.processNoiseCov.at<float>(0, 0))));
    //                ot_fused->ti[m].kf.state = ot_fused->ti[m].kf.kf_core.transitionMatrix * ot_fused->ti[m].kf.state + ot_fused->ti[m].kf.processNoise;
    //            }
    //            lock_ot_fused.unlock();
    //        }
    //    }

    lock_ot_fused.lockForWrite();
    for (int m = 0; m < ot_fused->ti.size(); m++) {
        if ((ot_fused->ti[m].track_status != TRACK_STATUS::NO_TARGET && !ot_fused->ti[m].fg_update) ||
                ot_fused->ti[m].track_status == TRACK_STATUS::PREDICT_TARGET) {
            ot_fused->ti[m].missed_count++;
            if (ot_fused->ti[m].missed_count == ot_fused->thresh_count) {
                ot_fused->resetObjectTracking(ot_fused->ti[m]);
            }
        }
        else
            ot_fused->ti[m].missed_count = 0;
    }
    lock_ot_fused.unlock();
}

void SensorInfo::dataCollisionAvoidance()
{
    if (!fg_ca_astar)
        return;

    // reset A* map
    ca->astar->resetMap();

    lock_data_fused.lockForRead();
    for (int p = 0 ; p < size_data_fused; p++) {
        if (data_fused[p].det_mode == DETECT_MODE::SV_ONLY ||
                data_fused[p].det_mode == DETECT_MODE::SV_RADAR) {
            cv::Point tl, br;
            tl = cv::Point(data_fused[p].rect_f.tl().x, data_fused[p].rect_f.tl().y);
            br = cv::Point(data_fused[p].rect_f.br().x, data_fused[p].rect_f.br().y);
            lock_a_star.lockForWrite();
            for (int c = tl.x; c <= br.x; c++) {
                for (int r = tl.y; r <= br.y; r++) {
                    ca->astar->kernelDilation(cv::Point(c, r),
                                              cv::Size(vehicle.width_pixel, vehicle.length_pixel));
                }
            }
            lock_a_star.unlock();
        }
        else if (data_fused[p].det_mode == DETECT_MODE::RADAR_ONLY) {
            ca->astar->kernelDilation(data_fused[p].plot_pt_f, cv::Size(vehicle.width_pixel + 9, vehicle.length_pixel + 11));
        }
    }
    lock_data_fused.unlock();

    // A* path planning
    lock_a_star.lockForWrite();
    ca->path = ca->astar->PathFind2D(320, 320, 320, 0);
    lock_a_star.unlock();
#ifdef debug_info_ca_astar
    int ratio = 1;
    int display_w = MAP_WIDTH * ratio;
    int display_h = MAP_HEIGHT * ratio;
    cv::Mat img_result = cv::Mat(display_h, display_w, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int c = 0; c < MAP_WIDTH; c++) {
        for (int r = 0; r < MAP_HEIGHT; r++) {
            if (ca->astar->map[c][r] == 1) {
                for (int rr = r * ratio; rr < (r + 1) * ratio; rr++) {
                    for (int cc = c * ratio; cc < (c + 1) * ratio; cc++) {
                        img_result.at<cv::Vec3b>(rr, cc) = cv::Vec3b(0, 0, 0);
                    }
                }
            }
        }
    }

    if (!ca->path.empty()) {
        for (int k = 0; k < ca->path.size(); k++) {
            int map_x = ca->path[k].first;
            int map_y = ca->path[k].second;
            for (int r = map_y * ratio; r < (map_y + 1) * ratio; r++) {
                for (int c = map_x * ratio; c < (map_x + 1) * ratio; c++) {
                    img_result.at<cv::Vec3b>(r, c) = cv::Vec3b(128, 128, 128);
                }
            }
        }
    }

    cv::imshow("A* result", img_result);
    //    cv::imwrite("Astar.jpg", img_result);
#endif
}

void SensorInfo::dataProcess(bool fg_sv, bool fg_radar)
{
    stereo_vision::objectInfo *d_sv = sv->objects_display;
    RadarController::objectTrackingInfo *d_radar = rc->objects_display;

    // Stereo vision
    int device = SENSOR::SV;
    if (fg_sv) {
        for (int k = 0; k < sv->objSize(); k++) {
            if (d_sv[k].labeled) {
                d_sv[k].pc_world = coordinateTransform(CT_SCS2WCS, sensors[device].pos_pixel, d_sv[k].pc);
                d_sv[k].plot_pt_f = point2FusedTopView(sensors[device].pos_pixel, d_sv[k].pc);
                d_sv[k].rect_f = rectOnFusedTopView(d_sv[k].plot_pt_f, d_sv[k].rect);
            }
        }
    }
    // RADAR
    device = SENSOR::RADAR;
    if (fg_radar) {
        for (int m = 0; m < rc->objSize(); m++) {
            if (d_radar[m].status >= rc->obj_status_filtered) {
                d_radar[m].pc = SensorBase::PC(100 * d_radar[m].range, d_radar[m].angle + sensors[device].location.theta);
                d_radar[m].plot_pt_f = point2FusedTopView(sensors[device].pos_pixel, d_radar[m].pc);
                d_radar[m].pc_world = coordinateTransform(CT_SCS2WCS, sensors[device].pos_pixel, d_radar[m].pc);
            }
        }
    }

    // Data fusion & transform data type for later process
    int count = 0;
    for (int m = 0; m < rc->objSize(); m++)
        d_radar[m].fg_fused = false;

    if (fg_sv) {
        // 1. within SV's rect
        // 2. closest point
        for (int k = 0; k < sv->objSize(); k++) {
            if (!d_sv[k].labeled) continue;

            cri.clear();
            double deviation_x, deviation_z, deviation, deviation_tmp;
            if (fg_fusion) {
                U_D = 800;    // max distance error (cm) //**// tuned param
                R_sv = pow(U_D * d_sv[k].pc_world.range / (1.0 * sv->max_distance), 2);  // (cm)
                sv_pos = SensorBase::polar2Cartf(d_sv[k].pc_world);
                //                std::cout<<"SV: "<<sv_pos.x<<" "<<sv_pos.y<<std::endl;
                for (int m = 0; m < rc->objSize(); m++) {
                    if (d_radar[m].status >= rc->obj_status_filtered && !d_radar[m].fg_fused) {
                        if (d_radar[m].pc_world.angle < -1 * sensors[SENSOR::SV].angle_half_fov ||
                                d_radar[m].pc_world.angle > sensors[SENSOR::SV].angle_half_fov ||
                                d_radar[m].pc_world.range * sin(d_radar[m].pc_world.angle * CV_PI / 180.0) > sv->max_distance)
                            continue;
                        cv::Point2d radar_pos = SensorBase::polar2Cartf(d_radar[m].pc_world);
                        deviation_x = pow((radar_pos.x - sv_pos.x), 2);
                        deviation_z = pow((radar_pos.y - sv_pos.y), 2);
                        deviation = deviation_x + deviation_z;
                        //                        std::cout<<deviation<<" "<<closest_radar_distance<<" "<<R_sv<<std::endl;
                        //                        std::cout<<"RADAR: "<<radar_pos.x<<" "<<radar_pos.y<<std::endl;
                        double related_ratio = abs(radar_pos.y - sv_pos.y) / abs(radar_pos.x - sv_pos.x);
                        double slope = 2.0;
                        // 1. within rect
                        if (d_radar[m].plot_pt_f.x <= (int)(d_sv[k].rect_f.br().x) && d_radar[m].plot_pt_f.x >= (int)(d_sv[k].rect_f.tl().x) &&
                                d_radar[m].plot_pt_f.y <= (int)(d_sv[k].rect_f.br().y) && d_radar[m].plot_pt_f.y >= (int)(d_sv[k].rect_f.tl().y)) {
                            cri.push_back(m);
                        }
                        // 2. within range
                        else if ((deviation < R_sv && (related_ratio >= slope || radar_pos.x - sv_pos.x == 0.0)) ||
                                 (deviation < 0.5 * R_sv && related_ratio < slope)) {
                            //                        if (deviation < R_sv) {
                            //                            qDebug()<<deviation<<related_ratio<<"\t"<<sv_pos.x<<sv_pos.y<<radar_pos.x<<radar_pos.y;
                            bool fg_closest = true;
                            for (int p = 0; p < sv->objSize(); p++) {
                                if (p == k || !d_sv[p].labeled) continue;
                                cv::Point2d sv_pos_tmp = SensorBase::polar2Cartf(d_sv[p].pc_world);
                                double related_ratio_tmp = abs(radar_pos.y - sv_pos_tmp.y) / abs(radar_pos.x - sv_pos_tmp.x);
                                deviation_tmp = pow(radar_pos.x - sv_pos_tmp.x, 2) + pow(radar_pos.y - sv_pos_tmp.y, 2);
                                if (deviation_tmp < deviation && related_ratio < related_ratio_tmp) {
                                    fg_closest = false;
                                    break;
                                }
                            }
                            if (fg_closest) {
                                cri.push_back(m);
                            }
                        }
                    }
                }
            }

            // un-fused - stereo vision
            if (cri.empty()) {
                data_fused[count].det_mode = DETECT_MODE::SV_ONLY;
                data_fused[count].ids.now = count;
                data_fused[count].img = d_sv[k].img.clone();
                data_fused[count].rect_f = d_sv[k].rect_f;
                data_fused[count].plot_pt_f = d_sv[k].plot_pt_f;
                data_fused[count].pc = d_sv[k].pc_world;
                data_fused[count].pos = SensorBase::polar2Cartf(data_fused[count].pc);
                data_fused[count].center = std::pair<int, int>(d_sv[k].center.first, d_sv[k].center.second);
                count++;
            }
            // fusion data
            else if (fg_fusion && !cri.empty()) {
                double var_sv = pow(0.2260, 2);
                double var_radar = pow(0.0204, 2);
                double var_fused = var_sv * var_radar / (var_sv + var_radar);
                data_fused[count].det_mode = DETECT_MODE::SV_RADAR;
                data_fused[count].ids.now = count;
                data_fused[count].img = d_sv[k].img.clone();
                radar_mean = SensorBase::PC();
                radar_plot_pt_f = cv::Point(0, 0);
                radar_vel = cv::Point2f(0.0, 0.0);
                for (int p = 0; p < cri.size(); p++) {
                    radar_mean.range += d_radar[cri[p]].pc_world.range;
                    radar_mean.angle += d_radar[cri[p]].pc_world.angle;
                    radar_plot_pt_f.x += d_radar[cri[p]].plot_pt_f.x;
                    radar_plot_pt_f.y += d_radar[cri[p]].plot_pt_f.y;
                    d_radar[cri[p]].fg_fused = true;
                }
                radar_mean.range /= (1.0 * cri.size());
                radar_mean.angle /= (1.0 * cri.size());
                radar_plot_pt_f.x = (1.0 * radar_plot_pt_f.x) / (1.0 * cri.size());
                radar_plot_pt_f.y = (1.0 * radar_plot_pt_f.y) / (1.0 * cri.size());
                data_fused[count].pc = SensorBase::PC((d_sv[k].pc_world.range / var_sv + radar_mean.range / var_radar) * var_fused,
                                                      (d_sv[k].pc_world.angle / var_sv + radar_mean.angle / var_radar) * var_fused);
                data_fused[count].pos = SensorBase::polar2Cartf(data_fused[count].pc);
                data_fused[count].center = std::pair<int, int>(d_sv[k].center.first, d_sv[k].center.second);
                data_fused[count].plot_pt_f = cv::Point((d_sv[k].plot_pt_f.x / var_sv + radar_plot_pt_f.x / var_radar) * var_fused,
                                                        (d_sv[k].plot_pt_f.y / var_sv + radar_plot_pt_f.y / var_radar) * var_fused);
                cv::Point tl_new = cv::Point(data_fused[count].plot_pt_f.x - 0.5 * d_sv[k].rect_f.width, data_fused[count].plot_pt_f.y - 0.5 * d_sv[k].rect_f.height);
                data_fused[count].rect_f = cv::Rect(tl_new.x, tl_new.y, d_sv[k].rect_f.width, d_sv[k].rect_f.height);
                count++;
            }
        }
    }

    // un-fused - radar
    if (fg_radar) {
        for (int m = 0; m < rc->objSize(); m++) {
            if (d_radar[m].status >= rc->obj_status_filtered && !d_radar[m].fg_fused) {
                data_fused[count].det_mode = DETECT_MODE::RADAR_ONLY;
                data_fused[count].ids.now = count;
                data_fused[count].img = NULL;
                data_fused[count].rect_f = cv::Rect();
                data_fused[count].plot_pt_f = d_radar[m].plot_pt_f;
                data_fused[count].pc = d_radar[m].pc_world;
                data_fused[count].pos = SensorBase::polar2Cartf(data_fused[count].pc);
                d_radar[m].fg_fused = true;
                count++;
            }
        }
    }
}

void SensorInfo::drawFusedTopView(bool fg_sv, bool fg_radar, bool fg_sv_each, bool fg_ot_trajectory, bool fg_ot_trajectory_raw, bool fg_ot_trajectory_kalman, bool fg_ot_kf)
{
    stereo_vision::objectInfo *d_sv = sv->objects_display;

    bool fg_update = false;
    if (t.elapsed() > time_gap) {
        fg_update = true;
        lock_f_topview.lockForWrite();
        fused_topview.setTo(cv::Scalar(0, 0, 0, 0));
        lock_f_topview.unlock();
        t.restart();
    }
    else return;

    // Sensor - Stereo vision
    std::string tag;
    int device = SENSOR::SV;
    if (fg_sv) {
        // every pixel
        if (fg_sv_each) {
            cv::Scalar color_pixel = sensors[device].color;
            color_pixel[3] = 100;
            lock_f_topview.lockForWrite();
            for (int r = 0; r < IMG_H; r++) {
                for (int c = 0; c < IMG_W; c++) {
                    if (sv->data[r][c].disp > 0) {
                        SensorBase::PC pc = SensorBase::PC(sqrt(pow((double)(sv->data[r][c].Z), 2) + pow((double)(sv->data[r][c].X), 2)),
                                                           atan(1.0 * sv->data[r][c].X / (1.0 * sv->data[r][c].Z)) * 180.0 / CV_PI);
                        cv::Point plot_pt = point2FusedTopView(sensors[device].pos_pixel, pc);
                        cv::circle(fused_topview, plot_pt, 1, color_pixel, -1, 8, 0);
                    }
                }
            }
            lock_f_topview.unlock();
        }

        // objects
        lock_f_sv.lockForWrite();
        if (sv->img_r_L.empty())
            sv->img_detected_display = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC3);
        else
            sv->img_detected_display = sv->img_r_L.clone();
        lock_f_sv.unlock();
        for (int k = 0; k < sv->objSize(); k++) {
            if (d_sv[k].labeled) {
                // calculate world range and angle
                lock_f_sv.lockForWrite();
                if (d_sv[k].br != std::pair<int, int>(-1, -1) && d_sv[k].tl != std::pair<int, int>(-1, -1)) {
                    cv::rectangle(sv->img_detected_display, cv::Rect(d_sv[k].tl.second, d_sv[k].tl.first, (d_sv[k].br.second - d_sv[k].tl.second), (d_sv[k].br.first - d_sv[k].tl.first)),
                                  d_sv[k].color, sv->thick_obj_rect, 8, 0);
                    cv::circle(sv->img_detected_display, cv::Point(d_sv[k].center.second, d_sv[k].center.first), sv->radius_obj_point, cv::Scalar(0, 255, 0), -1, 8, 0);
                    std::string distance_tag = QString::number(d_sv[k].pc_world.range / 100.0, 'g', range_precision).toStdString() + " M";
                    cv::putText(sv->img_detected_display, distance_tag, cv::Point(d_sv[k].tl.second, d_sv[k].br.first - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, cv::Scalar(0, 0, 255), 2);
                }
                lock_f_sv.unlock();
            }
        }
    }

    // un-tracking mode (tracking button not checked)
    if (!fg_ot) {
        lock_f_topview.lockForWrite();
        lock_data_fused.lockForRead();
        for (int p = 0; p < size_data_fused; p++) {
            switch (data_fused[p].det_mode) {
            case DETECT_MODE::SV_RADAR:
                tag = QString::number(p).toStdString() + ", " + QString::number(data_fused[p].pc.range / 100, 'g', range_precision).toStdString();
                cv::circle(fused_topview, data_fused[p].plot_pt_f, thickness + 2, cv::Scalar(139, 0, 139, 255), -1, 8, 0);
                cv::rectangle(fused_topview, data_fused[p].rect_f, cv::Scalar(255, 255, 0, 255), 1, 8, 0);
                cv::putText(fused_topview, tag, cv::Point(data_fused[p].plot_pt_f.x - 50, data_fused[p].plot_pt_f.y), font, font_size, cv::Scalar(139, 0, 139, 255), font_thickness);
                break;
            case DETECT_MODE::SV_ONLY:
                device = SENSOR::SV;
                tag = QString::number(p).toStdString() + ", " + QString::number(data_fused[p].pc.range / 100, 'g', range_precision).toStdString();
                cv::circle(fused_topview, data_fused[p].plot_pt_f, thickness, sensors[device].color, -1, 8, 0);
                cv::rectangle(fused_topview, data_fused[p].rect_f, cv::Scalar(255, 255, 0, 255), 1, 8, 0);
                cv::putText(fused_topview, tag, data_fused[p].plot_pt_f, font, font_size, sensors[device].color, font_thickness);
                break;
            case DETECT_MODE::RADAR_ONLY:
                device = SENSOR::RADAR;
                tag = QString::number(p).toStdString() + ", " + QString::number(data_fused[p].pc.range / 100, 'g', range_precision).toStdString();
                cv::circle(fused_topview, data_fused[p].plot_pt_f, thickness, sensors[device].color, -1, 8, 0);
                // display range and id
                if (data_fused[p].pc.angle <= sensors[SENSOR::SV].angle_half_fov &&
                        data_fused[p].pc.angle >= -1 * sensors[SENSOR::SV].angle_half_fov && data_fused[p].pc.range < 3000 ||
                        data_fused[p].pc.range < 500) {
                    cv::putText(fused_topview, tag, cv::Point(data_fused[p].plot_pt_f.x, data_fused[p].plot_pt_f.y + 15), font, font_size, sensors[device].color, font_thickness);
                }
                break;
            }
        }
        lock_data_fused.unlock();
        lock_f_topview.unlock();
    }
    // tracking mode
    else {
        // Fusion
        lock_f_topview.lockForWrite();
        lock_ot_fused.lockForRead();
        for (int p = 0; p < ot_fused->ti.size(); p++) {
            int last = ot_fused->ti[p].info.size() - 1;
            if (last < 0 || !ot_fused->ti[p].fg_update) continue;
            if (fg_ot_trajectory_raw) {
                for (int i = 1; i < ot_fused->ti[p].info.size(); i++) {
                    cv::line(fused_topview, point2FusedTopView(ot_fused->ti[p].info[i].pc), point2FusedTopView(ot_fused->ti[p].info[i - 1].pc), ot_fused->ti[p].color_trajectory, 1, 8, 0);
                }
            }
            cv::Scalar color_pt;
            cv::Point pos_text;
            cv::Scalar color_rect = cv::Scalar(255, 255, 0, 255);
            switch (ot_fused->ti[p].info[last].det_mode) {
            case DETECT_MODE::SV_RADAR:
                cv::rectangle(fused_topview, ot_fused->ti[p].info[last].rect_f, color_rect, 1, 8, 0);
                color_pt = cv::Scalar(139, 0, 139, 255);
                pos_text = cv::Point(ot_fused->ti[p].info[last].plot_pt_f.x - 50, ot_fused->ti[p].info[last].plot_pt_f.y);
                break;
            case DETECT_MODE::SV_ONLY:
                device = SENSOR::SV;
                cv::rectangle(fused_topview, ot_fused->ti[p].info[last].rect_f, color_rect, 1, 8, 0);
                color_pt = sensors[device].color;
                pos_text = ot_fused->ti[p].info[last].plot_pt_f;
                break;
            case DETECT_MODE::RADAR_ONLY:
                device = SENSOR::RADAR;
                color_pt = sensors[device].color;
                pos_text = ot_fused->ti[p].info[last].plot_pt_f;
                break;
            }
            tag = QString::number(p).toStdString() + ", " + QString::number(ot_fused->ti[p].info[last].pc.range / 100, 'g', range_precision).toStdString();
            cv::circle(fused_topview, point2FusedTopView(ot_fused->ti[p].info[last].pc), 2, color_pt, -1, 8, 0);
            cv::putText(fused_topview, tag, pos_text, font, font_size, color_pt, font_thickness);
            // raw / kf data velocity -> processed in dataTracking()
            if (fg_ot_vel && last >= 2) {
                float scale = 10.0;
                cv::Point p1 = point2FusedTopView(ot_fused->ti[p].info[last].pc);
                cv::Point v = scale * ot_fused->ti[p].info[last].vel;
                cv::Point p2 = cv::Point(p1.x + v.x, p1.y - v.y);
#ifdef debug_info_object_tracking_vel
                if (p == 0)
                    qDebug()<<"frame"<<re.vr->current_frame_count<<"id"<<p<<"vel"<<ot_fused->ti[p].info[last].vel.x<<ot_fused->ti[p].info[last].vel.y;
#endif
                drawArrow(fused_topview, p1, p2, 5, 30, cv::Scalar(255, 100, 255, 255), 2, 8);
                tag = QString::number(ot_fused->ti[p].info[last].vel.x, 'g', 2).toStdString() + ", " + QString::number(ot_fused->ti[p].info[last].vel.y, 'g', 2).toStdString() + " m/s";
                lock_f_sv.lockForWrite();
                cv::putText(fused_topview, tag, cv::Point(pos_text.x, pos_text.y + 20), font, font_size, color_pt, font_thickness);
                lock_f_sv.unlock();
            }
        }
        lock_ot_fused.unlock();
        lock_f_topview.unlock();

        // Kalman filter
        if (fg_ot_kf && fg_ot_trajectory_kalman) {
            lock_ot_fused.lockForRead();
            for (int i = 0; i < ot_fused->ti.size(); i++) {
                if (!ot_fused->ti[i].fg_update) continue;
                // trajectory ploting
                if (ot_fused->ti[i].trajectory_kf.size() >= 2) {
                    for (int j = 1; j < ot_fused->ti[i].trajectory_kf.size(); j++) {
                        cv::line(fused_topview, point2FusedTopView(ot_fused->ti[i].trajectory_kf[j - 1]), point2FusedTopView(ot_fused->ti[i].trajectory_kf[j]), cv::Scalar(255, 255, 255, 255), 1);
                    }
                }
                // predicted point
                if (ot_fused->ti[i].trajectory_kf.size() > 0)
                    cv::circle(fused_topview, point2FusedTopView(ot_fused->ti[i].kf.predictPt), 2, cv::Scalar(255, 255, 255, 255), -1, 8, 0);
            }
            lock_ot_fused.unlock();
        }
    }

    // Pre-collision warning
    if (gui_display_time >= gui_display_max) {
        emit guiDisplay(GUI::APPROACHING, false);
        emit guiDisplay(GUI::LEFT_ONCOMING, false);
        emit guiDisplay(GUI::RIGHT_ONCOMING, false);
        gui_display_time = 0;
    }
    for (int p = 0; p < ot_fused->ti.size(); p++) {
        int last = ot_fused->ti[p].info.size() - 1;
        if (last < 0 || !ot_fused->ti[p].fg_update) continue;
#ifdef debug_info_ca_pcw
        qDebug()<<"pre-collision warning"<<ot_fused->ti[p].info[last].pc.range<<SensorBase::velUnitTransform(ot_fused->ti[p].info[last].vel, VEL_UNIT::M_PER_SEC2KM_PER_HR).y;
#endif
        cv::Point2f vel = SensorBase::velUnitTransform(ot_fused->ti[p].info[last].vel, VEL_UNIT::M_PER_SEC2KM_PER_HR);
        // frontal collision
        if ((ot_fused->ti[p].info[last].rect_f.tl().x >= (vehicle.VCP.x - 0.5 * vehicle.width_pixel) && ot_fused->ti[p].info[last].rect_f.tl().x <= (vehicle.VCP.x + 0.5 * vehicle.width_pixel)) ||
                (ot_fused->ti[p].info[last].rect_f.br().x >= (vehicle.VCP.x - 0.5 * vehicle.width_pixel) && ot_fused->ti[p].info[last].rect_f.br().x <= (vehicle.VCP.x + 0.5 * vehicle.width_pixel))) {
            if (ot_fused->ti[p].info[last].pc.range <= 200 ||
                    (ot_fused->ti[p].info[last].pc.range <= 500 && vel.y < -5.0)) {
#ifdef debug_info_ca_pcw
                qDebug()<<"in frontal";
#endif
                emit guiDisplay(GUI::APPROACHING, true);
            }
        }
        // frontal side collision
        else {
            if (ot_fused->ti[p].info[last].pc.range <= 500) {
                bool fg_potential = false;
                for (int j = 0; j < 5; j++) {
                    cv::Point2f predict = cv::Point2f(ot_fused->ti[p].info[last].pos.x + j * ot_fused->ti[p].info[last].vel.x,
                                                      ot_fused->ti[p].info[last].pos.y + j * ot_fused->ti[p].info[last].vel.y);
                    if (predict.x >= -0.5 * vehicle.width && predict.x <= 0.5 * vehicle.width) {
                        cv::circle(fused_topview, point2FusedTopView(predict), 3, cv::Scalar(255, 255, 255, 255), -1, 8, 0);
                        fg_potential = true;
                    }
                }
                if (!fg_potential) continue;
#ifdef debug_info_ca_pcw
                qDebug()<<"in frontal side";
#endif
                if (vel.x > 0 && ot_fused->ti[p].info[last].pc.angle < 5.0)
                    emit guiDisplay(GUI::LEFT_ONCOMING, true);
                if (vel.x < 0 && ot_fused->ti[p].info[last].pc.angle > 5.0)
                    emit guiDisplay(GUI::RIGHT_ONCOMING, true);
            }
        }
        // side collision
        //        if (ot_fused->ti[p].info.pc < ) {

        //        }
    }
    gui_display_time++;

    // Collision avoidance
    // A*
    if (fg_ca_astar && !ca->path.empty()) {
        lock_f_topview.lockForWrite();
        for (int k = 0; k < ca->path.size(); k++) {
            int map_x = ca->path[k].first;
            int map_y = ca->path[k].second;
            for (int r = map_y; r < map_y + 1; r++) {
                for (int c = map_x; c < map_x + 1; c++) {
                    fused_topview.at<cv::Vec4b>(r, c) = cv::Vec4b(255, 0, 0, 255);
                }
            }
        }
        lock_f_topview.unlock();
    }


    if (fg_update) {
        emit updateGUI(&fused_topview, &sv->img_detected_display);
    }
}

SensorBase::PC SensorInfo::coordinateTransform(int type, cv::Point sensor_pos, SensorBase::PC pc_in)
{
    double range = pc_in.range;
    double angle = pc_in.angle;
    SensorBase::PC pc_out;
    switch (type) {
    case CT_SCS2WCS:
        double x_world, z_world; // (cm)
        x_world = (double)(range * sin(angle * CV_PI / 180.0) + sensor_pos.x);
        z_world = (double)(range * cos(angle * CV_PI / 180.0) + sensor_pos.y - vehicle.head_pos_pixel);

        pc_out.angle = atan(x_world / z_world) * 180.0 / CV_PI;
        pc_out.range = sqrt(pow(x_world, 2) + pow(z_world, 2));
        break;
    case CT_WCS2SCS:
        break;
    }

    return pc_out;
}

cv::Point SensorInfo::point2FusedTopView(cv::Point sensor_pos, SensorBase::PC pc)
{
    double x_tmp, z_tmp; // (cm)
    x_tmp = pc.range * sin(pc.angle * CV_PI / 180.0);
    z_tmp = pc.range * cos(pc.angle * CV_PI / 180.0);
    int out_x, out_y;
    out_x = vehicle.VCP.x + (x_tmp * ratio + sensor_pos.x);
    out_y = vehicle.VCP.y - (z_tmp * ratio + sensor_pos.y);

    return cv::Point(out_x, out_y);
}

cv::Point SensorInfo::point2FusedTopView(SensorBase::PC pc_world)
{
    double x_tmp, z_tmp; // (cm)
    x_tmp = pc_world.range * sin(pc_world.angle * CV_PI / 180.0);
    z_tmp = pc_world.range * cos(pc_world.angle * CV_PI / 180.0);
    int out_x, out_y;
    out_x = vehicle.VCP.x + (x_tmp * ratio);
    out_y = vehicle.VCP.y - (z_tmp * ratio);

    return cv::Point(out_x, out_y);
}

cv::Point SensorInfo::point2FusedTopView(cv::Point pos_world)
{
    int out_x, out_y;
    out_x = vehicle.VCP.x + (pos_world.x * ratio);
    out_y = vehicle.VCP.y - (pos_world.y * ratio);
    return cv::Point(out_x, out_y);
}

cv::Rect SensorInfo::rectOnFusedTopView(cv::Point pt_pixel, cv::Rect rect_in)
{
    double w_pixel = rect_in.width * ratio;
    double h_pixel = rect_in.height * ratio;
    return cv::Rect(pt_pixel.x - w_pixel * 0.5, pt_pixel.y - h_pixel * 0.5, w_pixel, h_pixel);
}

int SensorInfo::svDataExec()
{
    int stat = sv->dataExec();

    if (stat == SV::STATUS::OK)
        stat = sv->guiUpdate();

    return stat;
}

int SensorInfo::radarDataExec()
{
    int stat = rc->dataExec();

    if (stat == RADAR::STATUS::OK)
        stat = rc->guiUpdate();

    return stat;
}

bool SensorInfo::lrfDataExec()
{
    bool result = false;
    if (lrf->dataExec()) {
        lrf->guiUpdate();
        result = true;
    }
    return result;
}

void SensorInfo::lrfBufExec()
{
    lrf->pushToBuf();
}

void SensorInfo::drawArrow(cv::Mat &img, cv::Point pStart, cv::Point pEnd, int len, int alpha, cv::Scalar& color, int thickness, int lineType)
{
    cv::Point arrow;
    double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
    cv::line(img, pStart, pEnd, color, thickness, lineType);

    arrow.x = pEnd.x + len * cos(angle + CV_PI * alpha / 180);
    arrow.y = pEnd.y + len * sin(angle + CV_PI * alpha / 180);
    line(img, pEnd, arrow, color, thickness, lineType);
    arrow.x = pEnd.x + len * cos(angle - CV_PI * alpha / 180);
    arrow.y = pEnd.y + len * sin(angle - CV_PI * alpha / 180);
    line(img, pEnd, arrow, color, thickness, lineType);
}
