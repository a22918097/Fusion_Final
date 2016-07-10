#include "radarcontroller.h"

RadarController::RadarController() : TopView(1, 100, 3000, 102.3, 7409, 600, 900, 300, 400)//TopView(1, 200, 3000, 19.8, 1080, 750, 270, 125, 100)
{
    fg_read = false;
    fg_data_in = false;

    // input source
    input_mode = RADAR::INPUT_SOURCE::ESR;

    objects = new objectTrackingInfo[OBJECT_NUM];

    objects_display = new objectTrackingInfo[OBJECT_NUM];

    obj_status_filtered = 2;

    item = new QStandardItem[OBJECT_NUM];

    img_rows = 160;

    img_cols = 900;

    createLUT();

    img_center = cv::Point(img_cols / 2, img_rows / 2);

    obj_rect = cv::Point(20, 40);

    img_radar = cv::Mat::zeros(img_rows, img_cols, CV_8UC4);

    img_radar_BG = cv::Mat::zeros(img_rows, img_cols, CV_8UC3);

    drawFrontView();

    current_count = 0;

    update_count = 3;

    time_gap = 10;
    t.start();
}

RadarController::~RadarController()
{
    canClose(h);

    delete[] LUT_grid_row;
    delete[] LUT_grid_col;

    delete[] objects;
    delete[] objects_display;

    delete[] item;
}

void RadarController::createLUT()
{
    LUT_grid_row = new int[max_distance];
    LUT_grid_col = new int[chord_length];

    col_shift_LUT = 0.5 * chord_length;

    for (int m = 0; m < max_distance; m++)
        LUT_grid_row[m] = 1.0 * log10(1.0 * m / min_distance) / log10(1.0 + k);

    for (int m = 0; m < chord_length; m++)
        LUT_grid_col[m] = (1.0 * m) * img_col / chord_length;
}

int RadarController::corrGridRow(int k)
{
    int m = k > max_distance ? max_distance : k;
    if (m <= 0)
        return -1;
    return LUT_grid_row[m];
}

int RadarController::corrGridCol(int k)
{

    int m = k + col_shift_LUT;
    if (m > chord_length)
        m = chord_length;
    else if (m <= 0)
        m = 0;
    return LUT_grid_col[m];
}

bool RadarController::open()
{
    input_mode = RADAR::INPUT_SOURCE::ESR;
    canInitializeLibrary();
    h = canOpenChannel(0, canOPEN_NO_INIT_ACCESS);
    stat = canRequestBusStatistics(h);
    if (stat != canOK) {
        char msg[OBJECT_NUM];
        stat = canGetErrorText(stat, &msg[0], sizeof(msg));
        std::cout<<"ERR"<<std::endl;
        return false;
    }

    return true;
}

bool RadarController::write()
{
    BYTE msg[dlc_esr] = {0x00};
    msg[6] = 0xBF;
    stat = canWriteWait(h, id_esr, msg, dlc_esr, 0, 0xff);
//    std::cout<<"write stat: "<<stat<<std::endl;
    if (stat == canOK) {
        fg_data_in = true;
        return true;
    }

    return false;
}

void RadarController::reset()
{
    bin.clear();
    b.reset();
    b_track_lat_rate.reset();
    b_track_grouping_changed.reset();
    b_track_oncoming.reset();
    b_track_status.reset();
    b_track_angle.reset();
    b_track_range.reset();
    b_track_bridge_object.reset();
    b_track_rolling_count.reset();
    b_track_width.reset();
    b_track_range_accel.reset();
    b_track_med_range_mode.reset();
    b_track_range_rate.reset();
}

void RadarController::busOn()
{
    if (fg_read)
        return;
    switch (input_mode) {
    case RADAR::INPUT_SOURCE::ESR:
        h = canOpenChannel(0, canOPEN_NO_INIT_ACCESS);
        stat = canBusOn(h);

        if (stat == canOK)
            fg_read = true;
        break;
    case RADAR::INPUT_SOURCE::TXT:
        if (re.vr->fg_loaded)
            fg_read = true;
        break;
    }
}

void RadarController::busOff()
{
    fg_read = false;
    stat = canBusOff(h);
    stat = canClose(h);
}

bool RadarController::dataIn()
{
    bin.clear();
    switch (input_mode) {
    case RADAR::INPUT_SOURCE::ESR:
        stat = canReadWait(h, &id, &can_data[0], &dlc, &flag, &time, 0xff);
        if (stat != canOK) {
            return false;
        }

        b.reset();
        for (int i = 0; i < dlc; i++) {
            b = can_data[i];
            bin += b.to_string();
        }

        if (re.tr->fg_record) {
            std::string text;
            std::string id__;
            std::stringstream strstream, strstream_1;
            strstream << id;
            id__ = strstream.str();
            std::string frame_record__;
            if (re.vr->fg_frame_record) {
                strstream_1 << (int)(re.vr->current_frame_count);
            }
            else {
                strstream_1 << "null";
            }
            frame_record__ = strstream_1.str();

            text.append(frame_record__);
            text.append(",");
            text.append(id__);
            text.append(",");
            text.append(bin);
            re.recordData(text);
        }
        break;
    case RADAR::INPUT_SOURCE::TXT:
        fg_data_in = true;
        std::string text_raw;
        QString text, text_frame, text_id, text_bin;
        re.tr->file >> text_raw;
        if (!text_raw.empty()) {
            text = QString::fromStdString(text_raw);
            text_frame = text.section(",", 0, 0);
            text_id = text.section(",", 1, 1);
            text_bin = text.section(",", 2, 2);
            this->id = std::atol(text_id.toStdString().c_str());
            bin = text_bin.toStdString();
            // For synchronization replay
            if (text_frame == "null")
                re.tr->current_frame_count = -1;
            else
                re.tr->current_frame_count = text_frame.toInt();
        }
        else {
            emit dataEnd();
            return true;
        }
        break;
    }

    retrievingData();

    if (id == 0x53F) {
        if (input_mode == RADAR::INPUT_SOURCE::TXT) {
            // For synchronization replay
            if (re.tr->fg_loaded && re.vr->fg_loaded)
                if (re.vr->current_frame_count < re.tr->current_frame_count)
                    return false;
//            std::cout<<re.vr->current_frame_count<<"\t"<<re.tr->current_frame_count<<std::endl;
        }
        fg_all_data_in = true;
        return true;
    }
    else
        fg_all_data_in = false;

    return false;
}

int RadarController::dataExec()
{
    // dataIn() returns true when id == 0x53F
    while (!dataIn()) {}
    // flush buf data. The data retrieving freq. can't be higher than sampleing rate, so I flush the rest of them. //**//
    canFlushReceiveQueue(h);

    // For txt
    if (!fg_all_data_in)
        return RADAR::STATUS::NO_INPUT;

    if (fg_all_data_in && fg_data_in) {

        pointDisplayFrontView();

        if (fg_topview)
            pointProjectTopView();

        updateDataForDisplay();

        return RADAR::STATUS::OK;
    }

    return RADAR::STATUS::DATA_NOT_ENOUGHT;
}

void RadarController::updateDataForDisplay()
{
    lock_radar.lockForWrite();
    std::swap(objects, objects_display);
    lock_radar.unlock();
}

int RadarController::guiUpdate()
{
    if (t.elapsed() > time_gap) {
        time_proc = t_p.restart();
        emit updateGUI(detected_obj, &img_radar, &topview);

        t.restart();
        reset();

        return RADAR::STATUS::OK;
    }

    return RADAR::STATUS::NO_UPDATE;
}

void RadarController::retrievingData()
{
    if (fg_data_in) {
        int _id;
        if (id >= 0x500 && id <= 0x53F) {
            _id = id - 0x500;

            b_track_lat_rate = std::bitset<6>(bin.substr(0, 6));
            b_track_oncoming = std::bitset<1>(bin.substr(6, 1));
            b_track_grouping_changed = std::bitset<1>(bin.substr(7, 1));
            b_track_status = std::bitset<3>(bin.substr(8, 3));
            b_track_angle = std::bitset<10>(bin.substr(11, 10));
            b_track_range = std::bitset<11>(bin.substr(21, 11));
            b_track_bridge_object = std::bitset<1>(bin.substr(32, 1));
            b_track_rolling_count = std::bitset<1>(bin.substr(33, 1));
            b_track_width = std::bitset<4>(bin.substr(34, 4));
            b_track_range_accel = std::bitset<10>(bin.substr(38, 10));
            b_track_med_range_mode = std::bitset<2>(bin.substr(48, 2));
            b_track_range_rate = std::bitset<14>(bin.substr(50, 14));

#ifdef debug_info_radar_data
            std::cout<<_id<<"\t\n"<<
                    b_track_lat_rate <<" "<<
                    b_track_oncoming <<" "<<
                    b_track_grouping_changed <<" "<<
                    b_track_status <<" "<<
                    b_track_angle <<" "<<
                    b_track_range <<" "<<
                    b_track_bridge_object <<" "<<
                    b_track_rolling_count <<" "<<
                    b_track_width <<" "<<
                    b_track_range_accel <<" "<<
                    b_track_med_range_mode <<" "<<
                    b_track_range_rate <<" "<<
                    std::endl;
#endif

            lock_radar.lockForWrite();
            if (b_track_lat_rate.at(5) == 1) {
                b_track_lat_rate[5] = 0;

                objects[_id].lat_rate = b_track_lat_rate.to_ulong() * 0.25 - 8.0;
            }
            else
                objects[_id].lat_rate = b_track_lat_rate.to_ulong() * 0.25;
            objects[_id].grouping_changed = (b_track_lat_rate.to_ulong() == 1 ? true : false);
            objects[_id].oncoming = (b_track_oncoming.to_ulong() == 1 ? true : false);
            objects[_id].status = b_track_status.to_ulong();
            if (b_track_angle.at(9) == 1) {
                b_track_angle[9] = 0;
                objects[_id].angle = b_track_angle.to_ulong() * 0.1 - 51.2;
            }
            else
                objects[_id].angle = b_track_angle.to_ulong() * 0.1;
            objects[_id].range = b_track_range.to_ulong() * 0.1;
            objects[_id].bridge_object = (b_track_bridge_object.to_ulong() == 1 ? true : false);
            objects[_id].rolling_count = (b_track_rolling_count.to_ulong() == 1 ? true : false);
            objects[_id].width = b_track_width.to_ulong() * 0.5;
            if (b_track_range_accel.at(9) == 1) {
                b_track_range_accel[9] = 0;
                objects[_id].range_accel = b_track_range_accel.to_ulong() * 0.05 - 25.6;
            }
            else
                objects[_id].range_accel = b_track_range_accel.to_ulong() * 0.05;
            objects[_id].med_range_mode = b_track_med_range_mode.to_ulong();
            if (b_track_range_rate.at(13) == 1) {
                b_track_range_rate[13] = 0;
                objects[_id].range_rate = b_track_range_rate.to_ulong() * 0.01 - 81.92;
            }
            else
                objects[_id].range_rate = b_track_range_rate.to_ulong() * 0.01;
            //degree to radian
            objects[_id].x = 1.0 * objects[_id].range * sin(objects[_id].angle*CV_PI/180);
            objects[_id].y = 0.0;
            objects[_id].z = abs(objects[_id].range * cos(objects[_id].angle*CV_PI/180));
//            qDebug() << _id << objects[_id].x << objects[_id].z << objects[_id].range << objects[_id].angle;
#ifdef debug_info_radar_data
            std::cout<<_id<<"\t\n"<<
                    objects[_id].lat_rate <<" "<<
                    objects[_id].oncoming <<" "<<
                    objects[_id].grouping_changed <<" "<<
                    objects[_id].status <<" "<<
                    objects[_id].angle <<" "<<
                    objects[_id].range <<" "<<
                    objects[_id].bridge_object <<" "<<
                    objects[_id].rolling_count <<" "<<
                    objects[_id].width <<" "<<
                    objects[_id].range_accel <<" "<<
                    objects[_id].med_range_mode <<" "<<
                    objects[_id].range_rate <<" "<<
                    std::endl<<std::endl;
#endif
            lock_radar.unlock();
        }
#ifdef debug_info_radar_data
        if (id == 0x53F) {
            qDebug()<<t.elapsed();
        }
#endif
    }
}

void RadarController::drawFrontView()
{
    short_length = (img_cols / 4);
    long_length = img_cols;
    short_length_half = short_length / 2;
    long_length_half = long_length / 2;
    cv::line(img_radar_BG, cv::Point(img_center.x + short_length_half, 0), cv::Point(img_center.x + long_length_half, img_rows), cv::Scalar(255, 255, 255), 1, 8, 0);
    cv::line(img_radar_BG, cv::Point(img_center.x - short_length_half, 0), cv::Point(img_center.x - long_length_half, img_rows), cv::Scalar(255, 255, 255), 1, 8, 0);
}

void RadarController::pointDisplayFrontView()
{
    img_radar.setTo(cv::Scalar(0, 0, 0, 0));

    detected_obj = 0;

    lock_radar.lockForWrite();
    for (int k = 0; k < OBJECT_NUM; k++) {
        if (objects[k].status >= obj_status_filtered) {
            detected_obj++;

            int pt_x, pt_y;
            float gain = 5.0; //**// another value?
            pt_x = 1.0 * objects[k].x * gain * (1.0 - 1.0 * objects[k].z / max_distance) + img_center.x;
            pt_y = img_rows * (1.0 - 1.0 * objects[k].z / max_distance);
            cv::circle(img_radar, cv::Point(pt_x, pt_y), 1, cv::Scalar(0, 255, 0, 255), -1, 8, 0);
            cv::rectangle(img_radar, cv::Rect(pt_x - obj_rect.x / 2, pt_y - obj_rect.y / 2, obj_rect.x, obj_rect.y), cv::Scalar(0, 0, 255, 255), 2, 8, 0);
            cv::putText(img_radar, QString::number(k).toStdString(), cv::Point(pt_x + obj_rect.x / 2, pt_y), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0, 255));
#ifdef debug_info_radar_data
//                ui->textEdit->append("data struct\nangle: " + QString::number(objects[k].angle) + " range: " + QString::number(objects[k].range)
//                                     + " accel: " + QString::number(objects[k].range_accel) + " width: " + QString::number(objects[k].width)
//                                     + " (x,y,z) = (" + QString::number(objects[k].x) + ", " + QString::number(objects[k].y) + ", " + QString::number(objects[k].z) + ")");
#endif
            item[k].setText(QString::number(objects[k].range));
        }
        else {
            item[k].setText("0");
        }
    }
    lock_radar.unlock();
}

void RadarController::pointProjectTopView()
{
    if (current_count > update_count) {
        resetTopView();
        current_count = 0;
    }
    topview.setTo(cv::Scalar(0,0,0,0));
    int grid_row, grid_col;
    lock_radar.lockForWrite();
    for (int m = 0; m < OBJECT_NUM; m++) {
        if (objects[m].status >= obj_status_filtered) {

            grid_row = corrGridRow(100.0 * objects[m].z);
            grid_col = corrGridCol(100.0 * objects[m].x);
            cv::putText(topview,QString::number(objects[m].range).toStdString(),cv::Point(0.5*900+objects[m].x/37*450+10,600-objects[m].z/30*600+10),cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0, 255));
            cv::circle(topview,cv::Point(0.5*900+objects[m].x/37*450,600-objects[m].z/30*600),8,cv::Scalar(255,0,0,255),-1);
//            cv::circle(topview,cv::Point(0.5*270+int(objects[i].avg_X/100),710-objects[i].avg_Z/3000*710+20),8,cv::Scalar(255,255,255,255),-1);
//            qDebug() <<m <<objects[m].x <<objects[m].z << objects[m].range << objects[m].angle;
//            grid_row = 1.0 * log10(100.0 * objects[m].z / min_distance) / log10(1.0 + k);
//            grid_col = (100.0 * objects[m].x + 0.5 * chord_length) * img_col / chord_length;
            int grid_row_t = img_row - grid_row - 1;
            int grid_col_t = grid_col;
            // mark each point belongs to which cell
            if (grid_row_t >= 0 && grid_row_t < img_row &&
                    grid_col_t >= 0 && grid_col_t < img_col) {
                grid_map[grid_row_t][grid_col_t].pts_num++;
//                std::cout<<data[m].z<<std::endl;
            }
        }
    }
    lock_radar.unlock();

    // check whether the cell is satisfied as an object
    cv::Point pts[4];
    uchar* ptr = color_table->scanLine(0);
    int p;
    int gap_row  = 3;
    int gap_col  = 5;
    for (int r = 0; r < img_row; r++) {
        for (int c = 0; c < img_col; c++) {
            if (grid_map[r][c].pts_num >= thresh_free_space && grid_map[r][c].avg_Z < max_distance) {
                int row = img_row - r - gap_row > 0 ? img_row - r - gap_row : 0;
                int row_1 = img_row - (r + 1) + gap_row <= img_row ? img_row - (r + 1) + gap_row : img_row;
                int col = c - gap_col > 0 ? c - gap_col : 0;
                int col_1 = c + 1 + gap_col <= img_col ? c + 1 + gap_col : img_col;

                pts[0] = pointT(img_grid[row][col]);
                pts[1] = pointT(img_grid[row_1][col]);
                pts[2] = pointT(img_grid[row_1][col_1]);
                pts[3] = pointT(img_grid[row][col_1]);

                p = (max_distance - 0.5 * (img_grid[row][col].y + img_grid[row_1][col].y)) - min_distance;
//                cv::circle(grid_map_)
//                cv::circle(topview,


//                cv::fillConvexPoly(topview, pts, thick_polygon, cv::Scalar(ptr[3 * p + 0], ptr[3 * p + 1], ptr[3 * p + 2], 255), 8, 0);
//                cv::fillConvexPoly(topview, pts, thick_polygon, cv::Scalar(255, 255, 255, 255), 8, 0);
            }
        }
    }

    current_count++;
#ifdef debug_info_radar_data
    if (current_count >= 10) {
        cv::Mat tp;
        tp = topview.clone();
        //    cv::transpose(topview, tp);
        cv::imshow("topview - rc", tp);
    }
#endif
}
