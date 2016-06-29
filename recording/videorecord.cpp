#include "videorecord.h"
#include <QDebug>
videoRecord::videoRecord(int img_h, int img_w)
{
    fg_record = false;

    fg_loaded = false;

    fg_data_end = false;

    fg_file_established = false;

    fg_frame_record = false;

    // default setting
    defaultBasicInfo(img_h, img_w);
}

videoRecord::~videoRecord()
{
    if (fg_record) {
        writer.release();
        fg_record = false;
    }
}

void videoRecord::setPath(QString str, bool fg_str_is_file)
{
    if (fg_str_is_file) {
        file_name = str.section("/", -1, -1);
        file_path = str;
    }
    else {
        file_name = str.section("/", -1, -1) + ".avi";
        file_path = str + "/" + file_name;
    }
}

void videoRecord::defaultBasicInfo(int img_h, int img_w)
{
    ex = cv::VideoWriter::fourcc('D', 'I', 'V', 'X');
    fps = 24;
    s = cv::Size(2 * img_w, img_h);
}

void videoRecord::getBasicInfo(cv::VideoCapture &cap)
{
    ex = cv::VideoWriter::fourcc('D', 'I', 'V', 'X');
//    ex = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));
    fps = cap.get(cv::CAP_PROP_FPS);
    s = cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH),
                          cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    frame_count = cap.get(cv::CAP_PROP_FRAME_COUNT);
}

void videoRecord::createVideo()
{
    if (!fg_file_established) {
        if (!writer.isOpened()) {
            writer.open(file_path.toStdString(), ex, fps, s, true);
            fg_frame_record = true;
        }

        fg_file_established = true;
    }
}

bool videoRecord::record(cv::Mat img)
{
    if (!fg_file_established) {
        createVideo();
        current_frame_count = 0;
    }

    if (fg_record) {
        if (!writer.isOpened())
            return false;
        writer.write(img);
        return true;
    }
    return false;
}

void videoRecord::stop()
{
    fg_file_established = false;
    fg_frame_record = false;
    fg_record = false;
    fg_loaded = false;
    writer.release();
    cap.release();
}

void videoRecord::combineTwoImages(cv::Mat *img_merge, cv::Mat img_1, cv::Mat img_2, cv::Size s)
{
    for (int r = 0 ; r < s.height; r++) {
        cv::Vec3b pixel_1, pixel_2;
        cv::Vec3b *pixel;
        for (int c = 0 ; c < s.width; c++) {
            pixel_1 = img_1.at<cv::Vec3b>(r, c);
            pixel = &img_merge->at<cv::Vec3b>(r, c);
            *pixel = pixel_1;
            pixel->val[0] = pixel_1.val[2];
            pixel->val[1] = pixel_1.val[1];
            pixel->val[2] = pixel_1.val[0];
            pixel_2 = img_2.at<cv::Vec3b>(r, c);
            pixel = &img_merge->at<cv::Vec3b>(r, c + s.width);
            pixel->val[0] = pixel_2.val[2];
            pixel->val[1] = pixel_2.val[1];
            pixel->val[2] = pixel_2.val[0];
        }
    }
}

bool videoRecord::segmentTwoImages(cv::Mat *img_1, cv::Mat *img_2, cv::Size s)
{
    if (!cap.isOpened())
        return false;
    cv::Mat img_merge;
    cap >> img_merge;
    if (img_merge.empty())
        return false;
    for (int r = 0 ; r < s.height; r++) {
        cv::Vec3b *pixel_1, *pixel_2;
        cv::Vec3b pixel;
        for (int c = 0 ; c < s.width; c++) {
            pixel = img_merge.at<cv::Vec3b>(r, c);
            pixel_1 = &img_1->at<cv::Vec3b>(r, c);
            pixel_1->val[0] = pixel.val[2];
            pixel_1->val[1] = pixel.val[1];
            pixel_1->val[2] = pixel.val[0];
            pixel = img_merge.at<cv::Vec3b>(r, c + s.width);
            pixel_2 = &img_2->at<cv::Vec3b>(r, c);
            pixel_2->val[0] = pixel.val[2];
            pixel_2->val[1] = pixel.val[1];
            pixel_2->val[2] = pixel.val[0];
        }
    }

    return true;
}

bool videoRecord::loadVideo(QString str, bool fg_str_is_file)
{
    setPath(str, fg_str_is_file);
    if (file_path.isEmpty()) {
        fg_loaded = false;
        return fg_loaded;
    }
    qDebug() << file_path;
    cap.open(file_path.toStdString());
    if (!cap.isOpened()) {
        fg_loaded = false;
        return fg_loaded;
    }

    current_frame_count = 0;
    getBasicInfo(cap);

    fg_loaded = true;
    return fg_loaded;
}
