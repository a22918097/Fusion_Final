#ifndef STEREO_VISION_H
#define STEREO_VISION_H

#include "debug_info.h"

//#include <QThread>
#include <QImage>
#include <QFile>
#include <QDir>
extern QDir project_path;

// thread control
#include <QReadWriteLock>
extern QReadWriteLock lock;

#include <opencv2/opencv.hpp>

#define IMG_W 640
#define IMG_H 480
#define IMG_DIS_W 320
#define IMG_DIS_H 240

#define MIN_DISTANCE 200 // cm
#define MAX_DISTANCE 3000

namespace SV {

enum STEREO_MATCH{
    SGBM,
    BM
};

enum INPUT_SOURCE {
    CAM,
    IMG
};

}

class stereo_vision : public QObject
{
    Q_OBJECT

public:
    explicit stereo_vision();

    ~stereo_vision();

    int match_mode;

    bool open(int device_index_L, int device_index_R);

    bool isOpened() {return fg_cam_opened;}

    void close();

    bool loadRemapFile(int cam_focal_length, double base_line);

    bool stereoVision();

    // RGB images for displaying
    cv::Mat img_L;
    cv::Mat img_R;

    // calibrated images
    cv::Mat img_r_L;
    cv::Mat img_r_R;

    // status
    int input_mode;
    bool fg_calib;                      // check whether the calibration button is checked
    bool fg_stereoMatch;                // check whether do the correspondence matching

    // disparity image
    cv::Mat disp_raw;
    cv::Mat disp;

    // psuedo-color table
    QImage *color_table;
    cv::Mat disp_pseudo;

    void pseudoColorTable();

    // depth estimatiom
    struct camParam
    {
        double param_r;
        double focal_length;
        int cam_focal_length;
        double base_line;
    }cam_param;

    // data - stereo vision ========
    struct StereoData
    {
        short int disp;
        int X;
        int Y;
        int Z;
        int marked; // located cell in topview
                    // ex: row = 10, col = 95 -> marked = 10095 (1000 * row + col);

        StereoData() {
            disp = -1;
            X = -1;
            Y = -1;
            Z = -1;
            marked = -1;
        }
    };

    StereoData** data;
    // ============================= End

    // Stereo vision params ========
    void matchParamInitialize(int type);

    void updateParamsSmp();

    struct matchParamSGBM
    {
        int pre_filter_cap;
        int SAD_window_size;
        int min_disp;
        int num_of_disp;
        int uniquenese_ratio;
        int speckle_window_size;
        int speckle_range;
    }param_sgbm;

    struct matchParamBM
    {
        int pre_filter_size;
        int pre_filter_cap;
        int SAD_window_size;
        int min_disp;
        int num_of_disp;
        int texture_thresh;
        int uniquenese_ratio;
        int speckle_window_size;
        int speckle_range;
    }param_bm;

    // ============================= End

private:
    void resetOpen(int device_index_L, int device_index_R);

    void camCapture();

    bool rectifyImage();

    void stereoMatch();

    // status
    bool fg_cam_L, fg_cam_R;            // open or not
    bool fg_cam_opened;
    bool fg_calib_loaded;               // load the calibration files or not

    // capture from camera
    int device_index_L;
    int device_index_R;
    cv::VideoCapture cam_L;
    cv::VideoCapture cam_R;

    // cv::Mat type capture image
    cv::Mat cap_L;
    cv::Mat cap_R;

    // stereo calibration stuffs
    QDir remap_path;
    QString remap_folder;
    QString remap_file;
    cv::Mat rmapLx, rmapLy, rmapRx, rmapRy;
    cv::Rect calibROI[2];

    // correspondence matching method
    cv::Ptr<cv::StereoSGBM> sgbm;
    cv::Ptr<cv::StereoBM> bm;
    int cn;

    cv::Mat img_match_L;
    cv::Mat img_match_R;


private slots:
    // BM ===========================
    void change_bm_pre_filter_size(int value);

    void change_bm_pre_filter_cap(int value);

    void change_bm_sad_window_size(int value);

    void change_bm_min_disp(int value);

    void change_bm_num_of_disp(int value);

    void change_bm_texture_thresh(int value);

    void change_bm_uniqueness_ratio(int value);

    void change_bm_speckle_window_size(int value);

    void change_bm_speckle_range(int value);
    // ============================== End

    // SGBM =========================
    void change_sgbm_pre_filter_cap(int value);

    void change_sgbm_sad_window_size(int value);

    void change_sgbm_min_disp(int value);

    void change_sgbm_num_of_disp(int value);

    void change_sgbm_uniqueness_ratio(int value);

    void change_sgbm_speckle_window_size(int value);

    void change_sgbm_speckle_range(int value);
    // ============================== End

signals:
    void sendCurrentParams(std::vector<int> param);

    void svUpdateGUI(cv::Mat *img_L, cv::Mat *img_R, cv::Mat *disp);

    void setConnect(int old_mode, int new_mode);
};

class top_view
{

public:
    typedef stereo_vision::StereoData StereoData;

    top_view();

    ~top_view();

    // malloc and set the grid coordinates
    void initialTopView();

    void drawTopViewLines(int rows_interval, int cols_interval);

    void pointProjectTopView(StereoData **data, QImage *color_table, bool fg_plot_points);

    bool isInitialized() {return fg_topview;}

    cv::Point** img_grid;   // topview background cell points

    cv::Mat topview;    // topview on label

    cv::Mat topview_BG;

private:
    void releaseTopView();

    void resetTopView();

    bool fg_topview;    // whether topview is initialized

    int img_col;    // topview size on label

    int img_col_half;

    int img_row;

    int z_min;  // minimum detection depth

    float c;    //**// the number of adjacent image columns grouped into a polar slice

    float k;    // length of interval

    float view_angle;   // the view angle of stereo vision

    int chord_length;   // the chord length of stereo vision

    int** grid_map; // Storing pixels into cells

    int thresh_free_space;  // check whether the cell is satisfied as an object.
                            // Each cell containing more than this value is consider as an object.

};

#endif // STEREO_VISION_H
