#ifndef SENSORINFO_H
#define SENSORINFO_H

#include <opencv2/opencv.hpp>

#include <QReadWriteLock>
extern QReadWriteLock lock_ot_fused;

// Laser range finder controller
#include "lrf_controller.h"

// Stereo vision
#include "stereo_vision.h"
#include "stereomatchparamform.h"

// Radar ESR controller
#include "radarcontroller.h"

// Object Recognition
#include "objectrecognition.h"

// object matching
#include "objectTracking/objectmatching.h"

// Object tracking
#include "objectTracking/objecttracking.h"

// Collision avoidance
#include "collisionAvoidance/collisionavoidance.h"

namespace SENSOR {
enum {
    SV,
    RADAR,
    LRF
};
}

namespace VEHICLE {
enum {
    CART,
    CAR,
    TRACTOR,
    ROBOT
};
}

namespace GUI {
enum {
    APPROACHING,
    LEFT_ONCOMING,
    RIGHT_ONCOMING
};
}

//! SensorInfor class
//!
//! \brief This class deals with all the sensors' information and also fused one.
//!
class SensorInfo : public QObject, private ObjectMatching
{
    Q_OBJECT

public:
    SensorInfo();

    ~SensorInfo();

    stereo_vision* sv;

    RadarController* rc;

    lrf_controller* lrf;

    ObjectTracking::objectTrackingInfo *data_fused;

    ObjectTracking::objectTrackingInfo *data_fused_tmp;

    ObjectTracking::objectTrackingInfo *data_fused_prev;

    void resetTrackingInfo();

    ObjectTracking* ot_fused;

    CollisionAvoidance* ca;

    // Status =================
    bool fg_proc;
    bool fg_sv;
    bool fg_sv_each_pixel;
    bool fg_radar;
    bool fg_data_update;
    bool fg_fusion;
    bool fg_ot;
    bool fg_ot_trajectory;
    bool fg_ot_trajectory_raw;
    bool fg_ot_trajectory_kalman;
    bool fg_ot_vel;
    bool fg_ot_kf;
    bool fg_ot_pf;
    bool fg_ca_astar;
    bool fg_ca_vfh;
    // Status ================= End

    // Sensors' information ===
    //!
    //! \brief Retrieving data from stereo vision.
    //! \return int status
    //!
    int svDataExec();

    //!
    //! \brief Retrieving data from radar.
    //! \return int status
    //!
    int radarDataExec();

    //!
    //! \brief Retrieving data from laser rangefinder.
    //! \return bool receive status (not used. 20150525)
    //!
    bool lrfDataExec();

    //!
    //! \brief Retrieving lrf data to buf.
    //!
    void lrfBufExec();
    // Sensors' information === End

    // Fusion =================
    QPixmap pic_sv, pic_radar;

    cv::Mat fused_topview_BG;

    cv::Mat fused_topview;

    void chooseVehicle(int vehicle_type);

    void initialFusedTopView(int range_pixel);

    void updateFusedTopView();

    void updateFusedData();

    void dataExec();

    void zoomOutFusedTopView();

    void zoomInFusedTopView();
    // Fusion ================= End

    int test_range;

    // Analysis ===============
    int range_filter_min;
    int range_filter_max;
    // Analysis =============== End

private:    
    // sensor location
    struct sensorLocation {
        // sensor's position based on the center of vehicle. (cm) forward is x+, lateral right is y+.
        cv::Point2f pos;

        // Orientation of sensor (degree). Forward is set as zero and counting clockwisely.
        float theta;
    };

    QTime t;
    int time_gap;

    // Fusion =================
    int size_data_fused;

    void resetFusion();

    // fusion param
    double var_sv;
    double var_radar;
    double var_fused;

    std::vector<int> cri;                           // array of closest_radar's id
    double U_D;                                     // max distance error (cm)
    double R_sv;                                    // (cm)
    cv::Point2d sv_pos;
    SensorBase::PC radar_mean;
    cv::Point radar_plot_pt_f;
    cv::Point2f radar_vel;

    void dataProcess(bool fg_sv, bool fg_radar);    // fused topview and fusion information process

    // Object matching ========
    void resetMatchedInfo(ObjectTracking::objectTrackingInfo &src);

    void connectMatchedInfo(ObjectTracking::objectTrackingInfo &src, ObjectTracking::objectTrackingInfo &dst);

    void dataMatching();

    std::vector<matchedResult> matching_result;
    // Object matching ======== End
    cv::RNG rng;

    void dataTracking();

    void dataCollisionAvoidance();

    int gui_display_time;
    int gui_display_max = 20;
    void drawFusedTopView(bool fg_sv, bool fg_radar, bool fg_sv_each, bool fg_ot_trajectory, bool fg_ot_trajectory_raw, bool fg_ot_trajectory_kalman, bool fg_ot_kf);

    int range_precision;

    // topview
    int detection_range_pixel;                      // fused topview radius (pixel)

    float detection_range;                          // real detection range radius (cm)

    float ratio;                                    // scaling ratio of real to fused topview (cm -> pixel)

    float max_detection_range;                      // max. detection range in all sensors (cm)

    float min_detection_range;                      // min. detection range in all sensors (cm)

    int thickness;                                  // object params on topview

    int font;

    int font_size;

    int font_thickness;

    // Information of objects detected by different sensors
    // Based on fused topview
    struct sensorInformation {
        sensorLocation location;                    // Sensor location

        cv::Point pos_pixel;                        // (pixel)

        float angle_half_fov;                       // the half fov for displaying (degree)

        cv::Scalar color;                           // color of different sensors on the fused topview

        cv::Scalar color_fov;                       // color of fov lines

        sensorInformation() {
            pos_pixel = cv::Point(-1, -1);

            angle_half_fov = -1.0;

            color = cv::Scalar(0, 0, 0);

            color_fov = cv::Scalar(200, 200, 200, 255);
        }
    };

    sensorInformation *sensors;

    struct vehicleInfo {
        cv::Point VCP;                              // fused topview vehicle current position (pixel)

        double width;                               // average vehicle's size (cm)

        double length;                              // (cm)

        int width_pixel;                            // (pixel)

        int length_pixel;                           // (pixel)

        double head_pos;                            // (cm)

        int head_pos_pixel;                         // (pixel)

        cv::Rect rect;                              // (pixel)

        cv::Scalar color;                           // vehicle color
    } vehicle;

    int gap = 500;

    enum {
        CT_WCS2SCS,
        CT_SCS2WCS
    };

    //!
    //! \brief coordinateTransform. The transformation of coordinates system
    //! \param int type
    //! \param cv::Point sensor_pos
    //! \param SensorBase::PC pc_in
    //! \return SensorBase::PC pc_out
    //!
    SensorBase::PC coordinateTransform(int type, cv::Point sensor_pos, SensorBase::PC pc_in);
    cv::Rect rectOnFusedTopView(cv::Point pt_pixel, cv::Rect rect_in);
    cv::Point point2FusedTopView(cv::Point sensor_pos, SensorBase::PC pc);
    cv::Point point2FusedTopView(SensorBase::PC pc_world);
    cv::Point point2FusedTopView(cv::Point pos_world);
    // Fusion ================= End

private slots:
    void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha, cv::Scalar& color, int thickness = 1, int lineType = 8);

signals:
    void updateGUI(cv::Mat *fused_topview, cv::Mat *img_detected_display);

    void guiDisplay(int type, bool fg_on);
};

#endif // SENSORINFO_H
