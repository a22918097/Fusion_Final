#include "topview.h"

TopView::TopView(int thresh_free_space, int min_distance, int max_distance,
                 float view_angle, int chord_length, int display_row, int display_col,
                 int grid_row, int grid_col)
{
    fg_topview = false;

    thick_polygon = 4;

    img_col = grid_col;

    img_col_half = img_col / 2;

    img_row = grid_row;

    c = 6.4;

    k = 0.025;

    this->thresh_free_space = thresh_free_space;

    this->min_distance = min_distance;

    this->max_distance = max_distance;

    this->view_angle = view_angle;

    this->chord_length = chord_length;

    this->display_row = display_row;

    this->display_col = display_col;

    ratio_row = 1.0 * display_row / max_distance;

    ratio_col = 1.0 * display_col / chord_length;

    color_BG = cv::Scalar(161, 158, 149, 255);

    color_tag = cv::Scalar(0, 0, 255, 255);

    color_line = cv::Scalar(0, 255, 0, 255);

    initialTopView();

    pseudoColorTable();
}

TopView::~TopView()
{
    releaseTopView();

    delete color_table;
}

void TopView::releaseTopView()
{
    if (fg_topview) {
        for (int i = 0; i < (img_row + 1); i++)
            delete[] img_grid[i];
        delete[] img_grid;

        for (int i = 0; i < (img_row); i++)
            delete[] grid_map[i];
        delete[] grid_map;

        fg_topview = false;
    }
}

void TopView::changeParams(float view_angle, int chord_length)
{
    this->view_angle = view_angle;

    this->chord_length = chord_length;
}

void TopView::pseudoColorTable()
{
    // ==== Produce pseudo-color table
    int total_color = max_distance - min_distance + 1;
    color_table = new QImage(total_color, 12, QImage::Format_RGB888) ;

    float hue_start_angle = 0.0;
    float hue_end_angle = 240.0;

    float h = 0.0;
    float s = 1.0;
    float v = 255.0;
    float step = (hue_end_angle - hue_start_angle) / (total_color);

    float f;
    int hi, p, q, t;

    for (int r = 0; r < color_table->height(); r++) {
        uchar* ptr = color_table->scanLine(r);
        for (int c =0; c < color_table->width(); c++) {
            h = c * step;
            hi = (int)(h / 60.0) % 6;
            f = h / 60.0 - hi;
            p = v * (1 - s);
            q = v * (1 - f * s);
            t = v * (1 - (1 - f) * s);

            switch (hi) {
            case 0:
                ptr[3 * c + 0] = v;
                ptr[3 * c + 1] = t;
                ptr[3 * c + 2] = p;
                break;
            case 1:
                ptr[3 * c + 0] = q;
                ptr[3 * c + 1] = v;
                ptr[3 * c + 2] = p;
                break;
            case 2:
                ptr[3 * c + 0] = p;
                ptr[3 * c + 1] = v;
                ptr[3 * c + 2] = t;
                break;
            case 3:
                ptr[3 * c + 0] = p;
                ptr[3 * c + 1] = q;
                ptr[3 * c + 2] = v;
                break;
            case 4:
                ptr[3 * c + 0] = t;
                ptr[3 * c + 1] = p;
                ptr[3 * c + 2] = v;
                break;
            case 5:
                ptr[3 * c + 0] = v;
                ptr[3 * c + 1] = p;
                ptr[3 * c + 2] = q;
                break;
            }
        }
    }
}

void TopView::initialTopView()
{
    if (!fg_topview) {
        img_grid = new cv::Point * [img_row + 1];
        for (int r = 0; r < img_row + 1; r++)
            img_grid[r] = new cv::Point[img_col + 1];

        grid_map = new blobNode * [img_row];
        for (int r = 0; r < img_row; r++)
            grid_map[r] = new blobNode[img_col];

        topview_BG.create(display_row, display_col, CV_8UC4);
        topview_BG.setTo(cv::Scalar(0, 0, 0, 0));

        topview.create(display_row, display_col, CV_8UC4);
        topview.setTo(cv::Scalar(0, 0, 0, 0));

        resetTopView();
    }

    for (int r = 0; r < img_row + 1; r++) {
        for (int c = 0; c < img_col_half + 1; c++) {
            int z = min_distance * pow(1.0 + k, r);
            int x = z * tan(0.5 * view_angle * CV_PI / 180.0 * c / img_col_half);
            x > 0.5 * chord_length ? 0.5 * chord_length : x;

            if (c == 0) {
                img_grid[r][img_col_half] = cv::Point(0.5 * chord_length - x, max_distance - z);
#ifdef debug_info_sv_topview
                cv::circle(topview, img_grid[r][img_col_half + 1], 3, cv::Scalar(0, 255, 0, 255), 5, 8, 0);
#endif
            }
            else {
                img_grid[r][img_col_half - c] = cv::Point(0.5 * chord_length - x, max_distance - z);
                img_grid[r][img_col_half + c] = cv::Point(0.5 * chord_length + x, max_distance - z);
#ifdef debug_info_sv_topview
                cv::circle(topview, img_grid[r][img_col_half + 1 - c], 3, cv::Scalar(255, 0, 0, 255), 5, 8, 0);
                cv::circle(topview, img_grid[r][img_col_half + 1 + c], 3, cv::Scalar(0, 0, 255, 255), 5, 8, 0);
#endif
            }
            if (r == img_row) {
                x = max_distance * tan(0.5 * view_angle * CV_PI / 180.0 * c / img_col_half);
                img_grid[r][img_col_half - c] = cv::Point(0.5 * chord_length - x, 0);
                img_grid[r][img_col_half + c] = cv::Point(0.5 * chord_length + x, 0);

                // get min. chord length
                if (c == img_col_half)
                    chord_length_min = 2 * x;
#ifdef debug_info_sv_topview
                cv::circle(topview, img_grid[r + 1][img_col_half + 1 - c], 3, cv::Scalar(255, 255, 0, 255), 5, 8, 0);
                cv::circle(topview, img_grid[r + 1][img_col_half + 1 + c], 3, cv::Scalar(0, 255, 255, 255), 5, 8, 0);
#endif
            }

        }
    }

#ifdef debug_info_sv_topview
    cv::Mat topview_re;
    cv::resize(topview, topview_re, cv::Size(400, 800));
    cv::imshow("hi", topview_re);
#endif

    fg_topview = true;
}

void TopView::drawTopViewLines(int rows_interval, int cols_interval, bool fg_tag)
{
    topview_BG.setTo(cv::Scalar(0, 0, 0, 0));

    // Background color
    cv::Point pts[4];
    pts[0] = pointT(img_grid[0][0]);
    pts[1] = pointT(img_grid[0][img_col]);
    pts[2] = pointT(img_grid[img_row][img_col]);
    pts[3] = pointT(img_grid[img_row][0]);
    cv::fillConvexPoly(topview_BG, pts, 4, color_BG, 8, 0);

    // distance tag
    int distance, text_dev, font_size, line_thickness;
    // 30 m
    distance = 3000;
    text_dev = 100;
    font_size = 1;
    line_thickness = 1;
    cv::Point pt1 = cv::Point(0, max_distance - distance);
    cv::Point pt2 = cv::Point(chord_length, max_distance - distance);
    cv::line(topview_BG, pointT(pt1), pointT(pt2), color_tag, line_thickness, 8, 0);
    pt1.y += text_dev;
    cv::putText(topview_BG, "30 m", pointT(pt1), cv::FONT_HERSHEY_PLAIN, font_size, color_tag, 1);
    // 2 m
    distance = 200;
    pt1.y = max_distance - distance;
    pt2.y = max_distance - distance;
    cv::line(topview_BG, pointT(pt1), pointT(pt2), color_tag, line_thickness, 8, 0);
    pt1.y += text_dev;
    cv::putText(topview_BG, "2 m", pointT(pt1), cv::FONT_HERSHEY_PLAIN, font_size, color_tag, 1);
    // 1 m
    distance = 100;
    pt1.y = max_distance - distance;
    pt2.y = max_distance - distance;
    cv::line(topview_BG, pointT(pt1), pointT(pt2), color_tag, line_thickness, 8, 0);
    pt1.y += text_dev;
    cv::putText(topview_BG, "1 m", pointT(pt1), cv::FONT_HERSHEY_PLAIN, font_size, color_tag, 1);


    if (fg_tag) {
        // 20 m
        distance = 2000;
        pt1.y = max_distance - distance;
        pt2.y = max_distance - distance;
        cv::line(topview_BG, pointT(pt1), pointT(pt2), color_tag, line_thickness, 8, 0);
        pt1.y += text_dev;
        cv::putText(topview_BG, "20 m", pointT(pt1), cv::FONT_HERSHEY_PLAIN, font_size, color_tag, 1);
        // 10 m
        distance = 1000;
        pt1.y = max_distance - distance;
        pt2.y = max_distance - distance;
        cv::line(topview_BG, pointT(pt1), pointT(pt2), color_tag, line_thickness, 8, 0);
        pt1.y += text_dev;
        cv::putText(topview_BG, "10 m", pointT(pt1), cv::FONT_HERSHEY_PLAIN, font_size, color_tag, 1);
    }

    // draw lines
    line_thickness = 1;
    for (int r = 0; r < img_row + 1; r += rows_interval) {
        cv::line(topview_BG, pointT(img_grid[r][0]), pointT(img_grid[r][img_col]), color_line, line_thickness, 8, 0);
    }
    for (int c = 0; c < img_col_half + 1; c += cols_interval) {
#ifdef debug_info_sv_topview
        std::cout<<img_col_half + 1 - c<<" ";
#endif
        cv::line(topview_BG, pointT(img_grid[0][img_col_half - c]),
                pointT(img_grid[img_row][img_col_half - c]), color_line, line_thickness, 8, 0);
        cv::line(topview_BG, pointT(img_grid[0][img_col_half + c]),
                pointT(img_grid[img_row][img_col_half + c]), color_line, line_thickness, 8, 0);
    }
#ifdef debug_info_sv_topview
    std::cout<<std::endl;
#endif
}

void TopView::resetTopView()
{
    //topview.setTo(cv::Scalar(0, 0, 0, 0));

    for (int r = 0; r < img_row; r++)
        for (int c = 0; c < img_col; c++) {
            grid_map[r][c].obj_label = -1;
            grid_map[r][c].pts_num = 0;
            grid_map[r][c].avg_Z = 0;
            grid_map[r][c].avg_X = 0;
        }

    // data mark is reset in objectProjectTopView
}

cv::Point TopView::pointT(cv::Point src)
{
    cv::Point rst;

    rst.x = 1.0 * src.x * ratio_col;
    rst.y = 1.0 * src.y * ratio_row;

    return rst;
}
