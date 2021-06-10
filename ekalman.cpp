/*
 * MIT Licence.
 * Written by Milind Deore <tomdeore@gmail.com>
 *
 * Smooth, fluctuating mouse movements. It can be used elsewhere as well. 
 *
 * Compile:
 * g++ -std=c++11 ekalman.cpp `pkg-config --cflags --libs opencv` -o kalman
 *
 * Run:
 *  ./kalman
 */
#include "ekalman.hpp"

//
// Globals
//
std::vector<cv::Point> measured_pts;
std::vector<cv::Point> kalman_pts;
ExtendedKalmanFilter ekf(2, 2);

void draw_lines(cv::Mat img, std::vector<cv::Point> pts, int r, int g, int b)
{
    cv::polylines(img, pts, 0, cv::Scalar(r, g, b));
}

cv::Mat new_image()
{
    cv::Mat img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    return img; 
}


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    mouse_info_t* m_info = (mouse_info_t*)userdata;

    if ( event == cv::EVENT_MOUSEMOVE ) {
        m_info->x = x;
        m_info->y = y;
        std::cout << "Mouse move over the window - position (" << x << ", " << y   << ")" << std::endl;

    }
    else {
        std::cout << "Unsupported event" << std::endl;
    }

    cv::Mat img = new_image();

    // Grab current mouse position and add it to the trajectory
    cv::Point point = cv::Point(m_info->x, m_info->y);
    measured_pts.push_back(point);

    cv::Mat est_pts = ekf.step((float)m_info->x, (float)m_info->y);

    cv::Point kl_point = cv::Point(est_pts.at<float>(0,0), est_pts.at<float>(0,1));
    kalman_pts.push_back(kl_point);

    draw_lines(img, kalman_pts, 0, 255, 0);
    draw_lines(img, measured_pts, 255, 255, 0);

    imshow("Kalman Mousetracker [ESC to quit]", img);
    if (cv::waitKey(0) == 27) {
        exit(0);
    }

}



int main()
{
    cv::Mat img = new_image();
    cv::namedWindow("Kalman Mousetracker [ESC to quit]", 1);

    mouse_info_t m_info = {-1, -1};

    // 
    // Set the callback function for any mouse event, we are interested 
    // in movements only.
    cv::setMouseCallback("Kalman Mousetracker [ESC to quit]", 
                         CallBackFunc, &m_info);

    while (1)
    {
        imshow("Kalman Mousetracker [ESC to quit]", img);
        if (cv::waitKey(0) == 27) {
            std::cout << "Got 'ESC' singal" << std::endl;
            exit(0);
        }
    }

	return 0;
}
