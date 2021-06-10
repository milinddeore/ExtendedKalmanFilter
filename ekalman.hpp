/*
 * MIT Licence.
 * Written by Milind Deore <tomdeore@gmail.com>
 *
 * Smooth, fluctuating mouse movements. It can be used elsewhere as well. 
 */
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


typedef struct mouse_info
{
    int x;
    int y;
}mouse_info_t;


class ExtendedKalmanFilter
{
	private:
		const float pval = 0.1;
	    const float qval = 0.0001;
	    const float rval = 0.1;
        uint32_t height;
        uint32_t width;

		// No previous prediction noise covariance
		cv::Mat P_pre;
		cv::Mat x;
		cv::Mat P_post;
		cv::Mat Q;
		cv::Mat R;
		cv::Mat I;

		cv::Mat F;
		cv::Mat h;
		cv::Mat H;
		cv::Mat G;

	public:

		//
		// Constructor 
		//
		ExtendedKalmanFilter(uint32_t n, uint32_t m)
		{
            width = n;
            height = m;

			x = cv::Mat::zeros(1, n, CV_32FC1);
			P_post = cv::Mat::eye(n, n, CV_32FC1) * pval;
			Q = cv::Mat::eye(n, n, CV_32FC1) * qval;
			R = cv::Mat::eye(m, m, CV_32FC1) * rval;
			I = cv::Mat::eye(n, n, CV_32FC1);
		}

		// 
		// Step 
		//

		cv::Mat step(float xx, float yy)
		{
            float xxyy[] = {xx, yy};
            cv::Mat z(1, 2, CV_32FC1, xxyy);

			// Predict
			F = cv::Mat::eye(2, 2, CV_32FC1);
			P_pre = F * P_post * F.t() + Q;

			// Update
			x.copyTo(h);
			H = cv::Mat::eye(2, 2, CV_32FC1);	
			cv::Mat H_P_pre_R =  H * P_pre * H.t() + R; 
			G =  P_pre * H.t() * H_P_pre_R.inv(); 
			x += (z - h) * G;
			P_post = (I - (G * H)) * P_pre;

			return x;
		}
};
