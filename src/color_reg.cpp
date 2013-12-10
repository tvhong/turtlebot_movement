/*
   Some of the codes are copied from ros tutorial on cv_brdige
*/
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

static const string WINDOW_BARS = "window_bars";
static const string WINDOW_ORIGINAL = "window_original";
static const string WINDOW_FILTERED = "window_filtered";

class ImageConverter {

  public:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

  private:
    int lowerH;
    int lowerS;
    int lowerV;
    int upperH;
    int upperS;
    int upperV;

  public:
    ImageConverter() : it(nh) {
        // Subscribe to input video feed and publish output video feed
        image_sub = it.subscribe("/camera/rgb/image_raw", 1, 
                &ImageConverter::imageCb, this);
        image_pub = it.advertise("/image_converter/output_video", 1);

        init();
        setWindowSettings();
    }

    ~ImageConverter() {
        cv::destroyWindow(WINDOW_BARS);
        cv::destroyWindow(WINDOW_FILTERED);
        cv::destroyWindow(WINDOW_ORIGINAL);
    }

  private:
    // This function initialize values for color filter bars
    void init() {
        lowerH=0;
        lowerS=0;
        lowerV=0;
        upperH=180;
        upperS=256;
        upperV=256;
    }

    // This function initialize 2 windows, 1 contains adjusting bars
    // and the other is to display the filtered image
    void setWindowSettings(){
        cv::namedWindow(WINDOW_BARS);
        cv::namedWindow(WINDOW_FILTERED);
        cv::namedWindow(WINDOW_ORIGINAL);

        cv::createTrackbar("LowerH", WINDOW_BARS, &lowerH, 180, NULL);
                cv::createTrackbar("UpperH", WINDOW_BARS, &upperH, 180, NULL);

        cv::createTrackbar("LowerS", WINDOW_BARS, &lowerS, 256, NULL);
                cv::createTrackbar("UpperS", WINDOW_BARS, &upperS, 256, NULL);

        cv::createTrackbar("LowerV", WINDOW_BARS, &lowerV, 256, NULL);
                cv::createTrackbar("UpperV", WINDOW_BARS, &upperV, 256, NULL);
    }


    // This function is used to filter image
    cv_bridge::CvImagePtr getThresholdedImage(cv_bridge::CvImagePtr cvOrg) {
        //cv_bridge::CvImagePtr cvThre = new cv_bridge::CvImage(cvOrg->header, cvOrg->encoding, cvOrg->image);
        cv::inRange(cvOrg->image, cv::Scalar(lowerH, lowerS, lowerV), cv::Scalar(upperH, upperS, upperV), cvOrg->image);
        return cvOrg;
    }

    // This function is called everytime a frame is received
    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cvPtr;
        try {
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        /*
        // Draw an example circle on the video stream
        if (cvPtr->image.rows > 60 && cvPtr->image.cols > 60)
            cv::circle(cvPtr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
        */
        cv::imshow(WINDOW_ORIGINAL, cvPtr->image);
        cv::cvtColor(cvPtr->image, cvPtr->image, CV_BGR2HSV);
        cvPtr = getThresholdedImage(cvPtr);
        //cout << "M=" << endl << cvPtr->image << endl << endl;

        // Update GUI Window
        cv::imshow(WINDOW_FILTERED, cvPtr->image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub.publish(cvPtr->toImageMsg());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
