#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

class ImageConverter {

    public:

        ImageConverter(ros::NodeHandle &n) :
            n_(n), it_(n_)
    {
        image_pub_ = it_.advertise("image_topic_2",1);

        cvNamedWindow("Image window");
        image_sub_ = it_.subscribe(
                "/wide_stereo/left/image_rect_color", 1, &ImageConverter::imageCallback, this);
    }

        ~ImageConverter()
        {
            cvDestroyWindow("Image window");
        }

        void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
        {

            IplImage *cv_image = NULL;
            try
            {
                cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
            }
            catch (sensor_msgs::CvBridgeException error)
            {
                ROS_ERROR("error");
            }

            if (cv_image->width > 60 && cv_image->height > 60)
                cvCircle(cv_image, cvPoint(50,50), 10, cvScalar(255));

            cvShowImage("Image window", cv_image);
            cvWaitKey(3);

            try
            {
                image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8"));
            }
            catch (sensor_msgs::CvBridgeException error)
            {
                ROS_ERROR("error");
            }

        }

    protected:

        ros::NodeHandle n_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        sensor_msgs::CvBridge bridge_;
        image_transport::Publisher image_pub_;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle n;
    ImageConverter ic(n);
    ros::spin();
    return 0;
}

