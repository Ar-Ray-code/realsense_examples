#include <rclcpp/rclcpp.hpp>
//#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <unistd.h>

using std::placeholders::_1;

int initial_flag = 0;

std::string window_name = "image";
std::string depth_name = "depth";

class realsense_subscriber:public rclcpp::Node
{
    public:
        realsense_subscriber():Node("realsense_subscriber")
        {
            // cascade.load("/home/ubuntu/ros2_ws/src/realsense_subscriber/cascade/haarcascade_fullbody.xml");
            //sub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/color/image_raw", 5, std::bind(&realsense_subscriber::realsense_callback, this, _1));
            sub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/aligned_depth_to_color/image_raw", 5, std::bind(&realsense_subscriber::realsense_callback, this, _1));
            //sub_depth = this->create_subscription<sensor_msgs::msg::Image>("camera/depth/image_rect_raw", 5, std::bind(&realsense_subscriber::depth_update, this, _1));
            cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
            // cv::namedWindow(depth_name, cv::WINDOW_AUTOSIZE);
        }
        ~realsense_subscriber()
        {
            cv::destroyAllWindows();
        }
    private:
        // cv::CascadeClassifier cascade;
        std::vector<cv::Rect> contour;

        void realsense_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void depth_update(const sensor_msgs::msg::Image::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth;

        cv::Mat depth_data;
};

void realsense_subscriber::depth_update(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto data = msg->data;
    // depth_data = data->image;
}

void realsense_subscriber::realsense_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto data = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    
    //cv::cvtColor(data->image, gray, cv::COLOR_BGR2GRAY);
    // cascade.detectMultiScale(data->image, contour, 1.1, 3, 0, cv::Size(30, 30));

    // for(int i = 0; i < contour.size(); i++){
	// 	cv::rectangle(data->image, cv::Point(contour[i].x, contour[i].y), cv::Point(contour[i].x + contour[i].width, contour[i].y + contour[i].height), cv::Scalar(255, 0, 0), 1);
	// }

    cv::Point center(int(depth_data.cols/2),int(depth_data.rows/2));
    std::cout << "dist:" << 0.001*data->image.at<float>(center) << std::endl;
    cv::circle(data->image,cv::Point(int(depth_data.cols/2),int(depth_data.rows/2)),10,cv::Vec3b(0,0,255),-1);
    cv::imshow(window_name.c_str(),data->image);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<realsense_subscriber>());
    rclcpp::shutdown();
    return 0;
}