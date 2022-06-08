#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <hector_uav_msgs/EnableMotors.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#import "constants.h"


class SimpleMover {

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber image_down;
    ros::Subscriber image_front;
    ros::Subscriber position_sub;
    ros::Subscriber sonar_sub;
    ros::ServiceClient motor_client;
    ros::Rate rate = ros::Rate(30);

    cv_bridge::CvImagePtr cv_ptr;
    
    double _cx;
    double _cy;

    double _cz;
    double _cz2;
    double _ca;
    double _gaF = 0;
    double _gaD = 0;
    double _sa = 100.0;
    double _offsetF = 0.0;
    double _offsetD = 0.0;
    double _prevY = 0;
    double _prevZ = 0;
    double _prevA = 0;
    
    bool _isBlueCircleDetectFront = false;
    bool _isRedCircleDetectFront = false;
    bool _isRedCircleDetectDown = false;
    
    cv::Point _circleBlueCenter{};
    cv::Point _circleRedCenter{};
    double _circleRedRadius{};
    double _circleBlueRadius{};
    
    int _cameraDownWidth = 0;
    int _cameraDownHeight = 0;
    int _cameraFrontWidth = 0;
    int _cameraFrontHeight = 0;

  public:

    SimpleMover() {
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        image_down = nh.subscribe("/cam_1/camera/image", 1, &SimpleMover::camera_cb, this);
        image_front = nh.subscribe("/cam_2/camera/image", 1, &SimpleMover::camera_cb2, this);
        position_sub = nh.subscribe("/ground_truth/state", 1, &SimpleMover::odom_cb, this);
        sonar_sub = nh.subscribe("/sonar_height", 1, &SimpleMover::sonar_cb, this);
        motor_client = nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
        

        ros::Duration(1).sleep();       // требуется для инициализации времени
    }                                   // при слишком быстром старте узла


    ~SimpleMover() {}





    ////////// CALLBACK FUNCTIONS //////////

    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {

        if (_sa == 100.0)
            _sa = msg->pose.pose.orientation.z;

        _cx = msg->pose.pose.position.x;
        _cy = msg->pose.pose.position.y;
        _cz2 = msg->pose.pose.position.z;
        _ca = msg->pose.pose.orientation.z - _sa;
    }

    void sonar_cb(const sensor_msgs::Range::ConstPtr &msg) {

        _cz = msg->range;
    }

    void camera_cb(const sensor_msgs::Image::ConstPtr &msg) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }


        show_image(cv_ptr);
    }

    void camera_cb2(const sensor_msgs::Image::ConstPtr &msg) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }


        show_image2(cv_ptr);
    }





    ////////// FIND ROUTE FUNCTIONS //////////

    //Поиск дальней точки траектории по изображению
    cv::Point findMaxFront(const std::vector<cv::Point> &pixels)
    {
        cv::Point max_pair = cv::Point(0, _cameraFrontHeight);
        for (const cv::Point &pixel : pixels)
            if (pixel.y > _cameraFrontHeight/2 && pixel.y <= max_pair.y)
            {
                max_pair.x = pixel.x;
                max_pair.y = pixel.y;
            }
        return max_pair;
    }

    //Поиск ближней точки траектории по изображению для ниждей камеры
    cv::Point findMaxDown(const std::vector<cv::Point> &pixels)
    {
        cv::Point max_pair = cv::Point(0, _cameraDownHeight);
        for (const cv::Point &pixel : pixels)
            if (pixel.y <= max_pair.y)
            {
                max_pair.x = pixel.x;
                max_pair.y = pixel.y;
            }
        return max_pair;
    }

    //Поиск ближней точки траектории по изображению для передней камеры
    cv::Point findMin(const std::vector<cv::Point> &pixels)
    {
        cv::Point min_pair = cv::Point(0, 0);
        for (const cv::Point &pixel : pixels)
            if (pixel.y >= min_pair.y)
            {
                min_pair.x = pixel.x;
                min_pair.y = pixel.y;
            }
        return min_pair;
    }

    //Поиск необходимого смещения текущего положения относительно желаемой троектории
    double findOffset(const std::vector<cv::Point> &pixels, int cameraHeight)
    {
        double offset = 0;
        for (const cv::Point &pixel : pixels)
            offset += (double)(cameraHeight*0.8-pixel.x)/(double)cameraHeight;

        return offset;
    }





    ////////// DOWN CAMERA IMAGE PROCESSING //////////
    void show_image(const cv_bridge::CvImagePtr cv_ptr) {

        cv::Mat image = cv_ptr->image; 
        cv::Mat img = image(cv::Range(0,image.rows*0.7), cv::Range(0,image.cols));
        _cameraDownWidth = img.cols;
        _cameraDownHeight = img.rows;

        ////////// FINDING RED CIRCLE UNDER DRONE
        cv::Mat imageRed;
        GaussianBlur(img, imageRed, cv::Size(9, 9), 0);
        std::vector<cv::Mat> splitedImgRed = std::vector<cv::Mat>();
        split(imageRed, splitedImgRed);
        int redPixels = 0;
        //Поиск красных пикселей на изображении с нижней камеры
        for (int y = 0; y < imageRed.cols; y++) {
            for (int x = 0; x < imageRed.rows/2; x++) {

                int B = static_cast<int>(splitedImgRed[0].at<uchar>(x, y));
                int G = static_cast<int>(splitedImgRed[1].at<uchar>(x, y));
                int R = static_cast<int>(splitedImgRed[2].at<uchar>(x, y));

                if (B < 60 && G < 60 && R > 75)
                {
                    imageRed.at<cv::Vec3b>(x, y)[0] = 0;
                    imageRed.at<cv::Vec3b>(x, y)[1] = 0;
                    imageRed.at<cv::Vec3b>(x, y)[2] = 255;
                    ++redPixels;
                }
            }
        }
        //Проверка кол-ва красных пикстелей на достаточное кол-во, чтобы считать их частью круга
        _isRedCircleDetectDown = ((double)redPixels/((double)imageRed.rows*(double)imageRed.cols) > 0.000001)? true: false;


        ////////// FINDING LINE UNDER DRONE
        cv::Mat bw;
        GaussianBlur(img, img, cv::Size(7, 7), 0);
        std::vector<cv::Mat> splitedImg = std::vector<cv::Mat>();
        split(img, splitedImg);
        //Поиск серых пикселей траектории на изображении с нижней камеры
        for (int y = 0; y < img.cols; y++) {
            for (int x = 0; x < img.rows; x++) {

                int B = static_cast<int>(splitedImg[0].at<uchar>(x, y));
                int G = static_cast<int>(splitedImg[1].at<uchar>(x, y));
                int R = static_cast<int>(splitedImg[2].at<uchar>(x, y));


                if ((B > 80 && G > 80 && R > 80) && 
                    (B < 105 && G < 105 && R < 105) && 
                    (G > B-7 && G < B+7) &&
                    (R > B-7 && R < B+7)) 
                {
                    img.at<cv::Vec3b>(x, y)[0] = 255;
                    img.at<cv::Vec3b>(x, y)[1] = 255;
                    img.at<cv::Vec3b>(x, y)[2] = 255;
                }
                else
                {
                    img.at<cv::Vec3b>(x, y)[0] = 0;
                    img.at<cv::Vec3b>(x, y)[1] = 0;
                    img.at<cv::Vec3b>(x, y)[2] = 0;
                }
            }
        }

        //Перевод картинки с закрашеными пикселями траектории в бинарный вид
        cv::cvtColor(img, bw, CV_BGR2GRAY);
        threshold(bw, bw, 250, 255, CV_THRESH_OTSU);

        //Поиск контура вокруг серых пикселей с наибольшей площадью
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> maxCnt;
        std::vector<cv::Vec4i> hierarchy;
        findContours(bw, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
        if (!contours.empty())
        {
            int maxCntId = 666;
            double maxArea = 100;
            for (int i = 0; i < contours.size(); ++i)
            {
                double area = contourArea(contours[i]);
                if (area > maxArea)
                {
                    maxCntId = i;
                    maxArea = area;
                }
            }
            if (maxCntId != 666)
                {
                    maxCnt = contours[maxCntId];
                    std::vector<std::vector<cv::Point>> maxCnts = {maxCnt};
                    drawContours(imageRed, maxCnts, -1, cv::Scalar(0, 255, 0), 2);
                }
        }

        //Поиск желаемого угла и смещения от желаемой траектории
        _offsetD = maxCnt.size()? findOffset(maxCnt, _cameraDownHeight)/maxCnt.size(): _offsetF;
        cv::Point maxPointDown = findMaxDown(maxCnt);
        cv::Point minPointDown = findMin(maxCnt);
        _gaD = (maxPointDown.y != _cameraDownHeight)? _ca + std::atan2(minPointDown.x - maxPointDown.x, minPointDown.y - maxPointDown.y): _gaF;

        cv::imshow("camera down", imageRed);
        cv::waitKey(3);

    }





    ////////// FRONT CAMERA IMAGE PROCESSING //////////
    void show_image2(const cv_bridge::CvImagePtr cv_ptr) {
        
        cv::Mat image = cv_ptr->image;
        cv::Mat img = image(cv::Range(image.rows*0.2,image.rows), cv::Range(0,image.cols));
        _cameraFrontWidth = img.cols;
        _cameraFrontHeight = img.rows;

        cv::Mat imageCircleCopy;
        GaussianBlur(img, imageCircleCopy, cv::Size(5, 5), 0);
        ////////// FINDING RED CIRCLE IN FRONT OF DRONE
        
        
        cv::Mat imageRed=imageCircleCopy.clone();
        std::vector<cv::Mat> splitedImgRed = std::vector<cv::Mat>();
        split(imageRed, splitedImgRed);

        //Поиск красных кругов на изображении с фронтальной камеры
        for (int y = 0; y < imageRed.cols; y++) {
            for (int x = 0; x < imageRed.rows; x++) {

                int B = static_cast<int>(splitedImgRed[0].at<uchar>(x, y));
                int G = static_cast<int>(splitedImgRed[1].at<uchar>(x, y));
                int R = static_cast<int>(splitedImgRed[2].at<uchar>(x, y));

                if (B < 40 && G < 40 && R > 90)
                {
                    imageRed.at<cv::Vec3b>(x, y)[0] = 255;
                    imageRed.at<cv::Vec3b>(x, y)[1] = 255;
                    imageRed.at<cv::Vec3b>(x, y)[2] = 255;
                }
                else
                {
                    imageRed.at<cv::Vec3b>(x, y)[0] = 0;
                    imageRed.at<cv::Vec3b>(x, y)[1] = 0;
                    imageRed.at<cv::Vec3b>(x, y)[2] = 0;
                }
            }
        }


        cvtColor(imageRed, imageRed, CV_RGB2GRAY);
        Canny(imageRed, imageRed, 20, 70);

        std::vector<cv::Vec3f> circlesRed;
        HoughCircles(imageRed, circlesRed, CV_HOUGH_GRADIENT, 1, 10, 100, 20, 20, 0);

        //Поиск максимального круга
        int maxCircleRadiusRed = 0;
        cv::Point maxCircleCenterRed(_cameraFrontWidth/2, _cameraFrontHeight/2);
        bool isRedCircleDetect = false;
        for (size_t i = 0; i < circlesRed.size(); i++) {
            isRedCircleDetect = true;
            cv::Point center(cvRound(circlesRed[i][0]), cvRound(circlesRed[i][1]));
            int radius = cvRound(circlesRed[i][2]);
            if (radius < maxCircleRadiusRed)
                continue;
            maxCircleRadiusRed = radius;
            maxCircleCenterRed = center;
        }
        //Рисование найденого круга
        circle(imageCircleCopy, maxCircleCenterRed, maxCircleRadiusRed, cv::Scalar(0, 0, 255), 5);
        _isRedCircleDetectFront = isRedCircleDetect;
        _circleRedCenter = maxCircleCenterRed;
        _circleRedRadius = maxCircleRadiusRed;
        
        ////////// FINDING BLUE CIRCLE IN FRONT OF DRONE
        cv::Mat imageBlue=imageCircleCopy.clone();
        std::vector<cv::Mat> splitedImgBlue = std::vector<cv::Mat>();
        split(imageBlue, splitedImgBlue);

        //Поиск красных кругов на изображении с фронтальной камеры
        for (int y = 0; y < imageBlue.cols; y++) {
            for (int x = 0; x < imageBlue.rows; x++) {

                int B = static_cast<int>(splitedImgBlue[0].at<uchar>(x, y));
                int G = static_cast<int>(splitedImgBlue[1].at<uchar>(x, y));
                int R = static_cast<int>(splitedImgBlue[2].at<uchar>(x, y));

                if (B > 75 && G < 65 && R < 65)
                {
                    imageBlue.at<cv::Vec3b>(x, y)[0] = 255;
                    imageBlue.at<cv::Vec3b>(x, y)[1] = 255;
                    imageBlue.at<cv::Vec3b>(x, y)[2] = 255;
                }
                else
                {
                    imageBlue.at<cv::Vec3b>(x, y)[0] = 0;
                    imageBlue.at<cv::Vec3b>(x, y)[1] = 0;
                    imageBlue.at<cv::Vec3b>(x, y)[2] = 0;
                }

            }
        }


        cvtColor(imageBlue, imageBlue, CV_RGB2GRAY);
        Canny(imageBlue, imageBlue, 20, 70);

        std::vector<cv::Vec3f> circlesBlue;
        HoughCircles(imageBlue, circlesBlue, CV_HOUGH_GRADIENT, 1, 10, 100, 20, 20, 0);

        //Поиск максимального круга
        int maxCircleRadiusBlue = 0;
        cv::Point maxCircleCenterBlue(_cameraFrontWidth/2, _cameraFrontHeight/2);
        bool isBlueCircleDetect = false;
        for (size_t i = 0; i < circlesBlue.size(); i++) {
            isBlueCircleDetect = true;
            cv::Point center(cvRound(circlesBlue[i][0]), cvRound(circlesBlue[i][1]));
            int radius = cvRound(circlesBlue[i][2]);
            if (radius < maxCircleRadiusBlue)
                continue;
            maxCircleRadiusBlue = radius;
            maxCircleCenterBlue = center;
        }
        //Рисование найденого круга
        circle(imageCircleCopy, maxCircleCenterBlue, maxCircleRadiusBlue, cv::Scalar(255, 0, 0), 5);
        _isBlueCircleDetectFront = isBlueCircleDetect;
        _circleBlueCenter = maxCircleCenterBlue;
        _circleBlueRadius = maxCircleRadiusBlue;

        ////////// FINDING LINE IN FRONT OF DRONE
        cv::Mat bw;
        GaussianBlur(img, img, cv::Size(5, 5), 0);
        std::vector<cv::Mat> splitedImg = std::vector<cv::Mat>();
        split(img, splitedImg);
        //Поиск серых пикселей траектории на изображении с нижней камеры
        for (int y = 0; y < img.cols; y++) {
            for (int x = 0; x < img.rows; x++) {

                int B = static_cast<int>(splitedImg[0].at<uchar>(x, y));
                int G = static_cast<int>(splitedImg[1].at<uchar>(x, y));
                int R = static_cast<int>(splitedImg[2].at<uchar>(x, y));


                if ((B > 80 && G > 80 && R > 80) && 
                    (B < 105 && G < 105 && R < 105) && 
                    (G > B-7 && G < B+7) &&
                    (R > B-7 && R < B+7)) 
                {
                    img.at<cv::Vec3b>(x, y)[0] = 255;
                    img.at<cv::Vec3b>(x, y)[1] = 255;
                    img.at<cv::Vec3b>(x, y)[2] = 255;
                }
                else
                {
                    img.at<cv::Vec3b>(x, y)[0] = 0;
                    img.at<cv::Vec3b>(x, y)[1] = 0;
                    img.at<cv::Vec3b>(x, y)[2] = 0;
                }
            }
        }

        //Перевод картинки с закрашеными пикселями траектории в бинарный вид
        cv::cvtColor(img, bw, CV_BGR2GRAY);
        threshold(bw, bw, 250, 255, CV_THRESH_OTSU);

        //Поиск контура вокруг серых пикселей с наибольшей площадью
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> maxCnt;
        std::vector<cv::Vec4i> hierarchy;
        findContours(bw, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
        if (!contours.empty())
        {
            int maxCntId = 666;
            double maxArea = 100;
            for (int i = 0; i < contours.size(); ++i)
            {
                double area = contourArea(contours[i]);
                if (area > maxArea)
                {
                    maxCntId = i;
                    maxArea = area;
                }
            }
            if (maxCntId != 666)
                {
                    maxCnt = contours[maxCntId];
                    std::vector<std::vector<cv::Point>> maxCnts = {maxCnt};
                    drawContours(imageCircleCopy, maxCnts, -1, cv::Scalar(0, 255, 0), 2);
                }
        }

        //Поиск желаемого угла и смещения от желаемой траектории
        _offsetF = maxCnt.size()? findOffset(maxCnt, _cameraFrontHeight)/maxCnt.size(): _offsetD;
        cv::Point maxPointFront = findMaxFront(maxCnt);
        cv::Point minPointFront = findMin(maxCnt);
        _gaF = (maxPointFront.y != _cameraFrontHeight)? _ca + std::atan2(minPointFront.x - maxPointFront.x, minPointFront.y - maxPointFront.y): _gaD;
        
        cv::imshow("camera front circles", imageCircleCopy);
        cv::waitKey(3);
    }





    ////////// DRONE MOVING FUNCTIONS //////////

    void enable_motors() {
        ros::service::waitForService("/enable_motors");
        hector_uav_msgs::EnableMotors srv;
        srv.request.enable = true;
        if (!motor_client.call(srv)) {
            ROS_ERROR("Failed to call service enable_motors");
        }
    }

    void take_off() {

        enable_motors();

        geometry_msgs::Twist twist_msg;
        twist_msg.linear.z = HEIGHT_VELO;

        while (nh.ok() && (_cz < HEIGH_POINT)) {
            cmd_vel_pub.publish(twist_msg);
            ros::spinOnce();
            rate.sleep();
        }
    }

    geometry_msgs::Twist findVelocities()
    {
        geometry_msgs::Twist twist_msg;
        double ga = K_ANGULAR_FRONT * _gaF + K_ANGULAR_DOWN * _gaD;
        ga = (abs(ga)>0.1)? ga: 0;
        double offset = K_OFFSET_FRONT * _offsetF + K_OFFSET_DOWN * _offsetD;
        offset = (abs(offset)>0.05)? offset: 0;

        twist_msg.linear.x = LINE_VELO - abs(ga - _ca)*BACK_VELO_DA - abs(offset)*BACK_VELO_DL - _prevZ*K_UP;
        twist_msg.linear.y = tan(offset)*KPY  + (ga - _ca)*CICLE_OFFSET - _prevY*KDY;
        twist_msg.angular.z = (ga - _ca)*KPA + tan(tan(ga - _ca))*KPA_OFFSET - _prevA*KDA;

        double cz = _cz<2.9?_cz:_cz2;
        bool _isRedCircleDetect = _isRedCircleDetectDown || _isRedCircleDetectFront;
        if(_isRedCircleDetect && (_circleRedCenter.y + 2*_circleRedRadius > _cameraFrontHeight/2) && (_circleRedRadius > _circleBlueRadius))
        {
            //взлет
            twist_msg.linear.x /= 2;
            twist_msg.linear.z = twist_msg.linear.x/1.5;
        }
        else
        {
            //движение прямо
            twist_msg.linear.z = KPZ * (HEIGH_POINT - cz) - _prevZ * KDZ;
            if ( twist_msg.linear.z < 0)
                 twist_msg.linear.z /= 4;
            
             if(_isBlueCircleDetectFront)
             {
                 double distY = _cameraFrontWidth/2 - _circleBlueCenter.x;
                 double distZ = _cameraFrontHeight/2 - _circleBlueCenter.y;
                 twist_msg.linear.y += (abs(distY) > 20 && abs(distY) < 120)? K_BLUE * distY * 1.3: 0;
                 twist_msg.linear.z += (abs(distZ) > 20 && abs(distZ) < 120)? K_BLUE * distZ: 0;
             }
        }


         twist_msg.linear.z = (cz < 5)? twist_msg.linear.z: (twist_msg.linear.z < 0)? twist_msg.linear.z: 0;


        _prevY=twist_msg.linear.y;
        _prevZ=twist_msg.linear.z;
        _prevA=twist_msg.angular.z;
        
        //std::cout << "_gaF: " << _gaF << std::endl;
        //std::cout << "_gaD: " << _gaD << std::endl;
        //std::cout << "ga: " << ga << std::endl;
        //std::cout << "_offsetF: " << _offsetF << std::endl;
        //std::cout << "_offsetD: " << _offsetD << std::endl;
        //std::cout << "offset: " << offset << std::endl;
        //std::cout << "tan(offset): " << tan(offset) << std::endl;
        //std::cout << "(ga - _ca): " << (ga - _ca) << std::endl;
        //std::cout << "tan(ga - _ca): " << tan(ga - _ca) << std::endl;
        
        //std::cout << "XXX: " << _circleRedCenter.y + 2*_circleRedRadius - _cameraFrontHeight/2 << std::endl;
        //std::cout << "Blue x: " << _circleBlueCenter.x << "Blue y: " << _circleBlueCenter.y << std::endl;
        //std::cout << "Red x: " << _circleRedCenter.x << "Red y: " << _circleRedCenter.y << std::endl;
        //std::cout << "Red r: " << _circleRedRadius << std::endl << std::endl;


        return twist_msg;
    }


    void spin() {

        take_off();

        while (nh.ok()) {

            geometry_msgs::Twist twist_msg = findVelocities();
            cmd_vel_pub.publish(twist_msg);

            ros::spinOnce();
            rate.sleep();
        }
    }
};



int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_mover");

    SimpleMover simpleMover;
    sleep(1);
    simpleMover.spin();

  return 0;
}
