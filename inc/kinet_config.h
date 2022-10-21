//
// Created by sj3 on 22-10-20.
//

#ifndef KINET_CONFIG_H
#define KINET_CONFIG_H


#include <k4a/k4a.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>

//using namespace std;
//using namespace cv;
//using namespace pcl;




class KINECT_BASE{
private:
    k4a::device Kinect;
    k4a::calibration calibration;
    k4a::transformation transformation;


    bool Thread_Capture_Working = false;
    struct RAW_KINECT_DATA_
    {
        pthread_mutex_t mutex_k4a_image_t = PTHREAD_MUTEX_INITIALIZER; // 互斥锁
        k4a::image colorImage_k4a; // 彩色图片
        k4a::image depthImage_k4a; // 深度突破
        k4a::image infraredImage_k4a; // 红外图片
        bool Used_In_Img = true; // 是否被获取图片使用过
        bool Used_In_FastPointXYZ = true; // 是否被FastPointXYZ使用过
        bool Used_In_PointXYZ = true; // 是否被PointXYZ使用过
        bool Used_In_PointXYZRGB = true; // 是否被PointXYZRGB使用过
    }Raw_Kinect_Data_;

    bool kill_thread_capture = false;
    pthread_t id_thread_capture;
    static void* thread_capture(void *arg); // 捕获线程

    enum getType{
        Img,
        FastPointXYZ,
        PointXYZ,
        PointXYZRGB
    };
public:
    void init();
    /*初始化KinectAzureDK相机*/




// 相机拍摄的原始的数据
    void KinectAzureDK_Source_Grabber(k4a::image &colorImage_k4a, k4a::image &depthImage_k4a, k4a::image &infraredImage_k4a, uint8_t timeout_ms, getType type);

/*计算时间差*/
    double cal_time(timeval start_time, timeval end_time);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointXYZRGB(size_t timeout_ms=100);

    void start_Capture();
    void close(); // 关闭KinectAzureDK相机
    //将相机原始图片转化为opencv 和 pcl格式的图片
    std::vector<cv::Mat> getImg (uint8_t timeout_ms=100);

};




























#endif //RUN_KINET_CONFIG_H
