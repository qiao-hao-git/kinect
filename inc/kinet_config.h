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
        k4a::image depthImage_k4a; // 深度图片
        k4a::image infraredImage_k4a; // 红外图片
        bool Used_Img = true; // 是否被Img使用
        bool Used_PointXYZRGB = true; // 是否被PointXYZRGB使用
    }Raw_Kinect_Data_;

    bool kill_thread_capture = false;
    pthread_t id_thread_capture;


    enum Type{
        Img,
        PointXYZRGB
    };
public:
    /*初始化KinectAzureDK相机*/
    void init();

    void start_Capture();

    /*相机拍摄的原始的数据*/
    void KinectAzureDK_Source_Grabber(k4a::image &colorImage_k4a, k4a::image &depthImage_k4a, k4a::image &infraredImage_k4a, uint8_t timeout_ms, Type type);

    //将相机原始图片转化为opencv 和 pcl格式的图片
    std::vector<cv::Mat> getImg (uint8_t timeout_ms=100);

    /*获取点云*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointXYZRGB(size_t timeout_ms=100);

    void close(); // 关闭KinectAzureDK相机

    /*计算时间差*/
    double get_time_diff(struct timeval t1, struct timeval t2);

    /*捕获线程*/
    static void* thread_capture(void *arg);

};

#endif //RUN_KINET_CONFIG_H