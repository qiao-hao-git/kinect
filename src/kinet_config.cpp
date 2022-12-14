/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kinet_config.h"
pthread_t id_thread_capture;
static KINECT_BASE* KINET_BASE_Ptr_ = nullptr;

void KINECT_BASE:: init() {

    /*查询设备数量*/
    uint32_t devices_count = k4a::device::get_installed_count();
    if (devices_count == 0){
        printf("No K4a Devices Attached!\n");
        return ;
    }
    else{
        printf("Found %u Kinect Devices!\n", devices_count);
    }

    /*设置参数*/
    k4a_device_configuration_t init_params = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

    /*彩色相机的分辨率*/
    init_params.color_resolution = K4A_COLOR_RESOLUTION_720P;
    /*设置Kinect的相机帧率为30FPS*/
    init_params.camera_fps = K4A_FRAMES_PER_SECOND_30;
    /*设置Kinect的深度模式为Near FOV unbinned（这一代 Kinect 支持多种深度模式，官方文档推荐使用 K4A_DEPTH_MODE_NFOV_UNBINNED 和 K4A_DEPTH_MODE_WFOV_2X2BINNED 这两种深度模式）*/
    init_params.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
    /*设置Kinect的颜色属性为BGRA32*/
    init_params.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    /*为了同时获取depth和color图，保持这两种图像是同步的*/
    init_params.synchronized_images_only = true;

    /*打开相机*/
    int device_id_ = 0;
    Kinect = k4a::device::open(device_id_);
    Kinect.start_cameras(&init_params);

    /* 查询设备SN码*/
    std::string serial_number =  Kinect.get_serialnum();
    std::cout << "Open Kinect Device Serial Number: " << serial_number << std::endl;

    if(!Kinect){
        printf("Kinect Open Error!\n");
        return ;
    }

    /* 获取相机参数*/
    calibration = Kinect.get_calibration(init_params.depth_mode, init_params.color_resolution);
    transformation = k4a::transformation(calibration);

    // 初始化K4a_Grabber_Ptr_
    if(KINET_BASE_Ptr_ == nullptr){
        KINET_BASE_Ptr_ = this;
    }

    Kinect.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, 0);     // 曝光时间
//    Kinect.set_color_control(K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, 1);               // 亮度
    //Kinect.set_color_control(K4A_COLOR_CONTROL_CONTRAST, K4A_COLOR_CONTROL_MODE_MANUAL, 0);                 // 对比度
    // Kinect.set_color_control(K4A_COLOR_CONTROL_SATURATION, K4A_COLOR_CONTROL_MODE_MANUAL, 0);               // 饱和度
    // Kinect.set_color_control(K4A_COLOR_CONTROL_SHARPNESS, K4A_COLOR_CONTROL_MODE_MANUAL, 0);                // 锐度
    // Kinect.set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_MANUAL, 1);             // 白平衡
    //Kinect.set_color_control(K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, 0);   // 背光补偿
    // Kinect.set_color_control(K4A_COLOR_CONTROL_GAIN, K4A_COLOR_CONTROL_MODE_MANUAL, 0);                     // GAIN
    // Kinect.set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, 0);      // 电力线频率
    return ;
}


void KINECT_BASE::start_Capture()
{

    if(Thread_Capture_Working)
    {
        printf("Thread Capture Is Working!\n");
        return ;
    }

    Thread_Capture_Working = true;
    pthread_create(&id_thread_capture, NULL, thread_capture, NULL);
    return ;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr KINECT_BASE::getPointXYZRGB(size_t timeout_ms)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 获取相机原始数据
    k4a::image colorImage_k4a = nullptr, depthImage_k4a = nullptr, infraredImage_k4a = nullptr;
    KinectAzureDK_Source_Grabber(colorImage_k4a, depthImage_k4a, infraredImage_k4a, timeout_ms, PointXYZRGB);

    if(colorImage_k4a == nullptr || depthImage_k4a == nullptr)
    {
        printf("Grabber PointXYZRGB Failed!\n");
        return cloud;
    }

    // 数据格式转换
    int width = colorImage_k4a.get_width_pixels();;
    int height = colorImage_k4a.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(height * width);

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, width, height, width * (int)sizeof(uint16_t));
    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, width, height, width * 3 * (int)sizeof(int16_t));
    transformation.depth_image_to_color_camera(depthImage_k4a, &transformed_depth_image);
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();
    uint8_t *color_image_data = colorImage_k4a.get_buffer();

    for(int i = 0; i < width * height; i++)
    {
        pcl::PointXYZRGB p;
        p.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        p.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        p.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if(p.z == 0) continue;

        p.b = color_image_data[4 * i + 0];
        p.g = color_image_data[4 * i + 1];
        p.r = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];
        if (p.b == 0 && p.g == 0 && p.r == 0 && alpha == 0) continue;

        cloud->points[i] = p;
    }

    return cloud;
}

void KINECT_BASE::KinectAzureDK_Source_Grabber(k4a::image &colorImage_k4a, k4a::image &depthImage_k4a, k4a::image &infraredImage_k4a, uint8_t timeout_ms, Type type)
{
    /*清空变量*/
    colorImage_k4a = depthImage_k4a = infraredImage_k4a = nullptr;

    /*检测捕获线程是否在工作*/
    if(Thread_Capture_Working){
        timeval capture_start_time;
        gettimeofday(&capture_start_time, NULL);
        while(1){
            bool *used = nullptr;
            switch(type){
                case Img:
                    used = &(Raw_Kinect_Data_.Used_Img);
                    break;
                case PointXYZRGB:
                    used = &(Raw_Kinect_Data_.Used_PointXYZRGB);
                    break;
                default:
                    break;
            }

            /*检测到新图片*/
            if(*used == false){
                pthread_mutex_lock(&Raw_Kinect_Data_.mutex_k4a_image_t);
                colorImage_k4a = Raw_Kinect_Data_.colorImage_k4a;
                depthImage_k4a = Raw_Kinect_Data_.depthImage_k4a;
                infraredImage_k4a = Raw_Kinect_Data_.infraredImage_k4a;
                *used = true;
                pthread_mutex_unlock(&Raw_Kinect_Data_.mutex_k4a_image_t);
                return;
            }
            /*超时检测*/
            timeval capture_end_time;
            gettimeofday(&capture_end_time, NULL);
            if(get_time_diff(capture_start_time, capture_end_time) > timeout_ms){
                printf("Kinect Azure DK Grabber Timeout!\n");
                return;
            }
        }

    }else {
        //如果捕获线程没在工作，则自己调用捕获
        k4a::capture capture;

        if (!Kinect.get_capture(&capture, std::chrono::milliseconds(timeout_ms))) {
            printf("KinectAzureDK Grabber Failed!\n");
            return;
        }

        colorImage_k4a = capture.get_color_image();
        if (colorImage_k4a == nullptr)
            printf("Failed To Get Color Image From Kinect!\n");

        depthImage_k4a = capture.get_depth_image();
        if (depthImage_k4a == nullptr)
            printf("Failed To Get Depth Image From Kinect!\n");

        infraredImage_k4a = capture.get_ir_image();
        if (infraredImage_k4a == nullptr)
            printf("Failed To Get IR Image From Kinect!\n");

        return;
    }
}

std::vector<cv::Mat> KINECT_BASE::getImg(uint8_t timeout_ms)
{
    cv::Mat colorImage_ocv, depthImage_ocv, infraredImage_ocv;

    //获取相机原始数据
    k4a::image colorImage_k4a = nullptr, depthImage_k4a = nullptr, infraredImage_k4a = nullptr;
    KinectAzureDK_Source_Grabber(colorImage_k4a, depthImage_k4a, infraredImage_k4a, timeout_ms, Img);

    /*数据格式转换*/
    if(colorImage_k4a != nullptr){
        colorImage_ocv = cv::Mat(colorImage_k4a.get_height_pixels(), colorImage_k4a.get_width_pixels(), CV_8UC4, colorImage_k4a.get_buffer());
        cvtColor(colorImage_ocv, colorImage_ocv, cv::COLOR_BGRA2BGR); // 从四通道转到三通道
    }

    if(depthImage_k4a != nullptr){
        depthImage_ocv = cv::Mat(depthImage_k4a.get_height_pixels(), depthImage_k4a.get_width_pixels(), CV_8UC4, depthImage_k4a.get_buffer());
        depthImage_ocv.convertTo(depthImage_ocv, CV_8U, 1);
    }

    if(infraredImage_k4a != nullptr){
        infraredImage_ocv = cv::Mat(infraredImage_k4a.get_height_pixels(), infraredImage_k4a.get_width_pixels(), CV_8UC4, infraredImage_k4a.get_buffer());
        infraredImage_ocv.convertTo(infraredImage_ocv, CV_8U, 1);
    }

    std::vector<cv::Mat> pictures;
    pictures.push_back(colorImage_ocv);
    pictures.push_back(depthImage_ocv);
    pictures.push_back(infraredImage_ocv);

    return pictures;
}

/*关闭相机*/
void KINECT_BASE::close()
{
    if(Kinect){
        if(Thread_Capture_Working){
            kill_thread_capture = true;
            usleep(1e5);
        }

        transformation.destroy();
        Kinect.stop_cameras();
        Kinect.close();

    }
}

/*计算时间差*/
double KINECT_BASE::get_time_diff(struct timeval start, struct timeval end)
{
    double diff = 0.0;
    diff = (end.tv_sec - start.tv_sec) * 1000.0 + (end.tv_usec - start.tv_usec) / 1000.0;
    if(diff < 0.0){
        diff *= -1;
    }
    return diff;
}

void *KINECT_BASE::thread_capture(void *arg)
{
    while(1){
        if(KINET_BASE_Ptr_->kill_thread_capture){
            break;
        }
        k4a::capture capture;
        if(!KINET_BASE_Ptr_->Kinect.get_capture(&capture, std::chrono::milliseconds(100))){
            continue;
        }
        k4a::image colorImage_k4a = capture.get_color_image();
        k4a::image depthImage_k4a = capture.get_depth_image();
        k4a::image infraredImage_k4a = capture.get_ir_image();

        pthread_mutex_lock(&KINET_BASE_Ptr_->Raw_Kinect_Data_.mutex_k4a_image_t);
        KINET_BASE_Ptr_->Raw_Kinect_Data_.colorImage_k4a = k4a::image(colorImage_k4a);
        KINET_BASE_Ptr_->Raw_Kinect_Data_.depthImage_k4a = k4a::image(depthImage_k4a);
        KINET_BASE_Ptr_->Raw_Kinect_Data_.infraredImage_k4a = k4a::image(infraredImage_k4a);
        KINET_BASE_Ptr_->Raw_Kinect_Data_.Used_Img = false;
        KINET_BASE_Ptr_->Raw_Kinect_Data_.Used_PointXYZRGB = false;
        pthread_mutex_unlock(&KINET_BASE_Ptr_->Raw_Kinect_Data_.mutex_k4a_image_t);
    }
}