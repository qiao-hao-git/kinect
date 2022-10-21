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
#include "main.h"
#include <time.h>
#include <signal.h>
#include "kinet_config.h"

bool ctrl_c_pressed=false;

void ctrlc(int){
    ctrl_c_pressed = true;
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

pthread_mutex_t mutex_show = PTHREAD_MUTEX_INITIALIZER; // 互斥锁

/* USER CODE END 0 *//**
  * @brief  The application entry point.
  * @retval int
  */
int main()
{
    /*ctrl c 中断*/
    signal(SIGINT, ctrlc);
    /* Initialize all configured peripherals */
    // KINET_BASE* KINET_BASE_Ptr_1= nullptr;
    /* USER CODE BEGIN Init */
    KINECT_BASE kinta;
    kinta.init();
    kinta.start_Capture();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer_PCL"));

    //设置默认的坐标系
    viewer->addCoordinateSystem(1.0);
    //设置固定的元素。红色是X轴，绿色是Y轴，蓝色是Z
    viewer->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(10, 0, 0), "x");
    viewer->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 5, 0), "y");
    viewer->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 2), "z");


    while (1){
        if(ctrl_c_pressed == true)
            break;
        /* USER CODE BEGIN WHILE */

        std::vector<Mat> pictures = kinta.getImg();
        cv::Mat colorImage_ocv = pictures[0], depthImage_ocv = pictures[1], infraredImage_ocv = pictures[2];
        if(colorImage_ocv.cols * colorImage_ocv.rows != 0) imshow("RGB",colorImage_ocv);
        if(depthImage_ocv.cols * depthImage_ocv.rows != 0) imshow("Depth",depthImage_ocv);
        if(infraredImage_ocv.cols * infraredImage_ocv.rows != 0) imshow("Ir",infraredImage_ocv);
        printf("123\n");

        waitKey(20);

        PointCloud<PointXYZRGB>::Ptr cloud = kinta.getPointXYZRGB();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr res_cloud_rgb_show(new pcl::PointCloud<pcl::PointXYZRGB>);

        viewer->addPointCloud(cloud, "cloud");
        viewer->spinOnce(3);
        viewer->removePointCloud("cloud");
    }

    kinta.close();

    return 0;
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER ASSERT BEGIN */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER ASSERT END */
}
#endif /* USE_FULL_ASSERT */