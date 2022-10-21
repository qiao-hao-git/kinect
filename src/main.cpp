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
bool kill_thread_capture = false;

/* USER CODE END 0 *//**
  * @brief  The application entry point.
  * @retval int
  */
int main()
{
    /* Initialize all configured peripherals */
    pthread_mutex_t mutex_show = PTHREAD_MUTEX_INITIALIZER;
    /* USER CODE BEGIN Init */
    KINET_BASE kinta;
    kinta.init();
    kinta.start_Capture();
    //waitKey(0);
    /* USER CODE END Init */

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* Infinite loop */
    while (1){
        /* USER CODE BEGIN WHILE */
        std::vector<Mat> pictures = kinta.getImg();
        cv::Mat colorImage_ocv = pictures[0], depthImage_ocv = pictures[1], infraredImage_ocv = pictures[2];
        if(colorImage_ocv.cols * colorImage_ocv.rows != 0) imshow("RGB",colorImage_ocv);
        if(depthImage_ocv.cols * depthImage_ocv.rows != 0) imshow("Depth",depthImage_ocv);
        if(infraredImage_ocv.cols * infraredImage_ocv.rows != 0) imshow("Ir",infraredImage_ocv);

        waitKey(30);
        pthread_mutex_lock(&mutex_show);

        pthread_mutex_unlock(&mutex_show);
        /* USER CODE END WHILE */
    }

    kinta.close();
    /* USER CODE BEGIN 2 */
    return 0;
    /* USER CODE END 2 */
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

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