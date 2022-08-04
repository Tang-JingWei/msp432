#include "bsp_math.h"

/**
 * @brief  数组最大值
 * @param  
 * @retval 最大值
 */
float searchMax(uint32_t *arr, int size)
{
  float max = arr[0];

  for (int i = 1; i < size; i++)
  {
    if (max <= arr[i])
    {
      max = arr[i];
    }
  }

  return max;
}

/**
 * @brief  数组最小值
 * @param  
 * @retval 最小值
 */
float searchMin(uint32_t *arr, int size)
{
  float min = arr[0];

  for (int i = 1; i < size; i++)
  {
    if (min >= arr[i])
    {
      min = arr[i];
    }
  }

  return min;
}

/**
 * @brief  数组平均数
 * @param  
 * @retval 平均数
 */
float calAverage(uint32_t *arr, int size)
{
  float sum = 0, avg;

  for (int i = 0; i < size; i++)
  {
    sum += arr[i];
  }
  avg = sum / (float)size;

  return avg;
}

/**
 * @brief  绝对值 函数
 * @param  float dat
 * @retval float
 */
float Abs(float dat)
{
  if (dat < 0)
  {
    return dat * (-1);
  }
  else
  {
    return dat;
  }
}
