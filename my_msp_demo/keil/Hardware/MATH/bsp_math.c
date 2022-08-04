#include "bsp_math.h"

/**
 * @brief  �������ֵ
 * @param  
 * @retval ���ֵ
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
 * @brief  ������Сֵ
 * @param  
 * @retval ��Сֵ
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
 * @brief  ����ƽ����
 * @param  
 * @retval ƽ����
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
 * @brief  ����ֵ ����
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
