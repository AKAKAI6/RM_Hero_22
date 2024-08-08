#ifndef USER_LIB_H
#define USER_LIB_H
#include "struct_typedef.h"

typedef __packed struct
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num[1];       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);

//限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);

#endif
