/*
 * qpid.h
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-09-09     qiyongzhong       first version
 */

#ifndef __QPID_H__
#define __QPID_H__

//#define QPID_USING_TEST         //使用测试

typedef struct{
    float dst;
    float kp;
    float ki;
    float kd;
    float ki_max;
    float ki_min;
    float out_min;
    float out_max;
    float err[3];
}qpid_t;

void qpid_init(qpid_t *qpid);//初始化pid
void qpid_set_dst(qpid_t *qpid, float dst);//设置目标值
void qpid_set_ratio(qpid_t *qpid, float kp, float ki, float kd);//设置各项比例
void qpid_set_ki_lmt(qpid_t *qpid, float ki_max, float ki_min);//设置积分限值
void qpid_set_out_lmt(qpid_t *qpid, float out_min, float out_max);//设置输出限值
float qpid_cal_inc(qpid_t *qpid, float cur);//计算增量型pid, 输出增量值
float qpid_cal_pos(qpid_t *qpid, float cur);//计算位置型pid, 输出位置值

#endif

