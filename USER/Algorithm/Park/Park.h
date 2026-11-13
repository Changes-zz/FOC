#ifndef PARK_H
#define PARK_H

/**
 * @brief 两相静止坐标系
 */
struct AlphaBeta
{
    float alpha;
    float beta;
};

/**
 * @brief 两相旋转坐标系
 */
struct DQ
{
    float d;
    float q;
};

class Park
{
public:

    /**
     * @brief Park 正变换：αβ -> dq
     *
     * @param input       αβ 输入
     * @param electrical_angle 电角度，单位 rad
     * @return DQ         dq 输出
     */
    static DQ Transform(const AlphaBeta& input,
                       float electrical_angle);

    /**
     * @brief Park 逆变换：dq -> αβ
     *
     * @param input       dq 输入
     * @param electrical_angle 电角度，单位 rad
     * @return AlphaBeta  αβ 输出
     */
    static AlphaBeta InverseTransform(const DQ& input,
                                      float electrical_angle);
};

#endif