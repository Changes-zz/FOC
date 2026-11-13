#include "Park.h"
#include <math.h>

/**
 * @brief Park 正变换：αβ -> dq
 */
DQ Park::Transform(const AlphaBeta& input,
                   float electrical_angle)
{
    DQ output;

    float sin_theta = sinf(electrical_angle);
    float cos_theta = cosf(electrical_angle);

    output.d = input.alpha * cos_theta
             + input.beta * sin_theta;

    output.q = -input.alpha * sin_theta
             + input.beta * cos_theta;

    return output;
}

/**
 * @brief Park 逆变换：dq -> αβ
 */
AlphaBeta Park::InverseTransform(const DQ& input,
                                 float electrical_angle)
{
    AlphaBeta output;

    float sin_theta = sinf(electrical_angle);
    float cos_theta = cosf(electrical_angle);

    output.alpha = input.d * cos_theta
                 - input.q * sin_theta;

    output.beta = input.d * sin_theta
                + input.q * cos_theta;

    return output;
}
