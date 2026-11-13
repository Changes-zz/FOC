#include "Clarke.h"

void Clarke::Calculate(float u, float v, float w, ClarkeOutput& output)
{
    output.alpha = (2.0f * u - v - w) / 3.0f;
    output.beta = (v - w) * 0.57735026919f;
}
