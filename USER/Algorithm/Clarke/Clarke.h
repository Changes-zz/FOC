#ifndef CLARKE_H
#define CLARKE_H

struct ClarkeOutput
{
    float alpha;
    float beta;
};

class Clarke
{
public:
    // 幅值不变的三相到 αβ 变换；输出不包含零序分量
    void Calculate(float u, float v, float w, ClarkeOutput& output);
};

#endif
