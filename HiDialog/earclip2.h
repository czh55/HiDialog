#ifndef EARCLIP2_H
#define EARCLIP2_H

#include <pcl/point_cloud.h>
#include <vector>
#include "holefilling.h"
#include "earclip.h"

class EarClip2
{
public:
    EarClip2();
    ~EarClip2();
    // ��������Ķ����
    void setInputPoly(POLYGON2 *pPoly);
    // �Զ���ν������ǻ�
    bool triangulatePoly();

private:

    EarClip m_ec;
    POLYGON m_poly;
    POLYGON2* m_poly2;
};

#endif // EARCLIP2_H
