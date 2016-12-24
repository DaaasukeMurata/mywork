#include <stdio.h>
#include "squaref.h"

SquareF::SquareF(float tlX, float tlY,
                 float trX, float trY,
                 float blX, float blY,
                 float brX, float brY)
{
    topLeft.x = tlX;
    topLeft.y = tlY;
    topRight.x = trX;
    topRight.y = trY;
    bottomLeft.x = blX;
    bottomLeft.y = blY;
    bottomRight.x = brX;
    bottomRight.y = brY;
}

PointF SquareF::getCenter()
{
    PointF point;

    /* 左上と右下の真ん中の座標とする */
    point.x = (this->topLeft.x + this->bottomRight.x) / 2;
    point.y = (this->topLeft.y + this->bottomRight.y) / 2;

    return point;
}

void SquareF::dbgPrint()
{
    printf("[SquareF] topLeft     :x = %f, y = %f\n", topLeft.x, topLeft.y);
    printf("[SquareF] topRight    :x = %f, y = %f\n", topRight.x, topRight.y);
    printf("[SquareF] bottomLeft  :x = %f, y = %f\n", bottomLeft.x, bottomLeft.y);
    printf("[SquareF] bottomRight :x = %f, y = %f\n", bottomRight.x, bottomRight.y);
}
