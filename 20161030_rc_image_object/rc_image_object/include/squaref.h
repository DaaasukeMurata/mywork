#ifndef __SQUAREF_H_INCLUDED__
#define __SQUAREF_H_INCLUDED__

#include <std_msgs/Float32MultiArray.h>

// 座標
typedef struct
{
  float x;
  float y;
} PointF;

// 四角。頂点の座標を持つ
class SquareF
{
public:
  SquareF() {}

  SquareF(float tlX, float tlY,
          float trX, float trY,
          float blX, float blY,
          float brX, float brY);
  PointF getCenter();
  void dbgPrint();

private:
  PointF topLeft, topRight, bottomLeft, bottomRight;
};
#endif