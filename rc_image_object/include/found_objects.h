#ifndef __FOUND_OBJECTS_H_INCLUDED__
#define __FOUND_OBJECTS_H_INCLUDED__

#include <std_msgs/Float32MultiArray.h>
#include "squaref.h"

enum FObjectPosition
{
  FOBJECT_LEFT,
  FOBJECT_CENTER,
  FOBJECT_RIGHT
};

// find_2d_objectのObjectIDと、四角
// TODO SquareFから継承
class FoundObject
{
public:
  FoundObject(int idVal,
              float width, float height,
              float tlX, float tlY,
              float trX, float trY,
              float blX, float blY,
              float brX, float brY);
  FObjectPosition getPosition(int displayWidth, int displayHeight, int threshold);
  void dbgPrint();

private:
  int id;
  float width;
  float height;
  SquareF square;
};

// 検出したObject全部
class ObjectList
{
public:
  int addObject(std_msgs::Float32MultiArray &array);
  std::size_t getObjNum();
  void dbgPrint();
  FoundObject getObject(int index);

private:
  std::vector<FoundObject> objList;
};

#endif