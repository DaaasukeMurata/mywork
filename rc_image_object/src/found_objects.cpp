#include <stdio.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include "squaref.h"
#include "found_objects.h"

FoundObject::FoundObject(int idVal,
                         float width, float height,
                         float tlX, float tlY,
                         float trX, float trY,
                         float blX, float blY,
                         float brX, float brY)
{
    this->id = idVal;
    this->width = width;
    this->height = height;
    SquareF tmpSquareF(tlX, tlY,
                       trX, trY,
                       blX, blY,
                       brX, brY);
    this->square = tmpSquareF;
}

/* displayの真ん中から、thresholdの間はcenter */
FObjectPosition FoundObject::getPosition(int displayWidth, int displayHeight, int threshold)
{
    PointF wkPoint;
    wkPoint = this->square.getCenter();

    if ((int)(displayWidth / 2) - threshold > (int)wkPoint.x)
    {
        return FOBJECT_LEFT;
    }
    else if ((int)(displayWidth / 2) + threshold < (int)wkPoint.x)
    {
        return FOBJECT_RIGHT;
    }
    else
    {
        return FOBJECT_CENTER;
    }

    return FOBJECT_CENTER;
}

void FoundObject::dbgPrint()
{
    printf("[FoundObject] ID is %d\n", id);
    printf("[FoundObject] width is %f\n", width);
    printf("[FoundObject] height is %f\n", height);
    this->square.dbgPrint();
}

/* ---for ObjectList class--- */

int ObjectList::addObject(std_msgs::Float32MultiArray &array)
{
    int num = 0;

    if (array.data.size())
    {
        for (unsigned int i = 0; i < array.data.size(); i += 12)
        {
            // get data
            int id = (int)array.data[i];
            float objectWidth = array.data[i + 1];
            float objectHeight = array.data[i + 2];

            // Find corners OpenCV
            cv::Mat cvHomography(3, 3, CV_32F);
            cvHomography.at<float>(0, 0) = array.data[i + 3];
            cvHomography.at<float>(1, 0) = array.data[i + 4];
            cvHomography.at<float>(2, 0) = array.data[i + 5];
            cvHomography.at<float>(0, 1) = array.data[i + 6];
            cvHomography.at<float>(1, 1) = array.data[i + 7];
            cvHomography.at<float>(2, 1) = array.data[i + 8];
            cvHomography.at<float>(0, 2) = array.data[i + 9];
            cvHomography.at<float>(1, 2) = array.data[i + 10];
            cvHomography.at<float>(2, 2) = array.data[i + 11];
            std::vector<cv::Point2f> inPts, outPts;
            inPts.push_back(cv::Point2f(0, 0));
            inPts.push_back(cv::Point2f(objectWidth, 0));
            inPts.push_back(cv::Point2f(0, objectHeight));
            inPts.push_back(cv::Point2f(objectWidth, objectHeight));
            cv::perspectiveTransform(inPts, outPts, cvHomography);

            FoundObject obj(id,
                            objectWidth, objectHeight,
                            outPts.at(0).x, outPts.at(0).y,
                            outPts.at(1).x, outPts.at(1).y,
                            outPts.at(2).x, outPts.at(2).y,
                            outPts.at(3).x, outPts.at(3).y);

            this->objList.push_back(obj);
            num++;
        }
    }
    else
    {
        printf("No Object found.\n");
    }

    return num;
}

std::size_t ObjectList::getObjNum()
{
    return this->objList.size();
}

FoundObject ObjectList::getObject(int index)
{
    return this->objList[index];
}

void ObjectList::dbgPrint()
{
    std::size_t num;

    printf("[ObjectList] Number of Objects is %lu\n", num);

    num = this->objList.size();
    for (std::size_t i = 0; i < num; i++)
    {
        this->objList[i].dbgPrint();
    }
}
