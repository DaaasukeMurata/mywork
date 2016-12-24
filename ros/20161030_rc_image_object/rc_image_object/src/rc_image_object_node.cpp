#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>

#include "found_objects.h"
#include "teleop_rc.h"

class TrackingObjRc
{
  public:
    TrackingObjRc();

  private:
    ros::NodeHandle nh;
    ros::Subscriber objects_sub;
    TeleopRc teleoprc;
    void objCb(const std_msgs::Float32MultiArray &msg);
};

TrackingObjRc::TrackingObjRc()
{
    // find_2d_objectから配信される、/objectsの購読
    objects_sub = nh.subscribe("objects", 10, &TrackingObjRc::objCb, this);
}

void TrackingObjRc::objCb(const std_msgs::Float32MultiArray &msg)
{

    ROS_INFO("TrackingOjbRc::objCb() start");

    ObjectList objList;
    std_msgs::Float32MultiArray wkArray = msg;

    if (wkArray.data.size())
    {
        // Object登録
        objList.addObject(wkArray);
        objList.dbgPrint();

        // move
        FObjectPosition pos;
        pos = objList.getObject(0).getPosition(640, 480, 100);
        switch (pos)
        {
        case FOBJECT_CENTER:
            teleoprc.move(TELEOP_RC_FRONT);
            break;
        case FOBJECT_LEFT:
            teleoprc.move(TELEOP_RC_LEFT);
            break;
        case FOBJECT_RIGHT:
            teleoprc.move(TELEOP_RC_RIGHT);
            break;
        }
    }
    else
    {
        ROS_INFO("No Object Found");
        teleoprc.stop();
    }

    return;
}

int main(int argc, char **argv)
{
    ROS_INFO("start rc_image_object_node");

    // node名 = rc_image_node
    ros::init(argc, argv, "rc_image_object_node");

    TrackingObjRc trackingObjrc;

    ros::spin();
    return 0;
}