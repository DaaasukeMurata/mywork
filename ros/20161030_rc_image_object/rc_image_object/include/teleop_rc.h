#ifndef __TELEOP_RC_H_INCLUDED__
#define __TELEOP_RC_H_INCLUDED__

enum TeleopRcDirection
{
    TELEOP_RC_FRONT = 0,
    TELEOP_RC_RIGHT,
    TELEOP_RC_LEFT
};

class TeleopRc
{
  public:
    TeleopRc();
    void move(TeleopRcDirection direction);
    void stop();

  private:
    ros::NodeHandle nh;
    int l_scale_, a_scale_;
    ros::Publisher vel_pub_;
};

#endif