#include <Servo.h>
Servo steerServo;
Servo speedServo;

/*-----------------------------------------
 (IN )int steer_pin : steer制御に使うpin No
 (IN )int speed_pin : speed制御に使うpin No
-------------------------------------------*/
void tamiyaInit(int steer_pin, int speed_pin)
{
  //myservo.attach(port,正転最速指令パルス幅,逆転最速指令パルス幅)
  steerServo.attach(steer_pin, 1000, 2000);
  speedServo.attach(speed_pin, 1000, 2000);

  tamiyaCtrl(90,0);
}

/*-----------------------------------------
  [param]
    (IN )int speed : speed(-128 - 128)
    (IN )int steer : angle(0 - 180)
  [return]
    0:success, -1:err
-------------------------------------------*/
int tamiyaCtrl(int steer, int speed)
{
  int wk;
  int ret = 0;

  wk = steerCtrl(steer);
  if (wk < 0) ret = wk;
  
  wk = speedCtrl(speed);
  if (wk < 0) ret = wk;

  if (ret < 0) ret = -1;    // errは-1に丸め込む
  return ret;
}


/*-----------------------------------------
 (IN )int angle : angle(from 0 to 180)
-------------------------------------------*/
int steerCtrl(int angle)
{
  // 範囲チェック
  if ( (angle < 0) || (angle > 180) ){
    return -1;
  }
  
  steerServo.write(angle);
  return 0;
}

/*-----------------------------------------
 (IN )int speed : speed(from -100 to 100)
-------------------------------------------*/
int speedCtrl(int speed)
{
  int angle;
  
  // 範囲チェック
  if ( (speed < -100) || (speed > 100) ){
    return -1;
  }

  angle = 90 + ( ((float)speed / 128) * 90 );
//  printInt("speedangle = ",angle);

  speedServo.write(speed);
  return 0;
}

