
## 使い方

- tamiya_rc.ino, serial.ino, servo.inoを同じディレクトリに置いて、ArduinoIDEから開いてください。

- シリアルモニタの改行は、"LFのみ"または、"CRのみ"に設定（defaultは"改行なし"になっているので変更要）

- シリアルから"steer,speed"の形式で入力（例："90,-10"）
  - steer : 0〜180
  - speed : -100〜100

## PWM出力PINの変更

tamiya_rc.ino

    const int STEER_PIN = 8;
    const int SPEED_PIN = 10;



## API

void setup()内に記述

    /*-----------------------------------------
    (IN )int steer_pin : steer制御に使うpin No
    (IN )int speed_pin : speed制御に使うpin No
    -------------------------------------------*/
    void tamiyaInit(int steer_pin, int speed_pin)

モータ制御（実機で確認忘れました。前後左右逆かも。）

    /*-----------------------------------------
    [param]
        (IN )int speed : speed(-100 - 100)
        (IN )int steer : angle(0 - 180)
    [return]
        0:ok, -1:err
    -------------------------------------------*/
    int tamiyaCtrl(int steer, int speed)


