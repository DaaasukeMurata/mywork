const int STEER_PIN = 8;
const int SPEED_PIN = 10;

extern String serial_input;
extern bool serial_in_complete;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(1000UL); 
  Serial.println("RC Test");

  tamiyaInit(STEER_PIN, SPEED_PIN);
}


void loop()
{
  int steer;
  int speed;

  int index;
  String str_tmp;

  if (serial_in_complete){

    // "90,0" の形式でserial_inputに入ってくる
    index = serial_input.indexOf(",");
    if (-1 == index){
      Serial.println("Ilegal param. format is [steer,speed] (ex. 90,-10)");
      Serial.println("  [steer range] from    0       to  90");
      Serial.println("  [speed range] from -100(back) to 100(front)");
    } else {
      steer = serial_input.toInt();
      str_tmp = serial_input.substring(++index);
      speed = str_tmp.toInt();

      printInt("steer = ", steer);
      printInt("speed = ", speed);
      tamiyaCtrl(steer, speed);
  
      Serial.println();      
    }

    serial_input = "";
    serial_in_complete = false;
  }
  
}

