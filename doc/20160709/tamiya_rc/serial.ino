String serial_input = "";
bool serial_in_complete = false;

void serialEvent()
{ 
  char inchar;
  
  while( Serial.available() )
  {
    inchar = (char)Serial.read();
    serial_input += inchar;

    // CRのみ or LFのみ どちらかに対応
    if (inchar == '\r' || inchar == '\n'){
      serial_in_complete = true;
    }    
  }
}

void printInt(String str, int val)
{
  Serial.print(str);
  Serial.print(val);
  Serial.println();
}

void printString(String str, String val)
{
  Serial.print(str);
  Serial.print(val);
  Serial.println();
}


