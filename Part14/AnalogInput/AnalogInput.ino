
void setup()
{                
  Serial.begin(115200);
  analogReadResolution(13);
}

int pot1;
int pot2;

void loop()                     
{
  pot1 = analogRead(0);
  pot2 = analogRead(1);
  Serial.print("analog 0 is: ");
  Serial.print(pot1);
  Serial.print(", analog 1 is: ");
  Serial.println(pot2);
  delay(20);
}

