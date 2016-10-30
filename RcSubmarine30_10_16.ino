#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Wire.h>
#include <LSM6.h>

#define ZERO 0
#define PWM_MAX 255

LSM6 imu;

char report[80];

byte mac[] = {
  0x90, 0xA2, 0xDA, 0x0D, 0xA3, 0x12
};
IPAddress ip(192, 168, 1, 177);
EthernetServer server(80);
unsigned int localPort = 8888;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
EthernetUDP Udp;

int encoder1 = 18;
int encoder2 = 19;
int encoder3 = 2;
int encoder4 = 3;

//M1 counters
static int counterCW;
static int counterCW_p;
volatile int counterCCW;
volatile int counterCCW_p;

//M2 counters
int counterCWM2;
int counterCW_pM2;
int counterCCWM2;
int counterCCW_pM2;

// Sensor's for JoyStick
int sensorX = A10;
int sensorY = A11;

//int pumpVoltage = A2;
int pumpB = 13;

int refSpeedM1 = 6; // pin that is used for setpointing Motor1 reference speed
int refSpeedM2 = 5;

// Variables for counting time
volatile unsigned long start;
volatile unsigned long finish;
volatile int dif;
volatile int difM2;

int motorSpeed = 0; // Variable that is used to count motor speed in rpm
int motorSpeedM2 = 0;

// Variables used for flaging directions (CW or CCW)
boolean goingUp = false;
boolean goingDown = false;
boolean goingUpM2 = false;
boolean goingDownM2 = false;

boolean pumpFlag = false;
// Flag used for counting motor speed, it's deterined by motor direction.
boolean difFlag = false;
boolean difFlagM2 = false;

boolean regulatorFlag = false; //flaga zalaczajaca regulator
boolean u_flag = false; //flaga wewnętrzna regulatora

int M1_A = 34; // Motor M1 at pin 10 and 11
int M1_B = 35;
int M2_A = 31; // Motor M2 at pin 9 and 8
int M2_B = 30;

int pumpOn = 7;

int timeDif = 0;
int startDif = 0;
int testUdp = 0;

/*Testowanie dzialania UDP, potrzebna globalna wartosc zadana dla silnikow */
int omegaRefForward = 0;
int omegaRefBackward = 0;
int omegaRefRight = 0;
int omegaRefLeft = 0;

//SEKCJA SILNIKA KROKOWEGO
const int stepperMotorDirection = 22;
const int stepperMotorStep = 29;
const int stepperMotorAngleLimit = 22; // Jeden krok to 1.8st. 40/1.8 = ~22
const int stepperMotorSleep = 53;

int stepperCounter = 0;
boolean stepperFlag = true; // True dla "Up"
///////////////////////////////////

String motorSpeedUDP;
char udpArray[30];

unsigned long s = 0;
unsigned long f = 0;

void setup()
{
  pinMode(stepperMotorDirection, OUTPUT);
  pinMode(stepperMotorStep, OUTPUT);
  digitalWrite(stepperMotorStep, LOW);
  digitalWrite(stepperMotorDirection, LOW);
  pinMode(stepperMotorSleep, OUTPUT);
  digitalWrite(stepperMotorSleep, LOW);


  pinMode(pumpB, OUTPUT);
  digitalWrite(pumpB, LOW);

  //  pinMode(pumpVoltage, OUTPUT);
  //  analogWrite(pumpVoltage, 0);

  //Initialization of pin's
  pinMode(M1_A, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(M2_A, OUTPUT);
  pinMode(M2_B, OUTPUT);

  pinMode(refSpeedM1, OUTPUT);
  pinMode(refSpeedM2, OUTPUT);

  counterCW = 0;      //Liczniki enkoderów
  counterCCW = 0;
  counterCWM2 = 0;
  counterCCWM2 = 0;
  counterCW_p = 0;
  counterCCW_p = 0;
  counterCW_pM2 = 0;
  counterCCW_pM2 = 0;

  //Serial prints for debugging and testing
  //Serial.begin(19200);

  /* Setup encoder pins as inputs */
  pinMode(encoder1, INPUT);
  pinMode(encoder2, INPUT);
  pinMode(encoder3, INPUT);
  pinMode(encoder4, INPUT);


  pinMode(pumpOn, INPUT); // przycisk wlaczenia pompy
  digitalWrite(pumpOn, LOW);

  // encoder pin on interrupt 0,3
  attachInterrupt(4, Decoder, FALLING);
  attachInterrupt(1, DecoderM2, FALLING);

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  server.begin();
  Wire.begin();
  imu.init();
  imu.enableDefault();

  start = millis();
}

//**************************************************

void loop()
{
  GetMessage();
  MotorControlWithRegulation();
  EncoderControl();
  PrintToWebserver();
}

void PrintToWebserver()
{
  //imu.read();   //czujnik a,g
  EthernetClient client;
  client = server.available();
  boolean currentLineIsBlank = true;
  if (client) {
    while (client.connected()) {
      //char c = client.read();

      //if (c == '\n' && currentLineIsBlank) {
      // send a standard http response header
      client.println("HTTP/1.1 200 OK");
      // client.println("Content-Type: text/html");
      client.println("Connection: close");  // the connection will be closed after completion of the response
      client.println("Refresh: 0.75");  // refresh the page automatically every 5 sec
      client.println();

      client.println(motorSpeed);
      client.println(motorSpeedM2);
      client.println(stepperCounter);
      /* client.print("GyroX = ");
        client.println(imu.g.x);
        client.print("GyroY = ");
        client.println(imu.g.y);
        client.print("GyroZ = ");
        client.println(imu.g.z); */
      break;

    }
    client.stop();
    Ethernet.maintain();
  }
}
//******************************FUNCTIONS*******************************************************
String ConvertToString (int m)
{
  m = m + 1;
  return (String)m;
}
void Decoder()
{
  if (digitalRead(encoder1) == digitalRead(encoder2))
  {
    goingUp = 1; //if encoder channels are the same, direction is CW
  }
  else
  {
    goingDown = 1; //if they are not the same, direction is CCW
  }
}
void DecoderM2()
{
  if (digitalRead(encoder3) == digitalRead(encoder4))
  {
    goingUpM2 = 1; //if encoder channels are the same, direction is CW
  }
  else
  {
    goingDownM2 = 1; //if they are not the same, direction is CCW
  }
}

void EncoderControl()
{
  //start = millis();

  if (millis() >= start + 1000) //(start - finish >= 1000)
  {
    // Serial.println(millis());
    // Serial.println(start);
    start = millis();
    if (difFlag == false)
    {
      dif = (counterCW - counterCW_p);
      counterCW_p = counterCW;

      if (difFlagM2 == false)
      {
        difM2 = (counterCWM2 - counterCW_pM2);
        counterCW_pM2 = counterCWM2;
      }
      else if (difFlagM2 == true)
      {
        difM2 = (counterCCWM2 - counterCCW_pM2);
        counterCCW_pM2 = (counterCCWM2);
      }
    }
    else if (difFlag == true)
    {
      dif = (counterCCW - counterCCW_p);
      counterCCW_p = (counterCCW);
      if (difFlagM2 == false)
      {
        difM2 = (counterCWM2 - counterCW_pM2);
        counterCW_pM2 = (counterCWM2);
      }
      else if (difFlagM2 == true)
      {
        difM2 = (counterCCWM2 - counterCCW_pM2);
        counterCCW_pM2 = (counterCCWM2);
      }
    }
    // finish = start;
    motorSpeed = (int)(dif / 5); //300.0 * 60.0; // Obliczanie prędkości silników w RPM.
    motorSpeedM2 = (int)(difM2 / 5); // 300.0 * 60.0;
  }
}

void PumpControl()
{
  if (digitalRead(pumpOn) == HIGH)
  {
    pumpFlag = true;
  }
  else
    pumpFlag = false;


  if (pumpFlag == true)
  {
    digitalWrite(pumpB, HIGH);
    // analogWrite(pumpVoltage, 255);
  }
  else
  {
    digitalWrite(pumpB, LOW);
    //    analogWrite(pumpVoltage, 0);
  }
}



/* TESTOWANIE FUNKCJI PREDKOSCI SILNIKA Z REGULATOREM


*/
void MotorControlWithRegulation()
{

  /*Odczyt wartości z joysticka analogowego podlaczonego do plytki stykowej,
     Aktualnie nieaktywne ze wzgledu na sterowanie z joysticka XBOX360
   *  */
  //  int  getJoyX = analogRead(sensorX);   //odczyt z sensora analogowego osi X (0,1023)
  // int  getJoyY = analogRead(sensorY);   //odczyt z sensora analogowego osi Y (0, 1023)

  /*Kierunek przód - tył */


  // int omegaRefForward = map(getJoyX, 531, 1023, ZERO, PWM_MAX);

  // int omegaRefBackward = map(getJoyX, 531, 0, ZERO, PWM_MAX);
  /* Kierunek lewo - prawo */

  //Jesli pozycja joysticka wynosi 531, to wartość zadana wynosi 0,
  //jesli pozycja joysticka jest rowna 1023, to PWM = 255
  // int omegaRefRight = map(getJoyY, 505, 1023, ZERO, PWM_MAX);
  //Jesli pozycja joysticka wynosi 531, to wartość zadana wynosi 0,
  //jesli pozycja joysticka jest rowna 1023, to PWM = 255
  // int omegaRefLeft = map(getJoyY, 505, ZERO, ZERO, PWM_MAX);


  int m1TurnTemp = 0; //zmienne pomocnicze do skretu
  int m2TurnTemp = 0;
  const int turnConst = 7; //zmienna odejmowana od wartosci jednego z silnikow przy skrecaniu
  //sprawdzamy, czy kierunek jest do przodu
  if (omegaRefForward > ZERO && (omegaRefRight <= 15 && omegaRefLeft <= 15))
  {
    //okreslenie pinów dla kierunku do przodu
    digitalWrite(M1_A, HIGH);
    digitalWrite(M2_A, HIGH);
    digitalWrite(M1_B, LOW);
    digitalWrite(M2_B, LOW);

    omegaRefLeft = 0;
    omegaRefRight = 0;

    //ustawienie wartości początkowych silnika i załączenie regulatora
    if (!regulatorFlag)
    {
      analogWrite(refSpeedM1, omegaRefForward);
      analogWrite(refSpeedM2, omegaRefForward);
      regulatorFlag = true;
    }
    else
    {
      RegulatorBeta(omegaRefForward); // przekazuje regulatorowi aktualna wartosc zadana
    }
  }
  else if (omegaRefBackward > ZERO && (omegaRefRight <= 15 && omegaRefLeft <= 15))
  {
    //okreslenie pinów dla kierunku do tyłu
    digitalWrite(M1_A, LOW);
    digitalWrite(M2_A, LOW);
    digitalWrite(M1_B, HIGH);
    digitalWrite(M2_B, HIGH);

    omegaRefLeft = 0;
    omegaRefRight = 0;

    /*  Serial.print("OmegarefBackward = ");
      Serial.println(omegaRefBackward); */

    if (!regulatorFlag)
    {
      analogWrite(refSpeedM1, omegaRefBackward);
      analogWrite(refSpeedM2, omegaRefBackward);
      regulatorFlag = true;
    }
    else
    {
      RegulatorBeta(omegaRefBackward); // przekazuje regulatorowi aktualna wartosc zadana
    }
  }
  else if (omegaRefRight > ZERO && (omegaRefForward <= 15 && omegaRefBackward <= 15)) //maksymalny obrót w prawo
  { //zakladamy ze M1 to silnik prawy, M2 to silnik lewy
    digitalWrite(M2_A, HIGH);
    digitalWrite(M1_A, LOW);          //czyli lewy silnik kreci sie do przodu,
    digitalWrite(M1_B, HIGH);        //a prawy do tylu
    digitalWrite(M2_B, LOW);

    omegaRefBackward = 0;
    omegaRefForward = 0;
    omegaRefLeft = omegaRefRight;

    analogWrite(refSpeedM1, omegaRefRight);
    analogWrite(refSpeedM2, omegaRefLeft);
  }
  else if (omegaRefLeft > ZERO && (omegaRefForward <= 15 && omegaRefBackward <= 15))  //maksymalny obrót w lewo
  {
    digitalWrite(M2_A, LOW);
    digitalWrite(M1_A, HIGH);          //czyli prawy silnik kreci sie do przodu,
    digitalWrite(M1_B, LOW);        //a lewy do tylu
    digitalWrite(M2_B, HIGH);

    omegaRefBackward = 0;
    omegaRefForward = 0;
    omegaRefRight = omegaRefLeft;

    analogWrite(refSpeedM1, omegaRefRight);
    analogWrite(refSpeedM2, omegaRefLeft);
  }
  else if (omegaRefForward > ZERO && omegaRefRight > 15)    //prawo + przod
  {
    digitalWrite(M1_A, HIGH);
    digitalWrite(M2_A, HIGH);
    digitalWrite(M1_B, LOW);
    digitalWrite(M2_B, LOW);

    omegaRefBackward = 0;

    m1TurnTemp = ((omegaRefForward + omegaRefRight) / 2) - turnConst; //PWM silnik prawy
    m2TurnTemp = ((omegaRefForward + omegaRefRight) / 2);              //PWM silnik lewy

    analogWrite(refSpeedM1, m1TurnTemp);
    analogWrite(refSpeedM2, m2TurnTemp);
  }
  else if (omegaRefForward > ZERO && omegaRefLeft > 15) // lewo + przod
  {
    digitalWrite(M1_A, HIGH);
    digitalWrite(M2_A, HIGH);
    digitalWrite(M1_B, LOW);
    digitalWrite(M2_B, LOW);

    m1TurnTemp = ((omegaRefForward + omegaRefRight) / 2);             //PWM silnik prawy
    m2TurnTemp = ((omegaRefForward + omegaRefRight) / 2) - turnConst;  //PWM silnik lewy

    omegaRefBackward = 0;

    analogWrite(refSpeedM1, m1TurnTemp);
    analogWrite(refSpeedM2, m2TurnTemp);
  }
  else if (omegaRefBackward > ZERO && omegaRefRight > 15) //prawo + tyl
  {
    digitalWrite(M1_A, LOW);
    digitalWrite(M2_A, LOW);
    digitalWrite(M1_B, HIGH);
    digitalWrite(M2_B, HIGH);

    m1TurnTemp = ((omegaRefForward + omegaRefRight) / 2) - turnConst;    //PWM silnik prawy
    m2TurnTemp = ((omegaRefForward + omegaRefRight) / 2);                //PWM silnik lewy

    omegaRefBackward = 0;

    analogWrite(refSpeedM1, m1TurnTemp);
    analogWrite(refSpeedM2, m2TurnTemp);
  }
  else if (omegaRefBackward > ZERO && omegaRefLeft > 15) //lewo + tyl
  {
    digitalWrite(M1_A, LOW);
    digitalWrite(M2_A, LOW);
    digitalWrite(M1_B, HIGH);
    digitalWrite(M2_B, HIGH);

    m1TurnTemp = ((omegaRefForward + omegaRefRight) / 2);             //PWM silnik prawy
    m2TurnTemp = ((omegaRefForward + omegaRefRight) / 2) - turnConst; //PWM silnik lewy

    omegaRefBackward = 0;

    analogWrite(refSpeedM1, m1TurnTemp);
    analogWrite(refSpeedM2, m2TurnTemp);
  }
  else // STOP
  {
    digitalWrite(M1_A, LOW);
    digitalWrite(M2_A, LOW);
    digitalWrite(M1_B, LOW);
    digitalWrite(M2_B, LOW);
    analogWrite(refSpeedM1, ZERO);
    analogWrite(refSpeedM2, ZERO);
  }

  //using while statement to stay in the loop for continuous
  //interrupts
  while (goingUp == 1) // CW motion in the rotary encoder
  {
    goingUp = 0; // Reset the flag
    counterCW ++;
    difFlag = false;
  }
  while (goingDown == 1) // CCW motion in rotary encoder
  {
    goingDown = 0; // clear the flag
    counterCCW ++;
    difFlag = true;
  }
  //interrups for M2
  while (goingUpM2 == 1) // CW motion in the rotary encoder
  {
    goingUpM2 = 0; // Reset the flag
    counterCWM2 ++;
    difFlagM2 = false;
  }
  while (goingDownM2 == 1) // CCW motion in rotary encoder
  {
    goingDownM2 = 0; // clear the flag
    counterCCWM2 ++;
    difFlagM2 = true;
  }
}

void RegulatorAlpha (int omegaRef)
{
  static int u1;  //volatile ze wzgledu na przerwania
  static int u2;
  if (!u_flag)
  {
    u1 = omegaRef;
    u2 = omegaRef;
    u_flag = true;
  }
  else
    u_flag = false;

  if (u_flag)
  {
    if (motorSpeed > motorSpeedM2)
    {
      u1 -= 10;
      analogWrite(refSpeedM1, u1);
      analogWrite(refSpeedM2, u2);
      u_flag = false;
    }
    else if (motorSpeed < motorSpeedM2)
    {
      u2 -= 10;
      analogWrite(refSpeedM1, u1);   //niezmienione u1 = omegaRef
      analogWrite(refSpeedM2, u2);  //zmniejszone u2 o jakas wartosc
    }
    else //jakby jednak byly rowne
    {
      analogWrite(refSpeedM1, omegaRef);
      analogWrite(refSpeedM2, omegaRef);
      regulatorFlag = false;      //wylaczamy regulator, sprawdzamy czy
      u_flag = false;
    }
  }
  else    //coś jeszcze można zrobić, jeśli nie true?
    u_flag = false;
}

void RegulatorBeta (int omegaRef)
{
  static int u1;  //volatile ze wzgledu na przerwania
  static int u2;
  static int omegaRefPrevious = 0;

  omegaRefPrevious = omegaRef;

  if (!u_flag)
  {
    u1 = omegaRef;
    u2 = omegaRef;
    u_flag = true;
  }
  else
  {
    u_flag = false;
    if (motorSpeed > motorSpeedM2)
    {
      u1 -= 3;
      analogWrite(refSpeedM1, u1);
      analogWrite(refSpeedM2, u2);
      u_flag = false;
    }
    else if (motorSpeed < motorSpeedM2)
    {
      u2 -= 3;
      analogWrite(refSpeedM1, u1);   //niezmienione u1 = omegaRef
      analogWrite(refSpeedM2, u2);  //zmniejszone u2 o jakas wartosc

    }
    else //jakby jednak byly rowne
    {
      analogWrite(refSpeedM1, u1); //zamiast u1 bylo omegaref
      analogWrite(refSpeedM2, u2);
      regulatorFlag = false;      //wylaczamy regulator, sprawdzamy czy
      //u_flag = false;
    }
  }
  if (abs(omegaRef - omegaRefPrevious) > 0)
  {
    u_flag = false;
  }
}
void SendMessage()
{
  Udp.beginPacket(Udp.remoteIP(), localPort);
  motorSpeedUDP = (String)motorSpeed + "a" + (String)motorSpeedM2 + "a" + (String)stepperCounter + "a";
  int len = motorSpeedUDP.length();
  motorSpeedUDP.toCharArray(udpArray, len);

  Udp.write(udpArray);
  Udp.endPacket();
}

void GetMessage()
{

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    for (int i = 0; i < UDP_TX_PACKET_MAX_SIZE; i++)
      packetBuffer[i] = 0; //czyscimy bufor
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE); //czytamy bufor

    char dataIn[packetSize];

    for (int i = 0; i < packetSize; i++)
    {
      dataIn[i] = packetBuffer[i];
    }

    String data = dataIn;
    if (data.substring(0, 0) != "0")
    {
      int tmp = 0;
      int tmp2 = 0;
      if (data.substring(0, 1) == "1") {
        if (packetSize <= 4) {
          tmp = data.substring(1, 4).toInt();
          if (tmp > ZERO && tmp <= PWM_MAX)
          {
            omegaRefForward = tmp;
            omegaRefBackward = 0;
            omegaRefLeft = 0;
            omegaRefRight = 0;
          }
        }
        else if (packetSize == 6) 
        {
          tmp = data.substring(1, 4).toInt();
          if (tmp > ZERO && tmp <= PWM_MAX)
          {
            omegaRefForward = tmp;
            omegaRefBackward = 0;
            omegaRefLeft = 0;
            omegaRefRight = 0;
            if (stepperCounter >= 80)
              digitalWrite(stepperMotorSleep, LOW);
            else {
              digitalWrite(stepperMotorSleep, HIGH);
              digitalWrite(stepperMotorDirection, HIGH);
              digitalWrite(stepperMotorStep, HIGH);
              delay(3);
              digitalWrite(stepperMotorStep, LOW);
              delay(3);
              stepperCounter++;
            }
          }
        }
        else if (packetSize == 5)
        {
          tmp = data.substring(1, 3).toInt();
          if (tmp > ZERO && tmp <= PWM_MAX)
          {
            omegaRefForward = tmp;
            omegaRefBackward = 0;
            omegaRefLeft = 0;
            omegaRefRight = 0;
            if (stepperCounter >= 80)
              digitalWrite(stepperMotorSleep, LOW);
            else {
              digitalWrite(stepperMotorSleep, HIGH);
              digitalWrite(stepperMotorDirection, HIGH);
              digitalWrite(stepperMotorStep, HIGH);
              delay(3);
              digitalWrite(stepperMotorStep, LOW);
              delay(3);
              stepperCounter++;
            }
          }
        }
        else if (packetSize == 8)
        {
          tmp = data.substring(1, 4).toInt();
          if (tmp > ZERO && tmp <= PWM_MAX)
          {
            omegaRefForward = tmp;
            omegaRefBackward = 0;
            omegaRefLeft = 0;
            omegaRefRight = 0;
            if (stepperCounter >= 80)
              digitalWrite(stepperMotorSleep, LOW);
            else {
              stepperFlag = true;
              digitalWrite(stepperMotorSleep, HIGH);
              digitalWrite(stepperMotorDirection, LOW);
              digitalWrite(stepperMotorStep, HIGH);
              delay(3);
              digitalWrite(stepperMotorStep, LOW);
              delay(3);
              stepperCounter++;
            }
          }
        }
        else if (packetSize == 7)
        {
          tmp = data.substring(1, 3).toInt();
          if (tmp > ZERO && tmp <= PWM_MAX)
          {
            omegaRefForward = tmp;
            omegaRefBackward = 0;
            omegaRefLeft = 0;
            omegaRefRight = 0;
            if (stepperCounter >= 80)
              digitalWrite(stepperMotorSleep, LOW);
            else {
              digitalWrite(stepperMotorSleep, HIGH);
              digitalWrite(stepperMotorDirection, LOW);
              digitalWrite(stepperMotorStep, HIGH);
              delay(3);
              digitalWrite(stepperMotorStep, LOW);
              delay(3);
              stepperCounter++;
            }
          }
        }
      }
      else if (data.substring(0, 1) == "2")
      {
        //TYLKO PRAWO
        tmp = data.substring(1, 4).toInt();
        if (tmp > ZERO && tmp <= PWM_MAX)
        {
          omegaRefLeft = tmp;
          omegaRefBackward = 0;
          omegaRefForward = 0;
        }
      }
      else if (data.substring(0, 1) == "3")
      {
        //TYLKO TYL
        tmp = data.substring(1, 4).toInt();
        if (tmp > ZERO && tmp <= PWM_MAX)
        {
          omegaRefBackward = tmp;
          omegaRefForward = 0;
          omegaRefRight = 0;
          omegaRefLeft = 0;
        }
      }
      else if (data.substring(0, 1) == "4")
      {
        //TYLKO LEWO
        tmp = data.substring(1, 4).toInt();
        if (tmp >= ZERO && tmp <= PWM_MAX)
        {
          omegaRefRight = tmp;
          omegaRefBackward = 0;
          omegaRefForward = 0;
        }
      }
      else if (data.substring(0, 1) == "5")
      {
        //przod prawo
        if (packetSize == 7)
        {
          if (data.substring(3, 4) == " ")
          {
            tmp = data.substring(1, 3).toInt();
            tmp2 = data.substring(4, 7).toInt();
            if (tmp > ZERO && tmp <= PWM_MAX)
            {
              omegaRefForward = tmp;
              omegaRefLeft = tmp2;
              omegaRefRight = 0;
              omegaRefBackward = 0;
            }
          }
          else
          {
            tmp = data.substring(1, 4).toInt();
            tmp2 = data.substring(5, 7).toInt();
            if (tmp > ZERO && tmp <= PWM_MAX)
            {
              omegaRefForward = tmp;
              omegaRefLeft = tmp2;
              omegaRefRight = 0;
              omegaRefBackward = 0;
            }
          }
        }
        else if (packetSize == 8)
        {
          tmp = data.substring(1, 4).toInt();
          tmp2 = data.substring(5, 8).toInt();
        }
        if (tmp > ZERO && tmp <= PWM_MAX)
        {
          omegaRefForward = tmp;
          omegaRefLeft = tmp2;
          omegaRefRight = 0;
          omegaRefBackward = 0;
        }
      }
      else if (data.substring(0, 1) == "6")
      {
        //przod lewo
        if (packetSize == 7)
        {
          if (data.substring(3, 4) == " ")
          {
            tmp = data.substring(1, 3).toInt();
            tmp2 = data.substring(4, 7).toInt();
            if (tmp > ZERO && tmp <= PWM_MAX)
            {
              omegaRefForward = tmp;
              omegaRefLeft = 0;
              omegaRefRight = tmp2;
              omegaRefBackward = 0;
            }
          }
          else
          {
            tmp = data.substring(1, 4).toInt();
            tmp2 = data.substring(5, 7).toInt();
            if (tmp > ZERO && tmp <= PWM_MAX)
            {
              omegaRefForward = tmp;
              omegaRefLeft = 0;
              omegaRefRight = tmp2;
              omegaRefBackward = 0;
            }
          }
        }
        else if (packetSize == 8)
        {
          tmp = data.substring(1, 4).toInt();
          tmp2 = data.substring(5, 8).toInt();
        }
        if (tmp > ZERO && tmp <= PWM_MAX)
        {
          omegaRefForward = tmp;
          omegaRefLeft = 0;
          omegaRefRight = tmp2;
          omegaRefBackward = 0;
        }
      }
      else if (data.substring(0, 1) == "7")
      {
        //tyl prawo
        if (packetSize == 7)
        {
          if (data.substring(3, 4) == " ")
          {
            tmp = data.substring(1, 3).toInt();
            tmp2 = data.substring(4, 7).toInt();
            if (tmp > ZERO && tmp <= PWM_MAX)
            {
              omegaRefForward = 0;
              omegaRefLeft = tmp2;
              omegaRefRight = 0;
              omegaRefBackward = tmp;
            }
          }
          else
          {
            tmp = data.substring(1, 4).toInt();
            tmp2 = data.substring(5, 7).toInt();
            if (tmp > ZERO && tmp <= PWM_MAX)
            {
              omegaRefForward = 0;
              omegaRefLeft = tmp2;
              omegaRefRight = 0;
              omegaRefBackward = tmp;
            }
          }
        }
        if (packetSize == 8)
        {
          tmp = data.substring(1, 4).toInt();
          tmp2 = data.substring(5, 8).toInt();
        }
        if (tmp > ZERO && tmp <= PWM_MAX)
        {
          omegaRefForward = 0;
          omegaRefLeft = tmp2;
          omegaRefRight = 0;
          omegaRefBackward = tmp;
        }
      }
      else if (data.substring(0, 1) == "8")
      {
        //tyl lewo
        if (packetSize == 7)
        {
          if (data.substring(3, 4) == " ")
          {
            tmp = data.substring(1, 3).toInt();
            tmp2 = data.substring(4, 7).toInt();
            if (tmp > ZERO && tmp <= PWM_MAX)
            {
              omegaRefForward = 0;
              omegaRefLeft = 0;
              omegaRefRight = tmp2;
              omegaRefBackward = tmp;
            }
          }
          else
          {
            tmp = data.substring(1, 4).toInt();
            tmp2 = data.substring(5, 7).toInt();
            if (tmp > ZERO && tmp <= PWM_MAX)
            {
              omegaRefForward = 0;
              omegaRefLeft = 0;
              omegaRefRight = tmp2;
              omegaRefBackward = tmp;
            }
          }
        }
        if (packetSize == 8)
        {
          tmp = data.substring(1, 4).toInt();
          tmp2 = data.substring(5, 8).toInt();
        }
        if (tmp > ZERO && tmp <= PWM_MAX)
        {
          omegaRefForward = 0;
          omegaRefLeft = 0;
          omegaRefRight = tmp2;
          omegaRefBackward = tmp;
        }
      }
      /*else if (data.substring(0, 2) == "up")
      {
        if (stepperCounter >= 80)
          digitalWrite(stepperMotorSleep, LOW);
        else {
          digitalWrite(stepperMotorSleep, HIGH);
          digitalWrite(stepperMotorDirection, HIGH);
          digitalWrite(stepperMotorStep, HIGH);
          delay(3);
          digitalWrite(stepperMotorStep, LOW);
          delay(3);
          stepperCounter++;
        }
      }
      else if (data.substring(0, 4) == "down")
      {
        digitalWrite(stepperMotorSleep, HIGH);
        digitalWrite(stepperMotorDirection, LOW);
        digitalWrite(stepperMotorStep, HIGH);
        delay(3);
        digitalWrite(stepperMotorStep, LOW);
        delay(3);
      } */
      else
      {
        omegaRefBackward = ZERO;
        omegaRefRight = ZERO;
        omegaRefForward = ZERO;
        omegaRefLeft = ZERO;
        digitalWrite(stepperMotorStep, LOW);
        digitalWrite(stepperMotorSleep, LOW);
      }
    }
    //SendMessage();
  }
}
