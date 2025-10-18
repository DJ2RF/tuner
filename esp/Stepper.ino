#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266HTTPClient.h>
#define ARDUINOJSON_USE_LONG_LONG 1

#define dirPin 4
#define stepPin 5
#define onoff 0


long int stepsPerRun = 0;
int i=0;
long int ispeed = 0;
bool bspeed = true;
long int calls = 0;
bool fertig=true;
bool richtung=true;
int setedSteps=100;

char cbuffer[20];
String sbuffer;
String sString1 ="hoch,";
String sString2 ="tief,";
String sString3 ="s";

char c;
byte aindex = 0;
byte bindex = 0;
byte cindex = 0;

const char* wifiName = "farswitch";
const char* wifiPass = "Kl79_?Sa13_04_1961Kl79_?Sa";
WiFiServer wifiServer(75);

void charToStringL(const char S[], String &D);
void setzeV(String swert);
void setzeDir(int l_r);
void setzeSteps(long int pos, String sdata);
void setzeRounds(long int pos, String sdata);
void runstepper(int rounds);
void doanything();

void doanything() {
  byte a=0;
}

// Spin the stepper motor 1 revolution quickly:
void runstepper(int rounds)
{
  
  for (int i = 0; i < rounds; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(ispeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(ispeed);
    
  }
  yield();
  // Serial.println(calls);
}
void setzeSteps(long int pos, String sdata)
{
  String sHelp = sdata.substring(pos+1, sdata.length());
  stepsPerRun = sHelp.toInt(); 
  calls=stepsPerRun;
  fertig=false;
  //Serial.println();
  //Serial.println(calls);
}

void setzeRounds(long int pos, String sdata)
{
  String sHelp = sdata.substring(pos+1, sdata.length());
  setedSteps = sHelp.toInt(); 
  if (setedSteps > 100) {setedSteps = 100;}
  if (setedSteps < 1) {setedSteps = 1;}
}

void setzeDir(int l_r) {
  switch (l_r) {
    case 1:
      digitalWrite(dirPin, HIGH);    // hoch Statement(s)
      delay(100);
    break;
    case 2:
      digitalWrite(dirPin, LOW);   // tief Statement(s)
      delay(100);
    break;
    default:
     
    break;
  }
}

void setzeV(int swert) {
  switch (swert) {
    case 1:
      ispeed = 8000;
      bspeed = false;
    break;
    case 2:
      ispeed = 3500;
      bspeed = false;
    break;
    case 3:
      ispeed = 2000;
      bspeed = false;
    break;
    case 4:
      calls=-1;
      //ispeed = 0;
      //stepsPerRun = 0;
      //bspeed = false;
    break;
    default:
      ispeed = 3500;
      bspeed = false;
    break;
  }
}


void charToStringL(const char S[], String &D)
{
    byte at = 0;
    const char *p = S;
    D = "";

    while (*p++) {
      D.concat(S[at++]);
      }
}

void setup() {
  Serial.begin(115200);
  WiFi.setOutputPower(10);
  //WiFi.setAutoReconnect(true);
  //WiFi.persistent(true);
  WiFi.begin(wifiName, wifiPass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  Serial.print("Connected to WiFi. IP:");
  Serial.println(WiFi.localIP());
  wifiServer.begin();
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(onoff, OUTPUT);
  digitalWrite(onoff, LOW);            // aus
  //ESP.wdtEnable(10000);
}

void loop() {
  WiFiClient client = wifiServer.available();
  
  /*if (stepper.distanceToGo() == 0){
    stepper.moveTo(-stepper.currentPosition());
    Serial.println("Changing direction");
  }
  // move the stepper motor (one step at a time)
  stepper.run();
  
  //stepper.setAcceleration(100);
  stepper.setSpeed(0);
  //stepper.runSpeed();
  */ 
  if (client) {
    calls=0;
    while (client.connected()) {
      //bindex=0;
      
      while (client.available()>0) {
        digitalWrite(onoff, HIGH);            // aufwachen
        //byte aindex = client.available();
        c = client.read();
        //delay(100);
        if (c != '\n') {
          cbuffer[bindex] = c;
          bindex++;
          //Serial.println(bindex);
          //Serial.println(c);
        } else {
          //client.print(bindex);
          if (bindex == 1) {fertig=false;}
          cbuffer[bindex] = '\0';
          //Do anything else
          Serial.println(cbuffer);
          charToStringL(cbuffer, sbuffer);
          Serial.println(sbuffer);
          if (sbuffer.indexOf("langsam") == 0) {//muss 0 sein
          setzeV(1);
          bspeed = true;
          }
          if (sbuffer.indexOf("mittel") == 0) {//muss 0 sein
          setzeV(2);
          bspeed = true;
          }
          if (sbuffer.indexOf("power") == 0) {//muss 0 sein
          setzeV(3);
          bspeed = true;
          }
          if (sbuffer.indexOf("hoch,") == 0) {//muss 0 sein
            richtung=true;
            setzeDir(1);
          }
          if (sbuffer.indexOf("tief,") == 0) {//muss 0 sein
            richtung=true;
            setzeDir(2);
          }
          if (sbuffer.indexOf("rounds,") == 0) {//muss 0 sein
            richtung=false;
            setzeDir(2);
          }
          if (sbuffer.indexOf("s") == 0) {//muss 0 sein
          setzeV(4);
          }
          if ((sbuffer.indexOf(",") != -1) && !richtung) {//muss 0 sein
            setzeRounds(sbuffer.indexOf(","), sbuffer);
          }
          if ((sbuffer.indexOf(",") != -1) && richtung) {//muss 0 sein
            setzeSteps(sbuffer.indexOf(","), sbuffer);
          }
          bindex=0;
          Serial.println(stepsPerRun);
          Serial.println(ispeed);
          Serial.println(bspeed);
          if (bspeed) { client.println("Setze Speed "); }
          if (!richtung) { client.println("Setze Rounds "); }
          richtung=true;
          bspeed = false;
        }
      }
      if (((stepsPerRun >=1) && (ispeed >= 2000) && (bspeed == false) && (fertig == false))) {
            runstepper(setedSteps);  
            yield();
            //delay(100);
            calls=calls-1;
            client.print(".");
            if (calls <= 0) {
              calls=stepsPerRun;
              fertig=true;
              client.println();
              client.println("Fertig ");
              }
          }
    }
    client.stop();
    digitalWrite(onoff, LOW);            // aus
    //stepper.setSpeed(0);
    Serial.println("Client disconnected");
  }
}
