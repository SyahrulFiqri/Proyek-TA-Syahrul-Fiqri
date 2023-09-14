#define BLYNK_TEMPLATE_ID "TMPL6SASZPomE"
#define BLYNK_TEMPLATE_NAME "Kontrol Arus dan Tegangan"
#define BLYNK_AUTH_TOKEN "w_AEKidf5_3-DJxKxaJcDBZmlfyLJuFo"
#include <SoftwareSerial.h> /* include virtual Serial Port coding */
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
SoftwareSerial PZEMSerial;  // Move the PZEM DC Energy Meter communication pins from Rx to pin D1 = GPIO 5 & TX to pin D2 = GPIO 4
LiquidCrystal_I2C lcd(0x27, 16, 2);


/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Syahrul Fiqri";                     // Key in your wifi name. You can check with your smart phone for your wifi name
char pass[] = "alulrull";                          // Key in your wifi password.

#include <ModbusMaster.h>                // Load the (modified) library for modbus communication command codes. Kindly install at our website.
#define MAX485_DE 16                     // Define DE Pin to Arduino pin. Connect DE Pin of Max485 converter module to Pin D0 (GPIO 16) Node MCU board
#define MAX485_RE 12                     // Define RE Pin to Arduino pin. Connect RE Pin of Max485 converter module to Pin D1 (GIPO 5) Node MCU board \
                                         // These DE anr RE pins can be any other Digital Pins to be activated during transmission and reception process.
static uint8_t pzemSlaveAddr = 0x01;     // Declare the address of device (meter 1) in term of 8 bits.
static uint8_t pzemSlaveAddr2 = 0x02;    // Declare the address of device (meter 2) in term of 8 bits.
static uint16_t NewshuntAddr = 0x0002;   // Declare your external shunt value for DC Meter. Default 0x0000 is 100A, replace to "0x0001" if using 50A shunt, 0x0002 is for 200A, 0x0003 is for 300A
static uint16_t NewshuntAddr2 = 0x0000;  // By default manufacturer may already set, however, to set manually kindly delete the "//" for line "// setShunt(0x01);" in void setup.
ModbusMaster node;                       /* activate modbus master codes*/
ModbusMaster node2;
float PZEMVoltage = 0; /* Declare value for DC voltage */
float PZEMCurrent = 0; /* Declare value for DC current*/
float PZEMPower = 0;   /* Declare value for DC Power */
float PZEMEnergy = 0;  /* Declare value for DC Energy */

float PZEMVoltage2 = 0; /* Declare value for DC voltage */
float PZEMCurrent2 = 0; /* Declare value for DC current*/
float PZEMPower2 = 0;   /* Declare value for DC Power */
float PZEMEnergy2 = 0;  /* Declare value for DC Energy */

float tegangan1 = 0;
float tegangan2 = 0;
float arus1 = 0;
float arus2 = 0;
float energy1 = 0;
float energy2 = 0;
float daya1 = 0;
float daya2 = 0;

unsigned long startMillisPZEM;   /* start counting time for LCD Display */
unsigned long currentMillisPZEM; /* current counting time for LCD Display */
unsigned long startMilliSetShunt;
const unsigned long periodPZEM = 500;  // refresh every X seconds (in seconds) in LED Display. Default 1000 = 1 second
const unsigned long lcdterhubung = 3000;
const unsigned long lembar = 6000;
const unsigned long lcdpages = 500;


unsigned long startmillis2;
unsigned long currentmillis2;
unsigned long startmillis3;
unsigned long currentmillis3;
unsigned long startmillis4;
unsigned long currentmillis4;

/* 2 - Data submission to Blynk Server  */

unsigned long startMillisReadData;         /* start counting time for data collection */
unsigned long currentMillisReadData;       /* current counting time for data collection */
const unsigned long periodReadData = 1000; /* refresh every X seconds (in seconds) in LED Display. Default 1000 = 1 second */
int ResetEnergy = 0;                       /* reset energy function */
int ResetEnergy2 = 0;
int a = 0;
int pages = 1;
unsigned long startMillis1;  // to count time during initial start up

void setup()
{
  lcd.begin();
  lcd.setCursor(0, 0);
  lcd.print("Menghubungkan");
  lcd.setCursor(0, 1);
  lcd.print(ssid);
  startMillis1 = millis();

  /*0 General*/

  Serial.begin(115200);                           /* To assign communication port to communicate with meter. with 2 stop bits (refer to manual)*/
  PZEMSerial.begin(9600, SWSERIAL_8N2, 13, 0);  // 14 = Rx/R0/ GPIO 14 (D2) & 12 = Tx/DI/ GPIO 12 (D3) on NodeMCU
 Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  currentmillis2 = millis();
  if (currentmillis2 - startmillis2 >= lcdterhubung) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Terhubung");
    lcd.setCursor(0, 1);
    lcd.print(ssid);
    startmillis2 = currentmillis2;
  }
  startmillis2 = millis();


  /* 1- PZEM-017 DC Energy Meter */

  startMillisPZEM = millis();             /* Start counting time for run code */
  pinMode(MAX485_RE, OUTPUT);             /* Define RE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  pinMode(MAX485_DE, OUTPUT);             /* Define DE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  digitalWrite(MAX485_RE, 0);             /* Arduino create output signal for pin RE as LOW (no output)*/
  digitalWrite(MAX485_DE, 0);             /* Arduino create output signal for pin DE as LOW (no output)*/
                                          // both pins no output means the converter is in communication signal receiving mode
  node.preTransmission(preTransmission);  // Callbacks allow us to configure the RS485 transceiver correctly
  node.postTransmission(postTransmission);
  node2.preTransmission(preTransmission);  // Callbacks allow us to configure the RS485 transceiver correctly
  node2.postTransmission(postTransmission);
  delay(1000); /* after everything done, wait for 1 second */

  /* 2 - Data submission to Blynk Server  */

  startMillisReadData = millis(); /* Start counting time for data submission to Blynk Server*/
}

void loop()
{
 if (PZEMVoltage < 10) {
    tegangan1 = 0;
    arus1 = 0;
    daya1 = 0;
  }
  if (PZEMVoltage > 10) {
    tegangan1 = PZEMVoltage;
    arus1 = PZEMCurrent;
    daya1 = PZEMPower;
  }
  if (PZEMVoltage2 < 10) {
    tegangan2 = 0;
    arus2 = 0;
    daya2 = 0;
  }
  if (PZEMVoltage2 > 10) {
    tegangan2 = PZEMVoltage2;
    arus2 = PZEMCurrent2;
    daya2 = PZEMPower2;
  }
  /* 0- General */
  if (pages == 1) {
    currentmillis3 = millis();
    if (currentmillis3 - startmillis3 >= lcdpages) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Tegangan Panel");
      lcd.setCursor(0, 1);
      lcd.print(tegangan1);
      lcd.print(" Volt ");
      startmillis3 = currentmillis3;
    }
  }
  if (pages == 2) {
    currentmillis3 = millis();
    if (currentmillis3 - startmillis3 >= lcdpages) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Arus Panel");
      lcd.setCursor(0, 1);
      lcd.print(PZEMCurrent);
      lcd.print(" Ampere ");
      startmillis3 = currentmillis3;
    }
  }
  if (pages == 3) {
    currentmillis3 = millis();
    if (currentmillis3 - startmillis3 >= lcdpages) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Daya Panel");
      lcd.setCursor(0, 1);
      lcd.print(PZEMPower);
      lcd.print(" Watt ");
      startmillis3 = currentmillis3;
    }
  }
  if (pages == 4) {
    currentmillis3 = millis();
    if (currentmillis3 - startmillis3 >= lcdpages) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Energy ");
      lcd.setCursor(0, 1);
      lcd.print(PZEMEnergy);
      lcd.print(" Wh ");
      startmillis3 = currentmillis3;
    }
  }
  if (pages == 5) {
    currentmillis3 = millis();
    if (currentmillis3 - startmillis3 >= lcdpages) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Tegangan Baterai ");
      lcd.setCursor(0, 1);
      lcd.print(tegangan2);
      lcd.print(" Volt ");
      startmillis3 = currentmillis3;
    }
  }
  if (pages == 6) {
    currentmillis3 = millis();
    if (currentmillis3 - startmillis3 >= lcdpages) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Arus Baterai ");
      lcd.setCursor(0, 1);
      lcd.print(PZEMCurrent2);
      lcd.print(" Ampere ");
      startmillis3 = currentmillis3;
    }
  }
  if (pages == 7) {
    currentmillis3 = millis();
    if (currentmillis3 - startmillis3 >= lcdpages) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Energy ");
      lcd.setCursor(0, 1);
      lcd.print(PZEMEnergy2);
      lcd.print(" Wh ");
      startmillis3 = currentmillis3;
    }
  }
  if (pages == 8) {
    currentmillis3 = millis();
    if (currentmillis3 - startmillis3 >= lcdpages) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("PemakaianBaterai ");
      lcd.setCursor(0, 1);
      lcd.print(PZEMPower2);
      lcd.print(" Watt ");
      startmillis3 = currentmillis3;
    }
  }

  currentmillis2 = millis();
  if (currentmillis2 - startmillis2 >= lembar) {
    pages++;
    startmillis2 = currentmillis2;
  }
  if (pages > 8) {
    pages = 1;
  }

  Blynk.run();
  currentMillisPZEM = millis();

  /* 3- Set Shunt */
  if (millis() - startMilliSetShunt == 10000) { setShunt(0x01); }
  if (millis() - startMilliSetShunt == 15000) { setShunt2(0x02); }

  if (a == 0) {
    node.begin(pzemSlaveAddr, PZEMSerial);                 /* Define and start the Modbus RTU communication. Communication to specific slave address and which Serial port */
    if (currentMillisPZEM - startMillisPZEM >= periodPZEM) /* for every x seconds, run the codes below*/
    {
      uint8_t result;                              /* Declare variable "result" as 8 bits */
      result = node.readInputRegisters(0x0000, 6); /* read the 9 registers (information) of the PZEM-014 / 016 starting 0x0000 (voltage information) kindly refer to manual)*/
      if (result == node.ku8MBSuccess)             /* If there is a response */
      {
        uint32_t tempdouble = 0x00000000;                                                     /* Declare variable "tempdouble" as 32 bits with initial value is 0 */
        PZEMVoltage = node.getResponseBuffer(0x0000) / 100.0;                                 /* get the 16bit value for the voltage value, divide it by 100 (as per manual)- 0x0000 to 0x0008 are the register address of the measurement value*/
        PZEMCurrent = node.getResponseBuffer(0x0001) / 100.0;                                 /* get the 16bit value for the current value, divide it by 100 (as per manual) */
        tempdouble = (node.getResponseBuffer(0x0003) << 16) + node.getResponseBuffer(0x0002); /* get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
        PZEMPower = tempdouble / 10.0;                                                        /* Divide the value by 10 to get actual power value (as per manual) */
        tempdouble = (node.getResponseBuffer(0x0005) << 16) + node.getResponseBuffer(0x0004); /* get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
        PZEMEnergy = tempdouble;
      }
      if (pzemSlaveAddr == 5) /* just for checking purpose to see whether can read modbus*/
      {
      } else {
      }
      a = 1;
      startMillisPZEM = currentMillisPZEM;
    }
  }

  if (a == 1) {
    node2.begin(pzemSlaveAddr2, PZEMSerial);
    if (currentMillisPZEM - startMillisPZEM >= periodPZEM) /* for every x seconds, run the codes below*/
    {
      uint8_t result2;                               /* Declare variable "result" as 8 bits */
      result2 = node2.readInputRegisters(0x0000, 6); /* read the 9 registers (information) of the PZEM-014 / 016 starting 0x0000 (voltage information) kindly refer to manual)*/
      if (result2 == node2.ku8MBSuccess)             /* If there is a response */
      {
        uint32_t tempdouble2 = 0x00000000;                                                       /* Declare variable "tempdouble" as 32 bits with initial value is 0 */
        PZEMVoltage2 = node2.getResponseBuffer(0x0000) / 100.0;                                  /* get the 16bit value for the voltage value, divide it by 100 (as per manual)- 0x0000 to 0x0008 are the register address of the measurement value*/
        PZEMCurrent2 = node2.getResponseBuffer(0x0001) / 100.0;                                  /* get the 16bit value for the current value, divide it by 100 (as per manual) */
        tempdouble2 = (node2.getResponseBuffer(0x0003) << 16) + node2.getResponseBuffer(0x0002); /* get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
        PZEMPower2 = tempdouble2 / 10.0;                                                         /* Divide the value by 10 to get actual power value (as per manual) */
        tempdouble2 = (node2.getResponseBuffer(0x0005) << 16) + node2.getResponseBuffer(0x0004); /* get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit */
        PZEMEnergy2 = tempdouble2;
      }
      if (pzemSlaveAddr == 5) /* just for checking purpose to see whether can read modbus*/
      {
      } else {
      }
      a = 0;
      startMillisPZEM = currentMillisPZEM;
    }
  }
  /* count time for program run every second (by default)*/
  /* 2 - Data submission to Blynk Server  */

  currentMillisReadData = millis();                                  /* Set counting time for data submission to server*/
  if (currentMillisReadData - startMillisReadData >= periodReadData) /* for every x seconds, run the codes below*/
  {
    Serial.println("Energy Meter 1 : ");
    Serial.print("Vdc : ");
    Serial.print(PZEMVoltage);
    Serial.println(" V ");
    Serial.print("Idc : ");
    Serial.print(PZEMCurrent);
    Serial.println(" A ");
    Serial.print("Power : ");
    Serial.print(PZEMPower);
    Serial.println(" W ");
    Serial.print("Energy : ");
    Serial.print(PZEMEnergy);
    Serial.println(" Wh ");
    Serial.print("Tegangan Sensor 1: ");
    Serial.print(tegangan1);
    Serial.print(" V ");
    Serial.print("\n");
    Serial.print("==========================");
    Serial.print("\n");
    Serial.println("Energy Meter 2 : ");
    Serial.print("Vdc : ");
    Serial.print(PZEMVoltage2);
    Serial.println(" V ");
    Serial.print("Idc : ");
    Serial.print(PZEMCurrent2);
    Serial.println(" A ");
    Serial.print("Power : ");
    Serial.print(PZEMPower2);
    Serial.println(" W ");
    Serial.print("Energy : ");
    Serial.print(PZEMEnergy2);
    Serial.println(" Wh ");
    Serial.print("Tegangan Sensor 2: ");
    Serial.print(tegangan2);
    Serial.print(" V ");
    Serial.print("\n");
    Serial.print("==========================");
    Serial.print("\n");


    Blynk.virtualWrite(V0, tegangan1);  // Send data to Blynk Server. Voltage value as virtual pin V0
    Blynk.virtualWrite(V1, PZEMCurrent);
    Blynk.virtualWrite(V2, PZEMPower);
    Blynk.virtualWrite(V3, PZEMEnergy);
    Blynk.virtualWrite(V13, PZEMVoltage);

    Blynk.virtualWrite(V5, tegangan2);  // Send data to Blynk Server. Voltage value as virtual pin V0
    Blynk.virtualWrite(V6, PZEMCurrent2);
    Blynk.virtualWrite(V7, PZEMPower2);
    Blynk.virtualWrite(V8, PZEMEnergy2);
    Blynk.virtualWrite(V14, PZEMVoltage2);

    startMillisReadData = millis();
  }
}

void preTransmission() /* transmission program when triggered*/
{
  /* 1- PZEM-017 DC Energy Meter */

  if (millis() - startMillis1 > 5000)  // Wait for 5 seconds as ESP Serial cause start up code crash
  {
    digitalWrite(MAX485_RE, 1); /* put RE Pin to high*/
    digitalWrite(MAX485_DE, 1); /* put DE Pin to high*/
    delay(1);                   // When both RE and DE Pin are high, converter is allow to transmit communication
  }
}

void postTransmission() /* Reception program when triggered*/
{

  /* 1- PZEM-017 DC Energy Meter */

  if (millis() - startMillis1 > 5000)  // Wait for 5 seconds as ESP Serial cause start up code crash
  {
    delay(3);                   // When both RE and DE Pin are low, converter is allow to receive communication
    digitalWrite(MAX485_RE, 0); /* put RE Pin to low*/
    digitalWrite(MAX485_DE, 0); /* put DE Pin to low*/
  }
}

void setShunt(uint8_t slaveAddr)  //Change the slave address of a node
{

  /* 1- PZEM-017 DC Energy Meter */

  static uint8_t SlaveParameter = 0x06;     /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0003; /* change shunt register address command code */

  uint16_t u16CRC = 0xFFFF;                  /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, slaveAddr);  // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewshuntAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewshuntAddr));

  preTransmission(); /* trigger transmission mode*/

  PZEMSerial.write(slaveAddr); /* these whole process code sequence refer to manual*/
  PZEMSerial.write(SlaveParameter);
  PZEMSerial.write(highByte(registerAddress));
  PZEMSerial.write(lowByte(registerAddress));
  PZEMSerial.write(highByte(NewshuntAddr));
  PZEMSerial.write(lowByte(NewshuntAddr));
  PZEMSerial.write(lowByte(u16CRC));
  PZEMSerial.write(highByte(u16CRC));
  delay(10);
  postTransmission(); /* trigger reception mode*/
  delay(100);
  while (PZEMSerial.available()) /* while receiving signal from Serial3 from meter and converter */
  {
  }
}

void setShunt2(uint8_t slaveAddr2)  //Change the slave address of a node
{

  /* 1- PZEM-017 DC Energy Meter */

  static uint8_t SlaveParameter2 = 0x06;     /* Write command code to PZEM */
  static uint16_t registerAddress2 = 0x0003; /* change shunt register address command code */

  uint16_t u16CRC2 = 0xFFFF;                    /* declare CRC check 16 bits*/
  u16CRC2 = crc16_update(u16CRC2, slaveAddr2);  // Calculate the crc16 over the 6bytes to be send
  u16CRC2 = crc16_update(u16CRC2, SlaveParameter2);
  u16CRC2 = crc16_update(u16CRC2, highByte(registerAddress2));
  u16CRC2 = crc16_update(u16CRC2, lowByte(registerAddress2));
  u16CRC2 = crc16_update(u16CRC2, highByte(NewshuntAddr2));
  u16CRC2 = crc16_update(u16CRC2, lowByte(NewshuntAddr2));

  preTransmission(); /* trigger transmission mode*/

  PZEMSerial.write(slaveAddr2); /* these whole process code sequence refer to manual*/
  PZEMSerial.write(SlaveParameter2);
  PZEMSerial.write(highByte(registerAddress2));
  PZEMSerial.write(lowByte(registerAddress2));
  PZEMSerial.write(highByte(NewshuntAddr2));
  PZEMSerial.write(lowByte(NewshuntAddr2));
  PZEMSerial.write(lowByte(u16CRC2));
  PZEMSerial.write(highByte(u16CRC2));
  delay(10);
  postTransmission(); /* trigger reception mode*/
  delay(100);
  while (PZEMSerial.available()) /* while receiving signal from Serial3 from meter and converter */
  {
  }
}

BLYNK_WRITE(V4)  // Virtual push button to reset energy for Meter 1
{
  if (param.asInt() == 1) {
    uint16_t u16CRC = 0xFFFF;           /* declare CRC check 16 bits*/
    static uint8_t resetCommand = 0x42; /* reset command code*/
    uint8_t slaveAddr = 0X01;           // if you set different address, make sure this slaveAddr must change also
    u16CRC = crc16_update(u16CRC, slaveAddr);
    u16CRC = crc16_update(u16CRC, resetCommand);
    preTransmission();                  /* trigger transmission mode*/
    PZEMSerial.write(slaveAddr);        /* send device address in 8 bit*/
    PZEMSerial.write(resetCommand);     /* send reset command */
    PZEMSerial.write(lowByte(u16CRC));  /* send CRC check code low byte  (1st part) */
    PZEMSerial.write(highByte(u16CRC)); /* send CRC check code high byte (2nd part) */
    delay(10);
    postTransmission(); /* trigger reception mode*/
    delay(100);
  }
}

BLYNK_WRITE(V12)  // Virtual push button to reset energy for Meter 2
{
  if (param.asInt() == 1) {
    uint16_t u16CRC = 0xFFFF;           /* declare CRC check 16 bits*/
    static uint8_t resetCommand = 0x42; /* reset command code*/
    uint8_t slaveAddr = 0X02;           // if you set different address, make sure this slaveAddr must change also
    u16CRC = crc16_update(u16CRC, slaveAddr);
    u16CRC = crc16_update(u16CRC, resetCommand);
    preTransmission();                  /* trigger transmission mode*/
    PZEMSerial.write(slaveAddr);        /* send device address in 8 bit*/
    PZEMSerial.write(resetCommand);     /* send reset command */
    PZEMSerial.write(lowByte(u16CRC));  /* send CRC check code low byte  (1st part) */
    PZEMSerial.write(highByte(u16CRC)); /* send CRC check code high byte (2nd part) */
    delay(10);
    postTransmission(); /* trigger reception mode*/
    delay(100);
  }
}
