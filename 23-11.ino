
#include <Wire.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

/**/
/*LCD OLED ssd1106*/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3c  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static const unsigned char PROGMEM logo_bmp[] = { 0b00000000, 0b11000000,
                                                  0b00000001, 0b11000000,
                                                  0b00000001, 0b11000000,
                                                  0b00000011, 0b11100000,
                                                  0b11110011, 0b11100000,
                                                  0b11111110, 0b11111000,
                                                  0b01111110, 0b11111111,
                                                  0b00110011, 0b10011111,
                                                  0b00011111, 0b11111100,
                                                 };

/*thu vien am thanh*/
#include "DFRobotDFPlayerMini.h"
#define FPSerial Serial2
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);
/*WiFi define*/
#define WIFI_SSID "Hieu_N"
#define WIFI_PASSWORD "hieu1234"
/* firebase*/
#define API_KEY "AIzaSyAW7VIOL5CnPjLA98fDJ-J4yQFqyocabiQ"
#define DATABASE_URL "https://fm-iot-e0bdd-default-rtdb.asia-southeast1.firebasedatabase.app/" 
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;                 
float freq_fb,freq_fb2;
float vol_fb=0,volmask=0;
unsigned long sendDataPrevMillis = 0;
/* ------------*/
/* NETWORK TIME PROTOCOL*/
#include <NTPClient.h>  //
#include <WiFiUdp.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
int gio, phut, giay;
/*RDA5807 DEFINE*/
#define TRUE 1
#define FALSE 0
#define RDA5807M_I2C_ADDR_RANDOM (0x22 >> 1)
#define BAND_WORLD (0x2 << 2)

#define BAND_WEST (0x0 << 2)
#define BAND_JAPAN (0x1 << 2)
#define BAND_WORLD (0x2 << 2)
#define BAND_EAST (0x3 << 2)
#define DEFAULT_BAND BAND_WORLD 
// dinh nghia reg 0x02.
#define BIT_DHIZ 0x8000   // 0b1000 0000 0000 0000:tat tong tro ngo ra.
#define BIT_DMUTE 0x4000  // 0b0100 0000 0000 0000: bat tieng.
#define BIT_DMUTE_OFF 0x0000
#define BIT_BASS 0x1000   // 0b0001 0000 0000 0000: bat bass.
#define BIT_SEEKUP 0x0200 // 0b0000 0010 0000 0000: tim kim len.
#define BIT_RDS 0x0008    // bat RDS.
#define BIT_NEW_METHOD 0x0004  // Phuong phap moi.
// dinh nghia reg 0x03.
#define BAND_MASK 0x000C
#define BIT_RESET 0x0001  //
#define SPACE_MASK word(0x0003)
#define REG_BLEND 0x07
#define FLG_EASTBAND65M 0x0200
#define BAND_SHIFT 2
#define REG_STATUS 0x0A
#define READCHAN_MASK 0x03FF
#define CHAN_MASK 0xFFC0
#define FLG_TUNE word(0x0010)
#define CHAN_SHIFT 6 
/*MIC*/
#define MIC_IN_ANALOG		34
#define LED_OUT_DIGITAL		2	
enum STATE{
	STATE_ON,
	STATE_OFF,
};
STATE State = STATE_ON;
unsigned long t_from = 0;
float th_Amp;float timeout;
int val; 
float volts;
float VoiceAmp;
/*define nut nhan*/
int volume_fm = 0b111;//define volume.
static float tanso_fm = 99.9;
float read_freq;
//int j;
int button1 = 14;//volume down
int button2 = 27;//volume up
int button3 = 26;//freq down
int button4 = 25;//freq up
int lastButtonState = 0;
int lastButtonState2 = 0;
int lastButtonState3 = 0;
int lastButtonState4 = 0;
int buttonState;
int buttonState2;
int buttonState3;
int buttonState4;
//bien cho firebase


/*----------MENU--------------------*/

void firebase_wifi_init(){
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
    }
    else{
      Serial.printf("%s\n", config.signer.signupError.message.c_str());
    }
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void ssd1306_init(){
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  
  }
  display.display();
  delay(2000);
  display.clearDisplay();
  delay(10);
  display.setTextSize(2);  
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 3);
  display.println(F("FM radio"));
  delay(100);
  display.display();
}

void Read_button(){
  buttonState = digitalRead(button1);
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
      volume_fm--; if(volume_fm <=0) volume_fm = 0b0;
      setVolume( volume_fm);
    }
  }
  lastButtonState = buttonState; 

  buttonState2 = digitalRead(button2);
  if (buttonState2 != lastButtonState2) {
    if (buttonState2 == HIGH) { 
      volume_fm++; if(volume_fm >=0b1111) volume_fm = 0b1111;
      setVolume( volume_fm);
      Serial.println(volume_fm);
    }
  }
  lastButtonState2 = buttonState2;  

  buttonState3 = digitalRead(button3);  
  if (buttonState3 != lastButtonState3) {
    if (buttonState3 == HIGH) {    
      tanso_fm = tanso_fm-0.1;
      if(tanso_fm<=87) tanso_fm = 87;
      set_Frequency(tanso_fm);
    }
  }
  lastButtonState3 = buttonState3;          
  buttonState4 = digitalRead(button4);  
  if (buttonState4 != lastButtonState4) {
    if (buttonState4 == HIGH) {
      tanso_fm = tanso_fm+0.1; 
      if(tanso_fm>=108) tanso_fm = 108;
      set_Frequency(tanso_fm);
    }
  }
  lastButtonState4 = buttonState4;  
}
void Mp3_init(){
  FPSerial.begin(9600);
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  if (!myDFPlayer.begin(FPSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0);      
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.volume(30);  
  myDFPlayer.play(25);  
}

void setup() { 
  Wire.begin();
  Serial.begin(9600);
  off_volume();
  firebase_wifi_init(); 
  Mp3_init();
  ssd1306_init();
  FM_init();
  Ntp_init();
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
  pinMode(button4, INPUT);
  pinMode(34, INPUT);
  pinMode(LED_OUT_DIGITAL, OUTPUT);
}
/*-----------------------------------------------------------------------------------------------------------*/
void loop() {
  Read_button();
  set_time();
  read_freq = get_Frequency();
  if(Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 1400 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    Firebase.RTDB.setFloat(&fbdo, "FM_IoT/Frequency",read_freq);
    Firebase.RTDB.setFloat(&fbdo, "FM_IoT/Volume",volume_fm);  
    //Serial.printf("Get float... %s\n",Firebase.RTDB.getFloat(&fbdo, F("/FM_IoT/Frequency_control")) ? String(freq_fb=fbdo.to<float>()).c_str() : fbdo.errorReason().c_str());  
    Firebase.RTDB.getFloat(&fbdo, F("/FM_IoT/Volume_control")); 
    vol_fb=fbdo.to<float>();
    if(volmask!=vol_fb){
      volmask = vol_fb;
      volume_fm = vol_fb;
      setVolume(volume_fm);
    }
    Firebase.RTDB.getFloat(&fbdo, F("/FM_IoT/Frequency_control"));
    freq_fb=fbdo.to<float>();
    if(freq_fb2!=freq_fb){
      freq_fb2 = freq_fb;
      tanso_fm = freq_fb;
      set_Frequency(tanso_fm);
      }           
  }
  oled_display();
}

/*   -----------------------------------------------------------------------------------------------------------------     */
void oled_display(){
  display.clearDisplay();
  display.drawLine(0, 16,127, 16, SSD1306_WHITE);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("Radio IoT"));

  display.setCursor(35, 18);
  display.println(gio);
  display.setCursor(55, 18);
  display.println(F(":"));
  display.setCursor(65, 18);
  display.println(phut);

  display.setCursor(30, 35);
  display.println(F("vol:"));
  display.setCursor(85, 35);
  display.println(volume_fm, DEC);
   
  display.setCursor(0, 50);
  display.println(F("F:"));
  if(tanso_fm>=100){
  display.setCursor(90, 50);
  display.println(F("MHz"));
  }
  else
  {
    display.setCursor(85, 50);
    display.println(F("MHz"));
  }
  display.setCursor(19, 50);
  display.println(read_freq);
  display.display();
}

/*Ham set tan so cho FM*/
// Doc khoang cach kenh va che do Band.
float get_Frequency(){
  uint16_t read_freq = getRegister(0x0a);
  return (read_freq&0b0000001111111111)*0.1+87;
}
void get_Band_Spacing(){
  uint16_t band = getRegister(0x03)&0x000f;
}

uint16_t getRegister(byte reg){
  uint16_t result=0;
  Wire.beginTransmission(RDA5807M_I2C_ADDR_RANDOM);
  Wire.write(reg);
  Wire.endTransmission(FALSE);
  Wire.requestFrom(RDA5807M_I2C_ADDR_RANDOM, 2, TRUE);
  result = Wire.read() << 8;
  result |= Wire.read();
  return result;
}
void setRegister(byte reg, const word value) {
  Wire.beginTransmission(RDA5807M_I2C_ADDR_RANDOM);
  Wire.write(reg);
  Wire.write(highByte(value));
  Wire.write(lowByte(value));
  Wire.endTransmission(true);
}
void set_Frequency(float frequency){
  Process_Noise();
  uint16_t channel = (frequency - 87) / 0.1;
  updateRegister(0x03, 0xfffc|0x0010,(channel<<6)|0x0010);
}
void writeRegister(byte reg, unsigned int value) {
  Wire.beginTransmission(RDA5807M_I2C_ADDR_RANDOM);
  Wire.write(reg);
  Wire.write(value >> 8);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}
void setVolume( unsigned int volume) {          // thay doi am luong
  writeRegister(0x05, 0x84D0 | volume);
}
void updateRegister(byte reg, uint16_t mask, uint16_t value) {
  setRegister(reg, getRegister(reg) & ~mask | value);
}
void setBand(uint16_t band) {
  updateRegister(0x03, BAND_MASK, band);
}
void setSpacing(int space){
  updateRegister(0x03,SPACE_MASK, space);
}
void FM_init(){
  // b1: thiet lap reg 0x02, b2: thiet lap Band reg 0x03, b3: thiet lap khoang cach kênh, b4: thiet lap volume, b5: thiet lap tan so.
  setRegister(0x02, BIT_DHIZ | BIT_DMUTE |BIT_BASS | BIT_SEEKUP |BIT_RDS | BIT_NEW_METHOD| BIT_RESET);
  setBand(DEFAULT_BAND);
  setSpacing(0);
  setVolume(volume_fm);
  set_Frequency(tanso_fm);
}
void off_volume(){
  setRegister(0x02, BIT_DHIZ | BIT_DMUTE_OFF |BIT_BASS | BIT_SEEKUP |BIT_RDS | BIT_NEW_METHOD| BIT_RESET);
}
void on_volume(){
  setRegister(0x02, BIT_DHIZ | BIT_DMUTE |BIT_BASS | BIT_SEEKUP |BIT_RDS | BIT_NEW_METHOD| BIT_RESET);
}

/********************************************/
void Process_Noise(){
  off_volume();
  val = analogRead(MIC_IN_ANALOG);
	volts = (float)val / 4095 * 5.0;	// 0.0 <= volts <= 3.3
	VoiceAmp = abs(volts - 2.5);
	
	/********************
	Led Control
	********************/
	/* StateChart */
	th_Amp = 1.0;
	timeout = 800;
	switch(State){
		case STATE_ON:
			if(th_Amp < VoiceAmp){
				State = STATE_OFF;
				t_from = millis();
			}
			break;			
		case STATE_OFF:
			if(th_Amp < VoiceAmp)			t_from = millis();
			if(timeout < millis() - t_from)	State = STATE_ON;			
			break;
	}
	/* Led control */
	if(State == STATE_ON)	{
    digitalWrite(LED_OUT_DIGITAL, HIGH);
    volume_fm=5;
    setVolume( volume_fm);
  }
	else{
    digitalWrite(LED_OUT_DIGITAL, LOW);
    volume_fm=12;	
    setVolume( volume_fm);
  } 
  on_volume();
}

void Ntp_init(){
  timeClient.begin();
  timeClient.setTimeOffset(25200);
}
void set_time(){
  timeClient.update();
  gio = timeClient.getHours();
  phut = timeClient.getMinutes();
  giay = timeClient.getSeconds();
  tell_time();
}
void tell_time(){
  if((gio==0)&&(phut==0)&&(giay==0))         myDFPlayer.play(0);
   else if((gio==1)&&(phut==0)&&(giay==0))   myDFPlayer.play(1); 
   else if((gio==2)&&(phut==0)&&(giay==0))   myDFPlayer.play(2); 
   else if((gio==3)&&(phut==0)&&(giay==0))   myDFPlayer.play(3); 
   else if((gio==4)&&(phut==0)&&(giay==0))   myDFPlayer.play(4); 
   else if((gio==5)&&(phut==0)&&(giay==0))   myDFPlayer.play(5); 
   else if((gio==6)&&(phut==0)&&(giay==0))   myDFPlayer.play(6); 
   else if((gio==7)&&(phut==0)&&(giay==0))   myDFPlayer.play(7); 
   else if((gio==8)&&(phut==0)&&(giay==0))   myDFPlayer.play(8); 
   else if((gio==9)&&(phut==0)&&(giay==0))   myDFPlayer.play(9); 
   else if((gio==10)&&(phut==0)&&(giay==0))  myDFPlayer.play(10); 
   else if((gio==11)&&(phut==0)&&(giay==0))  myDFPlayer.play(11); 
   else if((gio==12)&&(phut==0)&&(giay==0))  myDFPlayer.play(12); 
   else if((gio==13)&&(phut==0)&&(giay==0))  myDFPlayer.play(13); 
   else if((gio==14)&&(phut==0)&&(giay==0))  myDFPlayer.play(14); 
   else if((gio==15)&&(phut==0)&&(giay==0))  myDFPlayer.play(15); 
   else if((gio==16)&&(phut==0)&&(giay==0))  myDFPlayer.play(16); 
   else if((gio==17)&&(phut==0)&&(giay==0))  myDFPlayer.play(17); 
   else if((gio==18)&&(phut==0)&&(giay==0))  myDFPlayer.play(18); 
   else if((gio==19)&&(phut==0)&&(giay==0))  myDFPlayer.play(19); 
   else if((gio==20)&&(phut==0)&&(giay==0))  myDFPlayer.play(20); 
   else if((gio==21)&&(phut==0)&&(giay==0))  myDFPlayer.play(21); 
   else if((gio==22)&&(phut==0)&&(giay==0))  myDFPlayer.play(22); 
   else if((gio==23)&&(phut==0)&&(giay==0))  myDFPlayer.play(23);  
}
