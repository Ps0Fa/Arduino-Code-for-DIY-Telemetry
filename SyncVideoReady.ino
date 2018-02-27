// #include <SparkFunLSM9DS1.h>
#include <SD.h>
//#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SimpleKalmanFilter.h>
#include <Adafruit_GPS.h>

//#include "TinyGPS++.h"
//TinyGPSPlus tinyGPSPlus;

enum
{
  ST7735_INITB      = 0,        // 1.8" (128x160) ST7735B chipset (only one type)
  ST7735_INITR_GREENTAB   = 1,        // 1.8" (128x160) ST7735R chipset with green tab (same as ST7735_INITR_18GREENTAB)
  ST7735_INITR_REDTAB   = 2,        // 1.8" (128x160) ST7735R chipset with red tab (same as ST7735_INITR_18REDTAB)
  ST7735_INITR_BLACKTAB   = 3,        // 1.8" (128x160) ST7735S chipset with black tab (same as ST7735_INITR_18BLACKTAB)
  ST7735_INITR_144GREENTAB    = 4,        // 1.4" (128x128) ST7735R chipset with green tab
  ST7735_INITR_18GREENTAB   = ST7735_INITR_GREENTAB,  // 1.8" (128x160) ST7735R chipset with green tab
  ST7735_INITR_18REDTAB   = ST7735_INITR_REDTAB,    // 1.8" (128x160) ST7735R chipset with red tab
  ST7735_INITR_18BLACKTAB   = ST7735_INITR_BLACKTAB,  // 1.8" (128x160) ST7735S chipset with black tab
};
#define ST7735_CHIPSET    ST7735_INITR_18BLACKTAB
#define  ST7735_CS_PIN   6     // <= /CS pin (chip-select, LOW to get attention of ST7735, HIGH and it ignores SPI bus)
#define ST7735_DC_PIN   5     // <= DC pin (1=data or 0=command indicator line) also called RS
#define  ST7735_SAVE_SPCR  0
//SCLK: 52, MOSI: 51

#include <SPI.h>        // must include this here (or else IDE can't find it)
                                           
#include <PDQ_GFX.h>        // PDQ: Core graphics library
#include <PDQ_ST7735.h>     // PDQ: Hardware-specific driver library
PDQ_ST7735 tft;     // PDQ: create LCD object (using pins in "PDQ_ST7735_config.h")

#include <Fonts/FreeMono9pt7b.h>  // include fancy serif font
#include <Fonts/FreeSans12pt7b.h> // include fancy sans-serif font
extern "C" char __data_start[];    // start of SRAM data
extern "C" char _end[];     // end of SRAM data (used to check amount of SRAM this program's variables use)
extern "C" char __data_load_end[];  // end of FLASH (used to check amount of Flash this program's code and data uses)
#define  ST7735_RST_PIN   7 

#include <Fonts/FreeSansBoldOblique9pt7b.h>
#include <Fonts/FreeSansBoldOblique24pt7b.h>

SoftwareSerial mySerial(11,10);
Adafruit_GPS GPS(&mySerial);

SimpleKalmanFilter xKalhman(1, 1, 0.2);
SimpleKalmanFilter yKalhman(1, 1, 0.2);
SimpleKalmanFilter zKalhman(1, 1, 0.05);
SimpleKalmanFilter kSound(1, 1, 0.001);

float NN_LeanAngle = 0;
float NN_Will = 0;
float NN_Alpha = 0;

#define TOMILLIGAUSS 1953L  // For A1301: 2.5mV = 1Gauss, and 1024 analog steps = 5V, so 1 step = 1953mG

const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

static unsigned long lastPrint = 0; // Keep track of print time
static unsigned long lastCalc = 0; // Keep track of print time

int initialRead = 0;

const int chipSelect = 53;

String initialData[138];

float igx = 0;
float igy = 0;
float igz = 0;
float iax = 0;
float iay = 0;
float iaz = 0;
float gmes1 = 0;
float gmes2 = 0;
int gpsHour = 0;
int gpsMinute = 0;
int gpsSeconds = 0;
int gpsMilliseconds = 0;
float gpsAlt = 0;

String gpsRawData[20];
long gpsRawTime = 0;
long naGPSmillis = 0;
long syncVidTimer = 0;
int openScreen = 0;

long sound = 0;
float ksound = 0;

long normalTPS = 0;
long normalBRK = 0;
String gpsData ="";
int writeCounter = 0;
int writeButton = 0;
int trackp = 0;

long lapTime = 0;
long startLapTime = 0;
int lapcounterD = 0;
int lapcounterU = 0;
int lapCounterDown = 0;
int lapCounterUp = 0;
int currentLap = 0;
String UpORDown = "Prepearing";
long echoflag = 0;
int positionOnTrack = 0;
int prevpositionOnTrack = 0;
String whattosay = "";
float myLaptiming = 0;
String tftWritePosition = "";

long splitDraftTimer = 0;
int echosplit = 0;
float split1 = 0;
float split2 = 0;
float bestlap = 0;
float bestlapsplit1 = 0;
float bestlapsplit2 = 0;
float bestsplit1 = 0;
float bestsplit2 = 0;
float DSBS1 = 0;
float DSBS2 = 0;
float DSBLS1 = 0;
float DSBLS2 = 0;
float DLAP = 0;

float globalLat = 0;
float globalLon = 0;
float gclat = 0;
float gclon = 0;

float ghost_Lat[51];
float ghost_Lon[51];
float ghost_rad[51];
int ghost_tps[51];
int ghost_brk[51];
int ghost_string[51];
int ghost_cur_position = -1;

#define ST7735_BROWN 0xA145
#define ST7735_CYAN 0x07FF
#define ST7735_DARKBLUE 0x0011
#define ST7735_DARKGRAY 0xAD55
#define ST7735_GOLD 0xFEA0
#define ST7735_MAGENTA 0xF81F

#define ST7735_CONFIDENT 0x0560
#define ST7735_EASY 0xFFE0
#define ST7735_NORMAL 0xFCC0
#define ST7735_AGRESSIVE 0xFA60
#define ST7735_HARD 0xF9ED
#define ST7735_GLHF 0xF800

#define ST7735_TD05 0x0661
#define ST7735_TD1 0x075D
#define ST7735_TD2 0x021F
#define ST7735_TU05 0xEEC0
#define ST7735_TU1 0xEAE0
#define ST7735_TU2 0xE820


int buttonCounter = 0;
int pressCounter = 0;
int gpsSats = 0;

int inLapScreen = 0;
int inTopScreen = 0;
int inGhostScreen = 0;
int inAlphaScreen = 0;
int inSyncVidScreen = 0;
long clearScrCounterMills = 0;
long angleCounterMills = 0;
int clearScrCounter = 0;
int prevOpenScreen = 0;

String gpsTime1 = "";
String gpsTime2 = "";
String gpsTime1prev = "";
String gpsTime2prev = "";

String tftprlapSec ="N/A";
String tftprlapMills = "N/A";
String tftprlapMins = "N/A";
String tftprLA = "N/A";
String tftprSats = "N/A";
String tftprsp1 = "N/A";
String tftprsp2 = "N/A";
String tftprbl = "N/A";
String tftprdelta = "N/A";
String tftprtrpos = "N/A";
String tftprcurrentlap = "N/A";
String tftprupordown = "N/A";
String tftprinr = "N/A";
String tftprwc = "N/A";
String tftprsp1dif = "N/A";
String tftprsp2dif = "N/A";
String tftprwtsay = "N/A";
String tftpralpha = "N/A";

String tftlapSec ="";
String tftlapMills = "";
String tftlapMins = "";
String tftLA = "";
String tftSats = "";
String tftsp1 = "";
String tftsp2 = "";
String tftbl = "";
String tftdelta = "";
String tfttrpos = "";
String tftcurrentlap = "";
String tftupordown = "";
String tftinr = "N/A";
String tftwc = "N/A";
String tftsp1dif = "N/A";
String tftsp2dif = "N/A";
String tftwtsay = "N/A";
String tftalpha = "N/A";

float inlat = 0;
float inlon = 0;

//---------------------------------------------------------------------------Setup-----------------------------------------Start
void setup() {
  pinMode(8, OUTPUT);
  pinMode(A8, INPUT_PULLUP);
  pinMode(A9, INPUT_PULLUP);
  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand("$PMTK251,38400*27<CR><LF>");
  GPS.begin(38400);
  GPS.sendCommand("$PMTK220,100*2F"); 
  GPS.sendCommand(PGCMD_NOANTENNA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  delay(1000);
  #define GPSECHO false
  while (!Serial) {
    ;
  }

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);

  if (!SD.begin(chipSelect)) {
    initialRead = -1;
  }
  readInitialtxt();
  readGhostcsv();  
  #if defined(ST7735_RST_PIN) // reset like Adafruit does
  FastPin<ST7735_RST_PIN>::setOutput();
  FastPin<ST7735_RST_PIN>::hi();
  FastPin<ST7735_RST_PIN>::lo();
  delay(1);
  FastPin<ST7735_RST_PIN>::hi();
#endif
  tft.begin();      // initialize LCD
  tft.setRotation(1);
  tftSETUPScreen();

}
//---------------------------------------------------------------------------Setup-----------------------------------------End

//------------------------------------------------------------------------Loop--------------------------------------------Start
void loop() {
  char gps_c_inloop = "";
  int gps_exitC_inloop = 0;
  int gps_draftCounter_inloop = 0;
  String gps_inData_inloop = "";

  while (mySerial.available() > 0) {
    char gps_c_inloop = mySerial.read();
    //Serial.print(gps_c_inloop);
    if (gps_c_inloop == ',') {
     gps_exitC_inloop = 1;
    }
    if (gps_exitC_inloop == 0) {
     gps_inData_inloop += gps_c_inloop;
    }
    if (gps_c_inloop == ',') {
      if (gps_draftCounter_inloop < 20) {
        gpsRawData[gps_draftCounter_inloop] = gps_inData_inloop;
      }
      gps_inData_inloop = "";
      gps_exitC_inloop = 0;
      gps_draftCounter_inloop = gps_draftCounter_inloop + 1;
    }
  }
  cBrake();
  //Serial.println(stringToPrint());

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,30,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();

  igx = (float) GyX;
  igy = (float) GyY;
  igz = (float) GyZ;
  iax = (float) AcX / 16700 - 0.1; //To Fix The draft normalazation
  iay = (float) AcY / 16700 - 0.1; //To Fix The draft normalazation
  iaz = (float) AcZ / 16700 - 0.01; //To Fix The draft normalazation

  sound = analogRead(15);
  ksound = kSound.updateEstimate(sound);

  gmes1 = (float) gauseMesurment_1();
  gmes2 = (float) gauseMesurment_2();

  normalTPS = (gaussNormalization(gmes1, initialData[2].toInt(), initialData[3].toInt()) / 10);
  normalBRK = (gaussNormalization(gmes2, initialData[4].toInt(), initialData[5].toInt()) / 10);
  
  NN_Alpha = NN_A(iax, iay);

  TrackPosition();
  LapTimingCalc(trackp);
  whattosay = ghostWhatToSay(ghost_string[ghost_cur_position]);
  if (trackp == -1) {
    tfttrpos = "N/A GPS";
  } else if (trackp == 0) {
    tfttrpos = "N/A Track";
  } else {
    tfttrpos = initialData[trackp];
  }

  if (digitalRead(A9) == LOW && writeButton == 1 && clearScrCounterMills + 200 < millis()) {
    writeCounter = 0;
    writeButton = 0;
    openScreen = 0;
    clearScrCounterMills = millis();
  }
  if (digitalRead(A9) == LOW && writeButton == 0 && clearScrCounterMills + 200 < millis()) {
    writeButton = 1;
    openScreen = 1;
    syncVidTimer = millis() + 5000;
    clearScrCounterMills = millis();
  }
  if (writeButton == 1) {
    if (openScreen == 1) {
      tftSyncVid();
      if (syncVidTimer < millis()) {
        openScreen = 0;
      }
    }

    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    String dataString = "";
    dataString = stringToPrint();
    char stringToCharForPrint[400];
    dataString.toCharArray(stringToCharForPrint, 400);

      // Serial.println(dataString); 
      naGPSmillis = millis();
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {
        if (writeCounter == 0) {
        dataFile.println("NewSession");
        writeCounter = 1;
        }
        dataFile.println(stringToCharForPrint);
        dataFile.close();
        SPI.endTransaction();
      }
      else {
        writeCounter = 0;
        writeButton = 0;
        }
    } else {
      writeCounter = 0;
      writeButton = 0;
  }

  if (digitalRead(A8) == LOW && pressCounter == 0 && writeButton == 0 && openScreen == 0) {
    if (buttonCounter < 3) {
      SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
      tft.fillScreen(ST7735_BLACK);
      SPI.endTransaction();
      buttonCounter = buttonCounter + 1;
      pressCounter = 1;
      inLapScreen = 0;
      inTopScreen = 0;
      inGhostScreen = 0;
      inAlphaScreen = 0;
      clearScrCounterMills = millis();
      if (buttonCounter >= 3) {
        buttonCounter = 0;
      }
    }
  }
  if (clearScrCounterMills + 100 < millis()) {
    pressCounter = 0;
  }

  if (prevOpenScreen == 1 && openScreen == 0) {
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    tft.fillScreen(ST7735_BLACK);
    SPI.endTransaction();
    inLapScreen = 0;
    inTopScreen = 0;
    inGhostScreen = 0;
    inAlphaScreen = 0;
  }

  if (buttonCounter == 0 && openScreen == 0) {
    tftTOPBAR();
    tftLAPALPHA();
  } else if (buttonCounter == 1 && openScreen == 0) {
    tftTOPBAR();
    tftLAPTIMES();
  } else if (buttonCounter == 2 && openScreen == 0) {
    tftTOPBAR();
    tftGHOST();
  }
  prevOpenScreen = openScreen;
}
//------------------------------------------------------Loop------------------------------------------------------------END

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//                                                Returns

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float NN_A(float myValue_x, float myValue_y) {
  float alpha = 1 - ((customAbs(myValue_x) + customAbs(myValue_y)) / 2);
  return alpha;
}

long getLapTimingTrack(float Start_Lat_U, float Start_lon_U, float Finish_lat_U, float Finish_lot_U, float SFRad, float sp1lat, float sp1lon, float sp2lat, float sp2lon, float sprad, float latt, float longt) {
  float rad = SFRad;
  float spradi = sprad;
  float s_lattD = Start_Lat_U;
  float s_longtD = Start_lon_U;
  float f_lattD = Finish_lat_U;
  float f_longtD = Finish_lot_U;

  float startLapD =  ((latt - s_lattD)*(latt - s_lattD) + (longt - s_longtD)*(longt - s_longtD)) - (rad*rad);
  float finishLapD = ((latt - f_lattD)*(latt - f_lattD) + (longt - f_longtD)*(longt - f_longtD)) - (rad*rad);

  if (startLapD <= 0) {
  positionOnTrack = 1;
  }
  if (finishLapD <= 0) {
    positionOnTrack = 2;
  }

  if (positionOnTrack == 1 && prevpositionOnTrack != positionOnTrack) {
    DLAP = lapTime - bestlap;
    if (bestlap == 0) {
      bestlap = lapTime;
      bestlapsplit1 = split1;
      bestlapsplit2 = split2;
    }
    if (bestlap > lapTime) {
      bestlap = lapTime;
      bestlapsplit1 = split1;
      bestlapsplit2 = split2;
    }
    lapCounterDown = lapCounterDown + 1;
    startLapTime = millis();
    UpORDown = "Lap";
    split1 = 0;
    split2 = 0;
  }

  if (lapCounterDown > 0) {
    lapTime = (millis() - startLapTime);
  }

  if (((((latt - sp1lat)*(latt - sp1lat) + (longt - sp1lon)*(longt - sp1lon)) - (spradi*spradi)) < 0) && echosplit == 0) {
    split1 = lapTime;
    splitDraftTimer = millis();
    echosplit = 1;
    if (bestsplit1 == 0 && split1 != 0) {
      bestsplit1 = split1;
    }
    if (bestsplit1 > split1 && split1 != 0) {
      bestsplit1 = split1;
    }
    DSBS1 = split1 - bestsplit1;
    DSBLS1 = split1 - bestlapsplit1;
  }

  
  if (((((latt - sp2lat)*(latt - sp2lat) + (longt - sp2lon)*(longt - sp2lon)) - (spradi*spradi)) < 0) && echosplit == 0) {
    split2 = (lapTime - split1);
    splitDraftTimer = millis();
    echosplit = 1;
    if (bestsplit2 == 0 && split2 != 0) {
      bestsplit2 = split2;
    }
    if (bestsplit2 > split2 && split2 != 0) {
      bestsplit2 = split2;
    }
    DSBS2 = split2 - bestsplit2;
    DSBLS2 = split2 - bestlapsplit2;
  }

  if (millis() - splitDraftTimer > 2000) {
    echosplit = 0;
  }

  ghostPosition();
  
  prevpositionOnTrack = positionOnTrack;
  return lapTime;
}

long getLapTimingHill(float Start_Lat_U, float Start_lon_U, float Finish_lat_U, float Finish_lot_U, float Start_Lat_D, float Start_lon_D, float Finish_lat_D, float Finish_lot_D, float SFRad, float sp1lat, float sp1lon, float sp2lat, float sp2lon, float sprad, float latt, float longt) {
  float rad = SFRad;
  float spradi = sprad;
  float s_lattD = Start_Lat_U;
  float s_longtD = Start_lon_U;
  float f_lattD = Finish_lat_U;
  float f_longtD = Finish_lot_U;

  float s_lattU = Start_Lat_D;
  float s_longtU = Start_lon_D;
  float f_lattU = Finish_lat_D;
  float f_longtU = Finish_lot_D;

  float startLapD =  ((latt - s_lattD)*(latt - s_lattD) + (longt - s_longtD)*(longt - s_longtD)) - (rad*rad);
  float finishLapD = ((latt - f_lattD)*(latt - f_lattD) + (longt - f_longtD)*(longt - f_longtD)) - (rad*rad);
  float startLapU =  ((latt - s_lattU)*(latt - s_lattU) + (longt - s_longtU)*(longt - s_longtU)) - (rad*rad);
  float finishLapU = ((latt - f_lattU)*(latt - f_lattU) + (longt - f_longtU)*(longt - f_longtU)) - (rad*rad);

  if (startLapD <= 0) {
  positionOnTrack = 3;
  }
  if (startLapU <= 0) {
    positionOnTrack = 1;
  }
  if (finishLapD <= 0) {
    positionOnTrack = 4;
  }
  if (finishLapU <= 0) {
    positionOnTrack = 2;
  }

  if (prevpositionOnTrack == 4 && positionOnTrack == 3 && lapcounterD != 1) {
    DLAP = lapTime - bestlap;
    if (bestlap == 0) {
      bestlap = lapTime;
      bestlapsplit1 = split1;
      bestlapsplit2 = split2;
    }
    if (bestlap > lapTime) {
      bestlap = lapTime;
      bestlapsplit1 = split1;
      bestlapsplit2 = split2;
    }
    lapcounterD = 1;
    lapCounterDown = lapCounterDown + 1;
    startLapTime = millis();
    UpORDown = "UpHill";
    split1 = 0;
    split2 = 0;
  } 
  if (prevpositionOnTrack == 2 && positionOnTrack == 1 && lapcounterU != 1) {
    DLAP = lapTime - bestlap;
    if (bestlap == 0) {
      bestlap = lapTime;
    }
    if (bestlap > lapTime) {
      bestlap = lapTime;
      bestlapsplit1 = split1;
      bestlapsplit2 = split2;
    }
    lapcounterU = 1;
    lapCounterUp = lapCounterUp + 1;
    startLapTime = millis();
    UpORDown = "DownHill";
    split1 = 0;
    split2 = 0;
  } 
  if (prevpositionOnTrack == 2 && positionOnTrack == 2) {
    lapcounterD = 0;
    UpORDown = "Rotation";
  } 
  if (prevpositionOnTrack == 4 && positionOnTrack == 4) {
    lapcounterU = 0;
    UpORDown = "Rotation";
  }

  if (lapcounterD > 0) {
    lapTime = (millis() - startLapTime);
  }
  if (lapcounterU > 0) {
    lapTime = (millis() - startLapTime);
  }
  if (((((latt - sp1lat)*(latt - sp1lat) + (longt - sp1lon)*(longt - sp1lon)) - (spradi*spradi)) < 0) && echosplit == 0) {
    split1 = lapTime;
    splitDraftTimer = millis();
    echosplit = 1;
    if (bestsplit1 == 0 && split1 != 0) {
      bestsplit1 = split1;
    }
    if (bestsplit1 > split1 && split1 != 0) {
      bestsplit1 = split1;
    }
    DSBS1 = split1 - bestsplit1;
    DSBLS1 = split1 - bestlapsplit1;
  }
  
  if (((((latt - sp2lat)*(latt - sp2lat) + (longt - sp2lon)*(longt - sp2lon)) - (spradi*spradi)) < 0) && echosplit == 0) {
    split2 = (lapTime - split1);
    splitDraftTimer = millis();
    echosplit = 1;
    if (bestsplit2 == 0 && split2 != 0) {
      bestsplit2 = split2;
    }
    if (bestsplit2 > split2 && split2 != 0) {
      bestsplit2 = split2;
    }
    DSBS2 = split2 - bestsplit2;
    DSBLS2 = split2 - bestlapsplit2;
  }

  if (millis() - splitDraftTimer > 5000) {
    echosplit = 0;
  }

  ghostPosition();

  prevpositionOnTrack = positionOnTrack;
  return lapTime;
}

int gaussNormalization(long gaussToNormalise, int minValue, int maxVlue) {
  float spaceToPercent = abs(maxVlue - gaussToNormalise);
  float Range = abs(maxVlue - minValue);
  float returnNormalData = (spaceToPercent / Range) * 100;

  if (returnNormalData < 0) {
    returnNormalData = 0;
  }
  if (returnNormalData > 100) {
    returnNormalData = 100;
  }
  return ((int) returnNormalData);
}

long gauseMesurment_1() {
  // measure magnetic field
  int raw = analogRead(10); // Range : 0..1024
  return raw;
}

long gauseMesurment_2() {
// measure magnetic field
  int raw = analogRead(11); // Range : 0..1024
  return raw;
}


float whereAmI(float StaticLan, float StaticLot, float StaticRad, float dynamicLan, float dynamicLot){
  float currentPosition = ((dynamicLan - StaticLan)*(dynamicLan - StaticLan) + (dynamicLot - StaticLot)*(dynamicLot - StaticLot)) - (StaticRad)*(StaticRad);
  
  return currentPosition;
}

void TrackPosition() {
  int trackName = -1;

  float Track_1 = whereAmI(initialData[11 + 1].toFloat(), initialData[11 + 2].toFloat(), initialData[11 + 3].toFloat(), gclat, gclon);
  float Track_2 = whereAmI(initialData[29 + 1].toFloat(), initialData[29 + 2].toFloat(), initialData[29 + 3].toFloat(), gclat, gclon);
  float Track_3 = whereAmI(initialData[47 + 1].toFloat(), initialData[47 + 2].toFloat(), initialData[47 + 3].toFloat(), gclat, gclon);
  float Track_4 = whereAmI(initialData[65 + 1].toFloat(), initialData[65 + 2].toFloat(), initialData[65 + 3].toFloat(), gclat, gclon);
  float Track_5 = whereAmI(initialData[83 + 1].toFloat(), initialData[83 + 2].toFloat(), initialData[83 + 3].toFloat(), gclat, gclon);
  float Track_6 = whereAmI(initialData[101 + 1].toFloat(), initialData[101 + 2].toFloat(), initialData[101 + 3].toFloat(), gclat, gclon);
  float Track_7 = whereAmI(initialData[119 + 1].toFloat(), initialData[119 + 2].toFloat(), initialData[119 + 3].toFloat(), gclat, gclon);

  if (gclat != 0 && gclon != 0) {
    if (Track_1 < 0 ) {
      trackName = 11;
    } else if (Track_2 < 0 ) {
      trackName = 29;
    } else if (Track_3 < 0 ) {
      trackName = 47;
    } else if (Track_4 < 0 ) {
      trackName = 65;
    } else if (Track_5 < 0 ) {
      trackName = 83;
    } else if (Track_6 < 0 ) {
      trackName = 101;
    } else if (Track_7 < 0 ) {
      trackName = 119;
    } else {
      trackName = 0;
    }
  }
  trackp = trackName;
}

void LapTimingCalc(int myTrack) {
  myLaptiming = 0;

  if (myTrack == 11) {
    if(initialData[11 + 4].equals(initialData[11 + 8])) {
        myLaptiming = getLapTimingTrack(initialData[11 + 4].toFloat(),initialData[11 + 5].toFloat(),initialData[11 + 6].toFloat(),initialData[11 + 7].toFloat(),initialData[11 + 12].toFloat(),initialData[11 + 13].toFloat(),initialData[11 + 14].toFloat(),initialData[11 + 15].toFloat(),initialData[11 + 16].toFloat(),initialData[11 + 17].toFloat(),gclat,gclon);
      } else {
        myLaptiming = getLapTimingHill(initialData[11 + 4].toFloat(),initialData[11 + 5].toFloat(),initialData[11 + 6].toFloat(),initialData[11 + 7].toFloat(),initialData[11 + 8].toFloat(),initialData[11 + 9].toFloat(),initialData[11 + 10].toFloat(),initialData[11 + 11].toFloat(),initialData[11 + 12].toFloat(),initialData[11 + 13].toFloat(),initialData[11 + 14].toFloat(),initialData[11 + 15].toFloat(),initialData[11 + 16].toFloat(),initialData[11 + 17].toFloat(),gclat,gclon);
      }
  }
  if (myTrack == 29) {
      if(initialData[29 + 4].equals(initialData[29 + 8])) {
        myLaptiming = getLapTimingTrack(initialData[29 + 4].toFloat(),initialData[29 + 5].toFloat(),initialData[29 + 6].toFloat(),initialData[29 + 7].toFloat(),initialData[29 + 12].toFloat(),initialData[29 + 13].toFloat(),initialData[29 + 14].toFloat(),initialData[29 + 15].toFloat(),initialData[29 + 16].toFloat(),initialData[29 + 17].toFloat(),gclat,gclon);
      } else {
        myLaptiming = getLapTimingHill(initialData[29 + 4].toFloat(),initialData[29 + 5].toFloat(),initialData[29 + 6].toFloat(),initialData[29 + 7].toFloat(),initialData[29 + 8].toFloat(),initialData[29 + 9].toFloat(),initialData[29 + 10].toFloat(),initialData[29 + 11].toFloat(),initialData[29 + 12].toFloat(),initialData[29 + 13].toFloat(),initialData[29 + 14].toFloat(),initialData[29 + 15].toFloat(),initialData[29 + 16].toFloat(),initialData[29 + 17].toFloat(),gclat,gclon);    
      }
  }
  if (myTrack == 47) {
      if(initialData[47 + 4].equals(initialData[47 + 8])) {
        myLaptiming = getLapTimingTrack(initialData[47 + 4].toFloat(),initialData[47 + 5].toFloat(),initialData[47 + 6].toFloat(),initialData[47 + 7].toFloat(),initialData[47 + 12].toFloat(),initialData[47 + 13].toFloat(),initialData[47 + 14].toFloat(),initialData[47 + 15].toFloat(),initialData[47 + 16].toFloat(),initialData[47 + 17].toFloat(),gclat,gclon);
      } else {
        myLaptiming = getLapTimingHill(initialData[47 + 4].toFloat(),initialData[47 + 5].toFloat(),initialData[47 + 6].toFloat(),initialData[47 + 7].toFloat(),initialData[47 + 8].toFloat(),initialData[47 + 9].toFloat(),initialData[47 + 10].toFloat(),initialData[47 + 11].toFloat(),initialData[47 + 12].toFloat(),initialData[47 + 13].toFloat(),initialData[47 + 14].toFloat(),initialData[47 + 15].toFloat(),initialData[47 + 16].toFloat(),initialData[47 + 17].toFloat(),gclat,gclon); 
      }
  }
  if (myTrack == 65) {
      if(initialData[65 + 4].equals(initialData[65 + 8])) {
        myLaptiming = getLapTimingTrack(initialData[65 + 4].toFloat(),initialData[65 + 5].toFloat(),initialData[65 + 6].toFloat(),initialData[65 + 7].toFloat(),initialData[65 + 12].toFloat(),initialData[65 + 13].toFloat(),initialData[65 + 14].toFloat(),initialData[65 + 15].toFloat(),initialData[65 + 16].toFloat(),initialData[65 + 17].toFloat(),gclat,gclon);
      } else {
        myLaptiming = getLapTimingHill(initialData[65 + 4].toFloat(),initialData[65 + 5].toFloat(),initialData[65 + 6].toFloat(),initialData[65 + 7].toFloat(),initialData[65 + 8].toFloat(),initialData[65 + 9].toFloat(),initialData[65 + 10].toFloat(),initialData[65 + 11].toFloat(),initialData[65 + 12].toFloat(),initialData[65 + 13].toFloat(),initialData[65 + 14].toFloat(),initialData[65 + 15].toFloat(),initialData[65 + 16].toFloat(),initialData[65 + 17].toFloat(),gclat,gclon); 
      }
  }
  if (myTrack == 83) {
      if(initialData[83 + 4].equals(initialData[83 + 8])) {
        myLaptiming = getLapTimingTrack(initialData[83 + 4].toFloat(),initialData[83 + 5].toFloat(),initialData[83 + 6].toFloat(),initialData[83 + 7].toFloat(),initialData[83 + 12].toFloat(),initialData[83 + 13].toFloat(),initialData[83 + 14].toFloat(),initialData[83 + 15].toFloat(),initialData[83 + 16].toFloat(),initialData[83 + 17].toFloat(),gclat,gclon);
      } else {
        myLaptiming = getLapTimingHill(initialData[83 + 4].toFloat(),initialData[83 + 5].toFloat(),initialData[83 + 6].toFloat(),initialData[83 + 7].toFloat(),initialData[83 + 8].toFloat(),initialData[83 + 9].toFloat(),initialData[83 + 10].toFloat(),initialData[83 + 11].toFloat(),initialData[83 + 12].toFloat(),initialData[83 + 13].toFloat(),initialData[83 + 14].toFloat(),initialData[83 + 15].toFloat(),initialData[83 + 16].toFloat(),initialData[83 + 17].toFloat(),gclat,gclon); 
      }
  }
  if (myTrack == 101) {
      if(initialData[101 + 4].equals(initialData[101 + 8])) {
        myLaptiming = getLapTimingTrack(initialData[101 + 4].toFloat(),initialData[101 + 5].toFloat(),initialData[101 + 6].toFloat(),initialData[101 + 7].toFloat(),initialData[101 + 12].toFloat(),initialData[101 + 13].toFloat(),initialData[101 + 14].toFloat(),initialData[101 + 15].toFloat(),initialData[101 + 16].toFloat(),initialData[101 + 17].toFloat(),gclat,gclon);
      } else {
        myLaptiming = getLapTimingHill(initialData[101 + 4].toFloat(),initialData[101 + 5].toFloat(),initialData[101 + 6].toFloat(),initialData[101 + 7].toFloat(),initialData[101 + 8].toFloat(),initialData[101 + 9].toFloat(),initialData[101 + 10].toFloat(),initialData[101 + 11].toFloat(),initialData[101 + 12].toFloat(),initialData[101 + 13].toFloat(),initialData[101 + 14].toFloat(),initialData[101 + 15].toFloat(),initialData[101 + 16].toFloat(),initialData[101 + 17].toFloat(),gclat,gclon); 
      }
  }
  if (myTrack == 119) {
      if(initialData[119 + 4].equals(initialData[119 + 8])) {
        myLaptiming = getLapTimingTrack(initialData[119 + 4].toFloat(),initialData[119 + 5].toFloat(),initialData[119 + 6].toFloat(),initialData[119 + 7].toFloat(),initialData[119 + 12].toFloat(),initialData[119 + 13].toFloat(),initialData[119 + 14].toFloat(),initialData[119 + 15].toFloat(),initialData[119 + 16].toFloat(),initialData[119 + 17].toFloat(),gclat,gclon);
      } else {
        myLaptiming = getLapTimingHill(initialData[119 + 4].toFloat(),initialData[119 + 5].toFloat(),initialData[119 + 6].toFloat(),initialData[119 + 7].toFloat(),initialData[119 + 8].toFloat(),initialData[119 + 9].toFloat(),initialData[119 + 10].toFloat(),initialData[119 + 11].toFloat(),initialData[119 + 12].toFloat(),initialData[119 + 13].toFloat(),initialData[119 + 14].toFloat(),initialData[119 + 15].toFloat(),initialData[119 + 16].toFloat(),initialData[119 + 17].toFloat(),gclat,gclon); 
      }
  }
}

String lapTimeConversion(long mills) {
  String lapping = "";
  long day = 86400000; // 86400000 milliseconds in a day
  long hour = 3600000; // 3600000 milliseconds in an hour
  long minute = 60000; // 60000 milliseconds in a minute
  long second =  1000; // 1000 milliseconds in a second
  long mil = 1;

  //int days = timeNow / day ; //number of days
  int hours = (mills % day) / hour; //the remainder from days division (in milliseconds) divided by hours, this gives the full hours
  int minutes = ((mills % day) % hour) / minute ; //and so on
  int seconds = (((mills % day) % hour) % minute) / second;
  int millisp = (((mills % day) % hour) % minute) % second / mil;

  lapping = String(hours);
  lapping += " ";
  lapping += String(minutes);
  lapping += ":";
  lapping += String(seconds);
  lapping += ".";
  lapping += String(millisp);

  return lapping;
}

String stringToPrint() {
  String dataString = "";

  dataString += String(igx, 3);
  dataString += ",";
  dataString += String(igy, 3);
  dataString += ",";
  dataString += String(igz, 3);
  dataString += ",";
  dataString += String(iax, 3);
  dataString += ",";
  dataString += String(iay, 3);
  dataString += ",";
  dataString += String(iaz, 3);
  dataString += ",";
  dataString += String(sound);
  dataString += ",";
  dataString += String(ksound);
  dataString += ",";
  dataString += String(millis());
  dataString += ",";
  dataString += String(gpsSats);
  dataString += ",";
  dataString += String(gpsHour);
  dataString += ",";
  dataString += String(gpsMinute);
  dataString += ",";
  dataString += String(gpsSeconds);
  dataString += ",";
  dataString += String(gpsMilliseconds);
  dataString += ",";
  dataString += String(gpsAlt, 3);
  dataString += ",";
  dataString += String(gclat, 8);
  dataString += ",";
  dataString += String(gclon, 8);
  dataString += ",";
  dataString += String(inlat, 8);
  dataString += ",";
  dataString += String(inlon, 8);
  dataString += ",";
  dataString += String(trackp);
  dataString += ",";
  dataString += UpORDown;
  dataString += ",";
  dataString += String(positionOnTrack);
  dataString += ",";
  dataString += String(lapTime);
  dataString += ",";
  dataString += String(lapCounterDown);
  dataString += ",";
  dataString += String(lapCounterUp);
  dataString += ",";
  dataString += String(currentLap);
  dataString += ",";
  dataString += String(myLaptiming, 3);
  dataString += ",";
  dataString += String(split1, 2);
  dataString += ",";
  dataString += String(split2, 2);

  return dataString;
}

void ghostPosition() {
  for (int i=0; i <= 50; i++){
      if (((ghost_Lat[i] - gclat)*(ghost_Lat[i] - gclat) + (ghost_Lon[i] - gclon)*(ghost_Lon[i] - gclon)) - (ghost_rad[i]*ghost_rad[i]) < 0) {
        ghost_cur_position = i;
      }
   }
}

String ghostWhatToSay(int tableVal) {
  String WtoSay = "";
  if (tableVal == -1) {
    WtoSay = "";
  }
  if (tableVal == 1) {
    WtoSay = "push";
  }
  if (tableVal == 2) {
    WtoSay = "push push";
  }
  if (tableVal == 3) {
    WtoSay = "brake";
  }
  if (tableVal == 4) {
    WtoSay = "brake more";
  }
  if (tableVal == 5) {
    WtoSay = "go go go";
  }
  if (tableVal == 6) {
    WtoSay = "hold";
  }
  if (tableVal == 7) {
    WtoSay = "turn";
  }
  if (tableVal == 8) {
    WtoSay = "brake and turn";
  }
  if (tableVal == 9) {
    WtoSay = "start open";
  }
  if (tableVal == 10) {
    WtoSay = "full throttle";
  }
  return WtoSay;
}

void readInitialtxt() {
  File dataFile = SD.open("init.txt");
  int exitC = 0;
  if (dataFile) {
    int draftCounter = 0;
    String inData = "";
    while (dataFile.available()) {
      char in_char = dataFile.read();
      if (in_char == ' ') {
        exitC = 1;
      }
      if (exitC == 0) {
        inData += in_char;  
    }
    if (in_char == '\n')
      {
        initialData[draftCounter] = inData;
        inData = "";
        exitC = 0;
        draftCounter = draftCounter + 1;
      }
    }
    // for(byte i=0;i<137;i++){          //To Delete After Debug
    // Serial.println(initialData[i]);  //To Delete After Debug
    // }                                //To Delete After Debug      
    initialRead = 1;
    dataFile.close();
  } 
  else {
  }
}

void readGhostcsv() {
  File dataFile = SD.open("ghost.csv");
  if (dataFile) {
    int draftCounter = -1;
    int countComma = 0;
    String inData = "";
    while (dataFile.available()) {
      int exitC = 1;
      char in_char = dataFile.read();
      if (isdigit(in_char) || in_char == '.') {
        inData += in_char;  
      }
      if (in_char == ',') {
        if (countComma == 0) {
          ghost_Lat[draftCounter] = inData.toFloat();  
        }
        if (countComma == 1) {
          ghost_Lon[draftCounter] = inData.toFloat();  
        }
        if (countComma == 2) {
          ghost_rad[draftCounter] = inData.toFloat();  
        }
        if (countComma == 3) {
          ghost_tps[draftCounter] = inData.toInt();  
        }
        if (countComma == 4) {
          ghost_brk[draftCounter] = inData.toInt();  
        }
        countComma = countComma + 1;
      }
      if(in_char == '\n') {
        if (countComma == 5) {
          ghost_string[draftCounter] = inData.toInt();  
        }
      }
      if (in_char == ',') {
        inData = "";
      }
      if (in_char == '\n') {
        countComma = 0;
        draftCounter = draftCounter + 1;
        inData = "";
      }
    }
    initialRead = 2;
    dataFile.close();
  }
      //for(byte i=0;i<33;i++){          //To Delete After Debug
    //Serial.println(ghost_Lat[i]);  //To Delete After Debug
    //}                                //To Delete After Debug  
}

void boldRacingTxtB(String txt, int marginLeft, int marginTop, uint16_t color) {
  tft.setCursor(marginLeft, marginTop);
  tft.setFont(&FreeSansBoldOblique24pt7b);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(txt);
}

void tftSETUPScreen() {
  if (initialRead == 2) {
      SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
      tft.fillScreen(ST7735_BLACK);
      tft.setCursor(50, 45);
      tft.setFont(&FreeSansBoldOblique9pt7b);
      tft.setTextColor(ST7735_GREEN);
      tft.print("MORE GAS");
      tft.setCursor(5, 75);
      tft.setFont(&FreeSansBoldOblique9pt7b);
      tft.setTextColor(ST7735_GREEN);
      tft.print("LESS BRAKE");
      delay(1000);
      tft.fillScreen(ST7735_BLACK);
      SPI.endTransaction();
  } else if (initialRead == 1) {
      SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
      tft.fillScreen(ST7735_BLACK);
      tft.setCursor(50, 45);
      tft.setFont(&FreeSansBoldOblique9pt7b);
      tft.setTextColor(ST7735_WHITE);
      tft.print("MORE GAS");
      tft.setCursor(5, 75);
      tft.setFont(&FreeSansBoldOblique9pt7b);
      tft.setTextColor(ST7735_WHITE);
      tft.print("LESS BRAKE");
      delay(1000);
      tft.fillScreen(ST7735_BLACK);
      SPI.endTransaction();
  } else if (initialRead == 0) {
      SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
      tft.fillScreen(ST7735_BLACK);
      tft.setCursor(50, 45);
      tft.setFont(&FreeSansBoldOblique9pt7b);
      tft.setTextColor(ST7735_YELLOW);
      tft.print("MORE GAS");
      tft.setCursor(5, 75);
      tft.setFont(&FreeSansBoldOblique9pt7b);
      tft.setTextColor(ST7735_YELLOW);
      tft.print("LESS BRAKE");
      delay(1000);
      tft.fillScreen(ST7735_BLACK);
      SPI.endTransaction();
  } else if (initialRead == -1) {
      SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
      tft.fillScreen(ST7735_BLACK);
      tft.setCursor(50, 45);
      tft.setFont(&FreeSansBoldOblique9pt7b);
      tft.setTextColor(ST7735_RED);
      tft.print("MORE GAS");
      tft.setCursor(5, 75);
      tft.setFont(&FreeSansBoldOblique9pt7b);
      tft.setTextColor(ST7735_RED);
      tft.print("LESS BRAKE");
      delay(1000);
      tft.fillScreen(ST7735_BLACK);
      SPI.endTransaction();
  }
}

void tftSmallDefaultText(String TextP, String TextC,int marginLeft, int marginTop, uint16_t color) {
  //SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  tft.setFont(NULL);
  tft.setTextSize(1);
  tft.setCursor(marginLeft, marginTop);
  tft.setTextColor(ST7735_BLACK);
  tft.print(TextP);
  tft.setCursor(marginLeft, marginTop);
  tft.setTextColor(color);
  tft.print(TextC);
  //SPI.endTransaction();
}

void tftBigDefaultText(String TextP, String TextC,int marginLeft, int marginTop, uint16_t color) {
  //SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  tft.setFont(&FreeMono9pt7b);
  tft.setCursor(marginLeft, marginTop);
  tft.setTextColor(ST7735_BLACK);
  tft.print(TextP);
  tft.setCursor(marginLeft, marginTop);
  tft.setTextColor(color);
  tft.print(TextC);
  //SPI.endTransaction();
}

void tftSmallNormalText(String TextP, String TextC,int marginLeft, int marginTop, uint16_t color) {
  //SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  tft.setFont(&FreeSansBoldOblique9pt7b);
  tft.setCursor(marginLeft, marginTop);
  tft.setTextColor(ST7735_BLACK);
  tft.print(TextP);
  tft.setCursor(marginLeft, marginTop);
  tft.setTextColor(color);
  tft.print(TextC);
  //SPI.endTransaction();
}

void tftBigText(String TextP, String TextC,int marginLeft, int marginTop, uint16_t color) {
  //SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  tft.setFont(&FreeSansBoldOblique24pt7b);
  tft.setCursor(marginLeft, marginTop);
  tft.setTextColor(ST7735_BLACK);
  tft.print(TextP);
  tft.setCursor(marginLeft, marginTop);
  tft.setTextColor(color);
  tft.print(TextC);
  //SPI.endTransaction();
}

float customAbs(float myValue) {
  float vtoreturn = 0;
  if (myValue < 0) {
    myValue = myValue * -1;
  }
  return myValue;
}

float cCGC(float gpsCoor) {
  float gcoor = 0;
  
  int DD = gpsCoor/100;
  float mm = ((gpsCoor / 100 - DD) * 100) / 60;
  gcoor = DD+mm;

  return gcoor;
}

void cBrake() {
  if (gpsRawData[1].length() > 7 && gpsRawData[1].length() < 12) {
    long currtime = (long) (gpsRawData[1].toFloat() * 1000);
    if (currtime > gpsRawTime) {
      gpsRawTime = currtime;
      gpsHour = (int) (gpsRawTime / 10000000);
      gpsMinute = (int) ((gpsRawTime % 10000000) / 100000);
      gpsSeconds = (int) (((gpsRawTime % 10000000) % 100000) / 1000);
      gpsMilliseconds = (int) (gpsRawTime % 1000);
    }
  }
  if (gpsRawData[2].length() > 7 && gpsRawData[2].length() < 10) {
    inlat = gpsRawData[2].toFloat();
    if (inlat != gclat && inlat != 0) {
      gclat = cCGC(inlat);
    }
  }
  if (gpsRawData[4].length() > 7 && gpsRawData[2].length() < 10) {
    inlon = gpsRawData[4].toFloat();
    if (inlon != gclon && inlon != 0) {
      gclon = cCGC(inlon);
    }
  }
  if (gpsRawData[7].length() > 0) {
    int insats = gpsRawData[7].toInt();
    if (insats != gpsSats) {
      gpsSats = insats;
    }
  }
  if (gpsRawData[9].length() > 0) {
    float inalt = gpsRawData[9].toFloat();
    if (inalt != gpsAlt) {
      gpsAlt = inalt;
    }
  }
}

void tftTOPBAR() {
    tftSats = "";
    tftSats += "F ";
    tftSats += String(gpsSats);
    tftSats +="  ||";

SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  if (inTopScreen == 0) {
    tft.fillRect(0, 0, 160, 20, ST7735_BLACK);
    if (gpsSats == 0) {
      tftSmallDefaultText(tftprSats, tftSats, 5, 5, ST7735_DARKGRAY);
    } else {
      tftSmallDefaultText(tftprSats, tftSats, 5, 5, ST7735_GREEN);
    }
    if (writeCounter == 0) {
      tftSmallDefaultText("W", "W", 85, 5, ST7735_WHITE);
    } else {
      tftSmallDefaultText("W", "W", 85, 5, ST7735_GREEN);
    }
    if (initialRead == 2) {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_GREEN);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_GREEN);
    } else if (initialRead == 1) {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_GREEN);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_RED);
    } else if (initialRead == 0) {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_RED);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_GREEN);
    } else {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_RED);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_RED);
      tftSmallDefaultText("W", "W", 85, 5, ST7735_RED);
    }
    if (trackp == -1) {
      tftSmallDefaultText(tftprtrpos, tfttrpos, 100, 5, ST7735_DARKGRAY);
    } else if(trackp == 0) {
      tftSmallDefaultText(tftprtrpos, tfttrpos, 100, 5, ST7735_DARKGRAY);
    } else {
      tftSmallDefaultText(tftprtrpos, tfttrpos, 100, 5, ST7735_WHITE);
    }
  }

  if (!tftprSats.equals(tftSats) || !tftprtrpos.equals(tfttrpos)) {
    tft.fillRect(0, 0, 160, 20, ST7735_BLACK);
    if (gpsSats == 0) {
      tftSmallDefaultText(tftprSats, tftSats, 5, 5, ST7735_DARKGRAY);
    } else {
      tftSmallDefaultText(tftprSats, tftSats, 5, 5, ST7735_GREEN);
    }
    if (writeCounter == 0) {
      tftSmallDefaultText("W", "W", 85, 5, ST7735_WHITE);
    } else {
      tftSmallDefaultText("W", "W", 85, 5, ST7735_GREEN);
    }
    if (initialRead == 2) {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_GREEN);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_GREEN);
    } else if (initialRead == 1) {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_GREEN);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_RED);
    } else if (initialRead == 0) {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_RED);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_GREEN);
    } else {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_RED);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_RED);
      tftSmallDefaultText("W", "W", 85, 5, ST7735_RED);
    }
    if (trackp == -1) {
      tftSmallDefaultText(tftprtrpos, tfttrpos, 100, 5, ST7735_DARKGRAY);
    } else if(trackp == 0) {
      tftSmallDefaultText(tftprtrpos, tfttrpos, 100, 5, ST7735_DARKGRAY);
    } else {
      tftSmallDefaultText(tftprtrpos, tfttrpos, 100, 5, ST7735_WHITE);
    }
  }

  if (!tftSats.equals(tftprSats)) {
    if (gpsSats == 0) {
      tftSmallDefaultText(tftprSats, tftSats, 5, 5, ST7735_DARKGRAY);
    } else {
      tftSmallDefaultText(tftprSats, tftSats, 5, 5, ST7735_GREEN);
    }
  }
  tftprSats = tftSats;

  tftinr = String(initialRead);
  tftwc = String(writeCounter);
  if(!tftwc.equals(tftprwc)) {
    if (writeCounter == 0) {
      tftSmallDefaultText("W", "W", 85, 5, ST7735_WHITE);
    } else {
      tftSmallDefaultText("W", "W", 85, 5, ST7735_GREEN);
    }
  }
  tftprwc = tftwc;
  if (!tftinr.equals(tftprinr)) {
    if (initialRead == 2) {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_GREEN);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_GREEN);
    } else if (initialRead == 1) {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_GREEN);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_RED);
    } else if (initialRead == 0) {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_RED);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_GREEN);
    } else {
      tftSmallDefaultText("R", "R", 55, 5, ST7735_RED);
      tftSmallDefaultText("G", "G", 70, 5, ST7735_RED);
      tftSmallDefaultText("W", "W", 85, 5, ST7735_RED);
    }
  }
  tftprinr = tftinr;
  tftprtrpos = tfttrpos;

SPI.endTransaction();
  inTopScreen = 1;
}

void tftLAPTIMES() {
  tftlapSec = String((int) ((((lapTime % 86400000) % 3600000) % 60000) / 1000));
  tftlapMills = String((int) ((((lapTime % 86400000) % 3600000) % 60000) % 1000 / 100));
  tftlapMins = String((int) (((lapTime % 86400000) % 3600000) / 60000));
  tftsp1 = String(split1 / 1000);
  tftsp2 = String(split2 / 1000);
  tftbl = lapTimeConversion(bestlap);
  tftdelta = String(DLAP);
  tftcurrentlap = String(lapCounterDown + lapCounterUp);
  tftupordown = UpORDown;
  tftsp1dif = String(DSBS1);
  tftsp2dif = String(DSBS2);
  if (split1 == 0) {
    tftsp1 = "-";
  }
  if (split2 == 0) {
    tftsp2 = "-";
  }
  if (bestlap == 0) {
    tftbl = "-";
  }
SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  if (!tftlapSec.equals(tftprlapSec)) {
    tftSmallNormalText(tftprlapMins,tftlapMins,20,55,ST7735_WHITE);
  }
  if (!tftlapSec.equals(tftprlapSec)) {
    tftBigText(tftprlapSec,tftlapSec,46,55,ST7735_WHITE);
  } 
  if (!tftlapMills.equals(tftprlapMills)) {
    tftSmallNormalText(tftprlapMills,tftlapMills,110,55,ST7735_WHITE);
  }
  if (!tftprsp1.equals(tftsp1)) {
    if (DSBLS1 < 0) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TD05);
    } else if (DSBLS1 < 0 && DSBLS1 > -0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TD1);
    } else if (DSBLS1 <= 0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TD2);
    } else if (DSBLS1 > 0) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TU05);
    } else if (DSBLS1 > 0 && DSBLS1 < 0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TU1);
    } else if (DSBLS1 >= 0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_WHITE);
    } else {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TU2);
    }
    //---Delta
    tft.fillRect(85, 75, 70, 35, ST7735_BLACK);
    tftSmallNormalText(tftprsp1dif,tftsp1dif,100,95,ST7735_WHITE);
    tftSmallDefaultText("Delta","Delta",103,100,ST7735_WHITE);
  }
  if (!tftprsp2.equals(tftsp2)) {
    if (DSBLS2 < 0) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TD05);
    } else if (DSBLS2 < 0 && DSBLS2 > -0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TD1);
    } else if (DSBLS2 <= 0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TD2);
    } else if (DSBLS2 > 0) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TU05);
    } else if (DSBLS2 > 0 && DSBLS2 < 0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TU1);
    } else if (DSBLS2 >= 0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TU2);
    } else {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_WHITE);
    }
    //---Delta
    tft.fillRect(85, 75, 70, 35, ST7735_BLACK);
    tftSmallNormalText(tftprsp2dif,tftsp2dif,100,95,ST7735_WHITE);
    tftSmallDefaultText("Delta","Delta",103,100,ST7735_WHITE);
  }
  if (inLapScreen == 0) {
    tftSmallDefaultText("SP_1:", "SP_1:",5,70,ST7735_WHITE);
    tftSmallDefaultText("SP_2:", "SP_2:",5,85,ST7735_WHITE);
    tftSmallNormalText(":",":",38,55,ST7735_WHITE);
    tftSmallNormalText(".",".",105,55,ST7735_WHITE);
    tftSmallDefaultText("BL:","BL:",5,115,ST7735_WHITE);
    //---Delta
    tft.fillRect(85, 75, 70, 35, ST7735_BLACK);
    tftSmallNormalText("N/A","N/A",100,95,ST7735_WHITE);
    tftSmallDefaultText("Delta","Delta",103,100,ST7735_WHITE);
    //----NormalValues First Time Refreshing
    tft.fillRect(85, 75, 70, 35, ST7735_BLACK);
    tftSmallNormalText(tftprlapMins,tftlapMins,20,55,ST7735_WHITE);
    tftBigText(tftprlapSec,tftlapSec,46,55,ST7735_WHITE);
    tftSmallNormalText(tftprlapMills,tftlapMills,110,55,ST7735_WHITE);
    tftSmallNormalText(tftprsp1dif,tftsp1dif,100,95,ST7735_WHITE);
    tftSmallDefaultText(tftprupordown, tftupordown,25,100,ST7735_WHITE);
    tftSmallDefaultText(tftprcurrentlap, tftcurrentlap,5,100,ST7735_WHITE);
    tftSmallNormalText(tftprdelta,tftdelta,100,95,ST7735_WHITE);
    tftSmallDefaultText("Delta","Delta",103,100,ST7735_WHITE);
    tftSmallDefaultText(tftprbl, tftbl,45,115,ST7735_WHITE);
    tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_WHITE);
    tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_WHITE);
    tftSmallNormalText("00","00",120,55,ST7735_WHITE); //To efera katw giati bgainei moutzoura
  }
  if (!tftcurrentlap.equals(tftprcurrentlap)) {
    tftSmallDefaultText(tftprupordown, tftupordown,25,100,ST7735_WHITE);
    tftSmallDefaultText(tftprcurrentlap, tftcurrentlap,5,100,ST7735_WHITE);
    //---Delta
    tft.fillRect(85, 75, 70, 35, ST7735_BLACK);
    tftSmallNormalText(tftprdelta,tftdelta,100,95,ST7735_WHITE);
    tftSmallDefaultText("Delta","Delta",103,100,ST7735_WHITE);
  }
  if (!tftprbl.equals(tftbl)) {
    tftSmallDefaultText(tftprbl, tftbl,45,115,ST7735_WHITE);
  }
SPI.endTransaction();

  tftprlapSec = tftlapSec;
  tftprlapMills = tftlapMills;
  tftprlapMins = tftlapMins;
  tftprsp1 = tftsp1;
  tftprsp2 = tftsp2;
  tftprbl = tftbl;
  tftprdelta = tftdelta;
  tftprcurrentlap = tftcurrentlap;
  tftprupordown = tftupordown;
  tftprsp1dif = tftsp1dif;
  tftprsp2dif = tftsp2dif;

  inLapScreen = 1;
}

void tftGHOST() {
  tftlapSec = String((int) ((((lapTime % 86400000) % 3600000) % 60000) / 1000));
  tftlapMills = String((int) ((((lapTime % 86400000) % 3600000) % 60000) % 1000 / 100));
  tftlapMins = String((int) (((lapTime % 86400000) % 3600000) / 60000));
  tftsp1 = String(split1 / 1000);
  tftsp2 = String(split2 / 1000);
  tftdelta = String(DLAP);
  tftsp1dif = String(DSBS1);
  tftsp2dif = String(DSBS2);
  tftwtsay = whattosay;
  if (whattosay.equals("")) {
    tftwtsay = "Push Push Push";
  }
  if (split1 == 0) {
    tftsp1 = "-";
  }
  if (split2 == 0) {
    tftsp2 = "-";
  }
  if (bestlap == 0) {
    tftbl = "-";
  }
SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  if (!tftlapSec.equals(tftprlapSec)) {
    tftSmallNormalText(tftprlapMins,tftlapMins,20,55,ST7735_WHITE);
  }
  if (!tftlapSec.equals(tftprlapSec)) {
    tftBigText(tftprlapSec,tftlapSec,46,55,ST7735_WHITE);
  } 
  if (!tftlapMills.equals(tftprlapMills)) {
    tftSmallNormalText(tftprlapMills,tftlapMills,110,55,ST7735_WHITE);
  }
  if (inGhostScreen == 0) {
    tft.fillRect(85, 75, 70, 35, ST7735_BLACK);
    tftSmallNormalText(tftprlapMins,tftlapMins,20,55,ST7735_WHITE);
    tftBigText(tftprlapSec,tftlapSec,46,55,ST7735_WHITE);
    tftSmallNormalText(tftprlapMills,tftlapMills,110,55,ST7735_WHITE);
    tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_WHITE);
    tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_WHITE);
    tftSmallDefaultText("SP_1:", "SP_1:",5,70,ST7735_WHITE);
    tftSmallDefaultText("SP_2:", "SP_2:",5,85,ST7735_WHITE);
    tftSmallNormalText(tftprwtsay,tftwtsay,5,120,ST7735_WHITE);
    tftSmallNormalText("00","00",120,55,ST7735_WHITE); //To efera katw giati bgainei moutzoura
  }
  if (!tftprsp1.equals(tftsp1)) {
    if (DSBLS1 < 0) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TD05);
    } else if (DSBLS1 < 0 && DSBLS1 > -0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TD1);
    } else if (DSBLS1 <= 0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TD2);
    } else if (DSBLS1 > 0) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TU05);
    } else if (DSBLS1 > 0 && DSBLS1 < 0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TU1);
    } else if (DSBLS1 >= 0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_WHITE);
    } else {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TU2);
    }
  }
  if (!tftprsp2.equals(tftsp2)) {
    if (DSBLS2 < 0) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TD05);
    } else if (DSBLS2 < 0 && DSBLS2 > -0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TD1);
    } else if (DSBLS2 <= 0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TD2);
    } else if (DSBLS2 > 0) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TU05);
    } else if (DSBLS2 > 0 && DSBLS2 < 0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TU1);
    } else if (DSBLS2 >= 0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TU2);
    } else {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_WHITE);
    }
  }
  if (!tftcurrentlap.equals(tftprcurrentlap)) {
    tftSmallDefaultText(tftprupordown, tftupordown,25,100,ST7735_WHITE);
    tftSmallDefaultText(tftprcurrentlap, tftcurrentlap,5,100,ST7735_WHITE);
  }
  if (!tftprbl.equals(tftbl)) {
    tftSmallDefaultText(tftprbl, tftbl,45,115,ST7735_WHITE);
  }
  if(!tftwtsay.equals(tftprwtsay)) {
    tftSmallNormalText(tftprwtsay,tftwtsay,5,120,ST7735_WHITE);
  }

SPI.endTransaction();

  tftprlapSec = tftlapSec;
  tftprlapMills = tftlapMills;
  tftprlapMins = tftlapMins;
  tftprsp1 = tftsp1;
  tftprsp2 = tftsp2;
  tftprsp1dif = tftsp1dif;
  tftprsp2dif = tftsp2dif;
  tftprwtsay = tftwtsay;

  inGhostScreen = 1;
}

void tftLAPALPHA() {
//lapTime = millis();
  tftlapSec = String((int) ((((lapTime % 86400000) % 3600000) % 60000) / 1000));
  tftlapMills = String((int) ((((lapTime % 86400000) % 3600000) % 60000) % 1000 / 100));
  tftlapMins = String((int) (((lapTime % 86400000) % 3600000) / 60000));
  tftsp1 = String(split1 / 1000);
  tftsp2 = String(split2 / 1000);
  tftbl = lapTimeConversion(bestlap);
  tftdelta = String(DLAP);
  tftcurrentlap = String(lapCounterDown + lapCounterUp);
  tftupordown = UpORDown;
  tftsp1dif = String(DSBS1);
  tftsp2dif = String(DSBS2);
  if (split1 == 0) {
    tftsp1 = "-";
  }
  if (split2 == 0) {
    tftsp2 = "-";
  }
  if (bestlap == 0) {
    tftbl = "-";
  }

SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  if (!tftlapSec.equals(tftprlapSec)) {
    tftSmallNormalText(tftprlapMins,tftlapMins,20,55,ST7735_WHITE);
  }
  if (!tftlapSec.equals(tftprlapSec)) {
    tftBigText(tftprlapSec,tftlapSec,46,55,ST7735_WHITE);
  } 
  if (!tftlapMills.equals(tftprlapMills)) {
    tftSmallNormalText(tftprlapMills,tftlapMills,110,55,ST7735_WHITE);
  }
  if (angleCounterMills + 100 < millis()) {
  tftalpha = String(NN_Alpha, 2);
  if (NN_Alpha > 0.9) {
    tftSmallNormalText(tftpralpha,tftalpha,100,95,ST7735_WHITE);
  }else if (NN_Alpha <= 0.9 && NN_Alpha > 0.7) {
    tftSmallNormalText(tftpralpha,tftalpha,100,95,ST7735_CONFIDENT);
  }else if (NN_Alpha <= 0.7 && NN_Alpha > 0.5) {
    tftSmallNormalText(tftpralpha,tftalpha,100,95,ST7735_NORMAL);
  }else if (NN_Alpha <= 0.5 && NN_Alpha > 0.3) {
    tftSmallNormalText(tftpralpha,tftalpha,100,95,ST7735_AGRESSIVE);
  }else if (NN_Alpha <= 0.1 && NN_Alpha > 0) {
    tftSmallNormalText(tftpralpha,tftalpha,100,95,ST7735_HARD);
  } else {
    tftSmallNormalText(tftpralpha,tftalpha,100,95,ST7735_GLHF);
  }
  angleCounterMills = millis();
  }
  if (inAlphaScreen == 0) {
    tftSmallNormalText("00","00",120,55,ST7735_WHITE);
    tftSmallDefaultText("SP_1:", "SP_1:",5,70,ST7735_WHITE);
    tftSmallDefaultText("SP_2:", "SP_2:",5,85,ST7735_WHITE);
    tftSmallNormalText(":",":",38,55,ST7735_WHITE);
    tftSmallNormalText(".",".",105,55,ST7735_WHITE);
    tftSmallDefaultText("BL:","BL:",5,115,ST7735_WHITE);
    //---Delta
    tft.fillRect(85, 75, 70, 35, ST7735_BLACK);
    tftSmallDefaultText("Alpha","Alpha",103,100,ST7735_WHITE);
    //----NormalValues First Time Refreshing
    tftSmallNormalText(tftprlapMins,tftlapMins,20,55,ST7735_WHITE);
    tftBigText(tftprlapSec,tftlapSec,46,55,ST7735_WHITE);
    tftSmallNormalText(tftpralpha,tftalpha,100,95,ST7735_WHITE);
    tftSmallNormalText(tftprlapMills,tftlapMills,110,55,ST7735_WHITE);
    tftSmallDefaultText(tftprupordown, tftupordown,25,100,ST7735_WHITE);
    tftSmallDefaultText(tftprcurrentlap, tftcurrentlap,5,100,ST7735_WHITE);
    tftSmallDefaultText(tftprbl, tftbl,45,115,ST7735_WHITE);
    tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_WHITE);
    tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_WHITE);
    tftSmallNormalText("00","00",120,55,ST7735_WHITE);
  }
  if (!tftprsp1.equals(tftsp1)) {
    if (DSBLS1 < 0) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TD05);
    } else if (DSBLS1 < 0 && DSBLS1 > -0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TD1);
    } else if (DSBLS1 <= 0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TD2);
    } else if (DSBLS1 > 0) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TU05);
    } else if (DSBLS1 > 0 && DSBLS1 < 0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TU1);
    } else if (DSBLS1 >= 0.5) {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_WHITE);
    } else {
      tftSmallDefaultText(tftprsp1, tftsp1,45,70,ST7735_TU2);
    }
  }
  if (!tftprsp2.equals(tftsp2)) {
    if (DSBLS2 < 0) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TD05);
    } else if (DSBLS2 < 0 && DSBLS2 > -0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TD1);
    } else if (DSBLS2 <= 0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TD2);
    } else if (DSBLS2 > 0) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TU05);
    } else if (DSBLS2 > 0 && DSBLS2 < 0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TU1);
    } else if (DSBLS2 >= 0.5) {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_TU2);
    } else {
      tftSmallDefaultText(tftprsp2, tftsp2,45,85,ST7735_WHITE);
    }
  }
  if (!tftcurrentlap.equals(tftprcurrentlap)) {
    tftSmallDefaultText(tftprupordown, tftupordown,25,100,ST7735_WHITE);
    tftSmallDefaultText(tftprcurrentlap, tftcurrentlap,5,100,ST7735_WHITE);
  }
  if (!tftprbl.equals(tftbl)) {
    tftSmallDefaultText(tftprbl, tftbl,45,115,ST7735_WHITE);
  }
SPI.endTransaction();

  tftprlapSec = tftlapSec;
  tftprlapMills = tftlapMills;
  tftprlapMins = tftlapMins;
  tftprsp1 = tftsp1;
  tftprsp2 = tftsp2;
  tftprbl = tftbl;
  tftprdelta = tftdelta;
  tftprcurrentlap = tftcurrentlap;
  tftprupordown = tftupordown;
  tftprsp1dif = tftsp1dif;
  tftprsp2dif = tftsp2dif;
  tftpralpha = tftalpha;

  inAlphaScreen = 1;
}

void tftSyncVid() {
  gpsTime1 = String(gpsHour) + "." +String(gpsMinute) + ":" + String(gpsSeconds);
  gpsTime2 = String(gpsMilliseconds);
SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  tft.fillScreen(ST7735_BLACK);
  tftSmallDefaultText("Sync Video", "Sync Video",45,20,ST7735_TD2);
  tftSmallNormalText(gpsTime1prev,gpsTime1,42,55,ST7735_WHITE); 
  tftBigText(gpsTime2prev,gpsTime2,34,100,ST7735_WHITE);
SPI.endTransaction();
  gpsTime1prev = gpsTime1;
  gpsTime2prev = gpsTime2;

  inSyncVidScreen = 1;
}