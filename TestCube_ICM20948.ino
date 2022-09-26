#include <M5Core2.h>

#define FLT_MIN -3.4028234663852886e+38
#define FLT_MAX  3.4028234663852886e+38

#include <Teensy-ICM-20948.h>

TeensyICM20948 icm20948;

TeensyICM20948Settings icmSettings =
{
  .cs_pin = 27,                  // SPI chip select pin
  .spi_speed = 7000000,           // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                     // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = false,      // Enables gyroscope output
  .enable_accelerometer = false,  // Enables accelerometer output
  .enable_magnetometer = false,   // Enables magnetometer output
  .enable_quaternion = true,     // Enables quaternion output
  .gyroscope_frequency = 1,      // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 1,  // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 1,   // Max frequency = 70, min frequency = 1
  .quaternion_frequency = 225     // Max frequency = 225, min frequency = 50
};

#define LGFX_AUTODETECT
#include <LovyanGFX.hpp>

static LGFX lcd;
static LGFX_Sprite sprite[2];

//#pragma GCC optimize ("O3")
struct point3df{ float x, y, z;};
struct surface{ uint8_t p[4]; int16_t z;};
#define U  140         // size of cube
 
struct point3df cubef[8] ={ // cube edge length is 2*U
  { -U, -U,  U },
  {  U, -U,  U },
  {  U, -U, -U },
  { -U, -U, -U },
  { -U,  U,  U },
  {  U,  U,  U },
  {  U,  U, -U },
  { -U,  U, -U },
};
 
struct surface s[6] = {// define the surfaces
  { {0, 1, 2, 3}, 0 }, // bottom
  { {4, 5, 6, 7}, 0 }, // top
  { {0, 1, 5, 4}, 0 }, // back
  { {3, 7, 6, 2}, 0 }, // front
  { {1, 2, 6, 5}, 0 }, // right
  { {0, 3, 7, 4}, 0 }, // left
};
 
struct point3df cubef2[8];

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

double pitch = 0.0F;
double roll  = 0.0F;
double yaw   = 0.0F;

bool flip;
uint32_t pre_show_time = 0;
unsigned int pre_time = 0;

int ws;
int hs;

void rotate_cube_xyz( float roll, float pitch, float yaw){
  uint8_t i;

  float DistanceCamera = 300;
  float DistanceScreen = 550;

  float cosyaw   = cos(yaw);
  float sinyaw   = sin(yaw);
  float cospitch = cos(pitch);
  float sinpitch = sin(pitch);
  float cosroll  = cos(roll);
  float sinroll  = sin(roll);

  float sinyaw_sinroll = sinyaw * sinroll;
  float sinyaw_cosroll = sinyaw * cosroll;
  float cosyaw_sinroll = cosyaw * sinroll;
  float cosyaw_cosroll = cosyaw * cosroll;

  float x_x = cosyaw * cospitch;
  float x_y = cosyaw_sinroll * sinpitch - sinyaw_cosroll;
  float x_z = cosyaw_cosroll * sinpitch + sinyaw_sinroll;

  float y_x = sinyaw * cospitch;
  float y_y = sinyaw_sinroll * sinpitch + cosyaw_cosroll;
  float y_z = sinyaw_cosroll * sinpitch - cosyaw_sinroll;

  float z_x = -sinpitch;
  float z_y = cospitch * sinroll;
  float z_z = cospitch * cosroll;

  for (i = 0; i < 8; i++){
    float x = x_x * cubef[i].x
            + x_y * cubef[i].y
            + x_z * cubef[i].z;
    float y = y_x * cubef[i].x
            + y_y * cubef[i].y
            + y_z * cubef[i].z;
    float z = z_x * cubef[i].x
            + z_y * cubef[i].y
            + z_z * cubef[i].z;

    cubef2[i].x = (x * DistanceCamera) / (z + DistanceCamera + DistanceScreen) + (ws>>1);
    cubef2[i].y = (y * DistanceCamera) / (z + DistanceCamera + DistanceScreen) + (hs>>1);
    cubef2[i].z = z;
  }
}


void rotate_cube_quaternion(float a, float b, float c, float d)
{
  uint8_t i;

  float DistanceCamera = 300;
  float DistanceScreen = 550;

  float x_x = a*a + b*b - c*c - d*d;
  float x_y = 2*b*c + 2*a*d;
  float x_z = 2*b*d - 2*a*c;
  float y_x = 2*b*c - 2*a*d;
  float y_y = a*a - b*b + c*c - d*d;
  float y_z = 2*c*d + 2*a*b;
  float z_x = 2*b*d + 2*a*c;
  float z_y = 2*c*d - 2*a*b;
  float z_z = a*a - b*b - c*c + d*d;

  for (i = 0; i < 8; i++){
    float x = x_x * cubef[i].x
            + x_y * cubef[i].y
            + x_z * cubef[i].z;
    float y = y_x * cubef[i].x
            + y_y * cubef[i].y
            + y_z * cubef[i].z;
    float z = z_x * cubef[i].x
            + z_y * cubef[i].y
            + z_z * cubef[i].z;

    cubef2[i].x = (x * DistanceCamera) / (z + DistanceCamera + DistanceScreen) + (hs>>1);
    cubef2[i].y = (y * DistanceCamera) / (z + DistanceCamera + DistanceScreen) + (ws>>1);
    cubef2[i].z = z;
  }
}

void setup(void){ 
  M5.begin();

  icm20948.init(icmSettings);

  lcd.init();

  lcd.setRotation(1);


// バックライトの輝度を 0～255 の範囲で設定します。
//  lcd.setBrightness(80);

  lcd.setColorDepth(16);  // RGB565の16ビットに設定

  lcd.fillScreen(0);

  //ws = lcd.width();
  //hs = lcd.height();
  ws = 160;
  hs = 160;
  
  sprite[0].createSprite(ws,hs);
  sprite[1].createSprite(ws,hs);
  lcd.startWrite();
  lcd.fillScreen(TFT_DARKGREY);
  lcd.endWrite();
}

float smoothMove(float dst, float src)
{
  if (     dst + M_PI < src) src -= M_PI * 2;
  else if (src + M_PI < dst) src += M_PI * 2;
  return (dst + src * 19.0) / 20.0;
}

//https://www.kazetest.com/vcmemo/quaternion/quaternion.htm
void QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
                             double& roll, double& pitch, double& yaw)
{
    double q0q0 = q0 * q0;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;
    roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
    pitch = asin(2.0 * (q0q2 - q1q3));
    yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
}

void loop(void)
{
  char sensor_string_buff[128];
  float quat_w, quat_x, quat_y, quat_z;
  icm20948.task();

  Serial.println(sensor_string_buff);
  if (icm20948.quatDataIsReady())
  {
    icm20948.readQuatData(&quat_w, &quat_x, &quat_y, &quat_z);
    sprintf(sensor_string_buff, "{\"quat_w\":%f, \"quat_x\":%f, \"quat_y\":%f, \"quat_z\":%f}", quat_w, quat_x, quat_y, quat_z);
    Serial.println(sensor_string_buff);
  }

  //pitch = smoothMove(filter->getPitch(), pitch);
  //roll  = smoothMove(filter->getRoll() , roll );
  //yaw   = smoothMove(filter->getYaw()  , yaw  );

  //pitch = filter->getPitch()-offset_pitch;
  //roll  = filter->getRoll()-offset_roll;
  //yaw   = filter->getYaw()-offset_yaw;

  //rotate_cube_xyz(roll,pitch,yaw);
  rotate_cube_quaternion(quat_w, quat_x, quat_y, quat_z);

  //描写する面の順番に並び替え
  int ss[6]={0,1,2,3,4,5};
  float sf[6]={0};
  for (int i = 0; i < 6; i++)
  {
    float wz = 0;
    for(int j=0;j<4;j++){
      wz += cubef2[s[i].p[j]].z;
    }
    sf[i] = wz;
  }
  //交換ソート
  for (int j = 5; j > 0; j--){
    for (int i = 0; i < j; i++)
    {
        if(sf[i] < sf[i+1])
        {
          float work = sf[i];
          sf[i] = sf[i+1];
          sf[i+1] = work;
          
          int iw = ss[i];
          ss[i] = ss[i+1];
          ss[i+1] = iw;
        }
    }
  }

  flip = !flip;
  lcd.startWrite();
  sprite[flip].clear();
  for (int i = 0; i < 8; i++)
  {
    //sprite[flip].drawRect( (int)cubef2[i].x-2, (int)cubef2[i].y-2, 4, 4 , 0xF000);
    sprite[flip].drawRect( (int)cubef2[i].y-2, (int)cubef2[i].x-2, 4, 4 , 0xF000);
    //Serial.printf("%d,%f,%f,\r\n",i,cubef2[i].x, cubef2[i].y); 
  }

  for (int i = 0; i < 6; i++)
  {
    int ii = ss[i];
    
    sprite[flip].setColor( lcd.color565(((( ii + 1) &  1)      * 255),
                                  ((((ii + 1) >> 1) & 1) * 255),
                                  ((((ii + 1) >> 2) & 1) * 255)
                   )             );
//    sprite[flip].fillTriangle(    cubef2[s[ii].p[0]].x, cubef2[s[ii].p[0]].y,
//                            cubef2[s[ii].p[1]].x, cubef2[s[ii].p[1]].y,
//                            cubef2[s[ii].p[2]].x, cubef2[s[ii].p[2]].y);
//    sprite[flip].fillTriangle(    cubef2[s[ii].p[2]].x, cubef2[s[ii].p[2]].y,
//                            cubef2[s[ii].p[3]].x, cubef2[s[ii].p[3]].y,
//                            cubef2[s[ii].p[0]].x, cubef2[s[ii].p[0]].y);
      sprite[flip].fillTriangle(    cubef2[s[ii].p[0]].y, cubef2[s[ii].p[0]].x,
                            cubef2[s[ii].p[1]].y, cubef2[s[ii].p[1]].x,
                            cubef2[s[ii].p[2]].y, cubef2[s[ii].p[2]].x);
      sprite[flip].fillTriangle(    cubef2[s[ii].p[2]].y, cubef2[s[ii].p[2]].x,
                            cubef2[s[ii].p[3]].y, cubef2[s[ii].p[3]].x,
                            cubef2[s[ii].p[0]].y, cubef2[s[ii].p[0]].x);
  }
  
  int show_time = millis() - pre_show_time;
  pre_show_time = millis();
  sprite[flip].setCursor(0, 50);
  sprite[flip].printf("%5d\n",show_time);

  QuaternionToEulerAngles((double)quat_w, (double)quat_x, (double)quat_y, (double)quat_z, roll, pitch, yaw);
  sprite[flip].setCursor(0, 70);
  sprite[flip].printf("%3.2f\n",roll*180.0/PI);
  sprite[flip].setCursor(0, 90);
  sprite[flip].printf("%3.2f\n",pitch*180.0/PI);
  sprite[flip].setCursor(0, 110);
  sprite[flip].printf("%3.2f\n",yaw*180.0/PI);
  
  sprite[flip].pushSprite(&lcd, 0, 0);
  lcd.endWrite();
}
