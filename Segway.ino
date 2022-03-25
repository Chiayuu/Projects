#include <math.h>
#include <Event.h>
#include <Timer.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>

int motL1 = 6;
int motL2 = 7;
int motL3 = 5;
int motR1 = 8;
int motR2 = 9;
int motR3 = 10;
int motS = 0;
int16_t ax, ay, az;
float Ax,Ay,Az;
int16_t gx, gy, gz;
float Gx,Gy,Gz;
double kp=15,kd=3,kd1=0;
float g = 16384;
int pdval;
double err,lasterr=0;
float timechange = 50;
float angle1,gyro,out;
float derr,errsum,bal=0;
float xacc,yacc,zacc;
float out1,out2;


Timer t;



MPU6050 accelgyro;


void setup() {
  
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  
  accelgyro.initialize();
  t.every(timechange, pid);

  
  pinMode(motL1,OUTPUT);
  pinMode(motL2,OUTPUT);
  pinMode(motL3,OUTPUT);
  pinMode(motR1,OUTPUT);
  pinMode(motR2,OUTPUT);
  pinMode(motR3,OUTPUT);

  

}

void loop() {
  // put your main code here, to run repeatedly:

    t.update();
  
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //Serial.print(ax); Serial.print("\t");
    //Serial.print(ay); Serial.print("\t");
    //Serial.print(az); Serial.print("\t");
    //Serial.print(gx); Serial.print("\t");
    //Serial.print(gy); Serial.print("\t");
    //Serial.print(gz); Serial.print("\n");
    

    xacc = atan2(ax,az) * 180/PI;
    gyro = -gx/131.00;
    filter1(xacc,gyro);
    Serial.print ("AN");
    Serial.print (angle1);
    Serial.print("\n");
    pid(angle1,gyro);
    Serial.print(out);
    Serial.print("\n");

    if (out > 0){
      //正轉
      out1 = (out/530*255+5);
      if (out1>255){
         out1=255;
      }
      if (out < 30){
        out1 = 70;
      }

      Serial.print(out1);
      analogWrite(motL3,out1);
      analogWrite(motR3,out1);
      analogWrite(motL1,0);
      analogWrite(motL2,out1);
      analogWrite(motR1,0);
      analogWrite(motR2,out1);
      
    }
    else if (out < 0){
      //反轉

      out2 = abs(out/570*255)+5;
      if (out2>255){
        out2 = 255;
      }
      if (out > -30){
        out2 = 70;
      }

      Serial.print(out2);
      analogWrite(motL3,out2);
      analogWrite(motR3,out2);
      analogWrite(motL1,out2);
      analogWrite(motL2,0);
      analogWrite(motR1,out2);
      analogWrite(motR2,0);
      
    }

    delay(100);

}

float K1 =0.85; // 对加速度计取值的权重

float dt=20*0.001;//注意：dt的取值为滤波器采样时间




void filter1(float angle_m, float gyro_m)//采集后计算的角度和角加速度

{

angle1 = K1 * angle_m+ (1-K1) * (angle1 + gyro_m * dt);

return angle1;

}

void pid (float angle1,float gyr0)

{

float err = bal - angle1;
if (abs(err)<2){
  err = 0;
}

errsum += (err * timechange);

float derr = (err - lasterr) / timechange;

out = kp * err  + kd * gyro + kd1 * derr ;
//Serial.print ("OUT");
//Serial.print (out);
//Serial.print("\n");

lasterr = err;
return out;
}
