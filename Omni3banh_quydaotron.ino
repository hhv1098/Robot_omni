/*
   //////////----van toc dong cơ----///////////////////
*/
#include <TimerOne.h>
#define pi 3.1415927410
double v1_d, v2_d, v3_d;
double maxV;
int out1, out2, out3;
long err_xung1 = 0, xung_hientai1 = 0, pre_xung1 = 0, pre_out1;
int err1, pre_err1 ;
float khauP1, khauI1, khauD1;
int Vtoc1 = 0;
float kp1 = 3, ki1 = 0.02, kd1 = 2;
//---------****** Động cơ 2  ********------------//
long err_xung2 = 0, xung_hientai2 = 0, pre_xung2 = 0, pre_out2;
int err2, pre_err2 ;
float khauP2, khauI2, khauD2;
int Vtoc2 = 0;
float kp2 = 3, ki2 = 0.02, kd2 = 2;
//----------******động cơ 3 ********----------//
long err_xung3 = 0, xung_hientai3 = 0, pre_xung3 = 0, pre_out3;
int err3, pre_err3 ;
float khauP3, khauI3, khauD3;
int Vtoc3 = 0;
float kp3 = 3, ki3 = 0.02, kd3 = 2;
//--------------------//
const int vanToc_max = 170;
//******************************************//
float T = 0.08;
int xungVong = 972;
/*
   //----------------END van tốc -------------------------//
*/
#include <string.h>
String x, y, z;
String data = "";
unsigned long time, thoigian;
char buffer[15] = {}; // Biến đệm lưu giá trị
/*
   ------------END---- doc cam bien sieu am------------//
*/
///////////////////////
#include <Wire.h>
#include <math.h>
int xRobot_dat  , yRobot_dat  , x_dattam, y_dattam;
int x_thuc , y_thuc, x_thuctam , y_thuctam, xn_mouse, yn_mouse;
double dodai_x = 0;
double dodai_y = 0;

int xRobot_thuc, yRobot_thuc, xBall_thuc, yBall_thuc;
String x_do, y_do, x_xanh, y_xanh, x_mouse, y_mouse;
/*
   -------------- la bàn---------------///////////
*/

double gocdi = 0;
double a, pwm, goc_lech_rad, goc, goc_dat, heading;
long offset, g_min, g_max;
float err_goc;
float hso = 10;
float quangduong, K, K_theta = 0;
float theta, theta_xoay;

boolean hoanthanh = false;  // whether the string is complete

const int nutnhan_goc = 28;
const int reset = 8;
/*
   //------ chân động cơ ------//////
*/
const int in1_1 = 22;  //DC1
const int in2_1 = 23;
const int in1_2 = 25;  //DC2
const int in2_2 = 24;
const int in1_3 = 26;  //DC3
const int in2_3 = 27;
const int pwm_1 = 5;    //PWM DC1
const int pwm_2 = 6;    //PWM DC2
const int pwm_3 = 7;   //PWM DC3
const int maxPWM = 255;     // 0-255
/*
   ------------- end chân-----------------//
   ///// khai bao chan encoder////
*/
#define encoderA_1  2
#define encoderB_1  28
#define encoderA_2  3
#define encoderB_2  29
#define encoderA_3  18
#define encoderB_3  30
long dem_1 = 0, dem_2 = 0, dem_3 = 0;
/*
*/
void tinhtoangoclech();
void hienthi();
void pid_vantoc();
void bodieukhien();
void setup() {
  Serial.begin (115200);
  Serial3.begin(19200);

  delay(5);
  Wire.begin();

  pinMode(in1_1, OUTPUT); pinMode(in2_1, OUTPUT); pinMode(in1_2, OUTPUT); pinMode(in2_2, OUTPUT); pinMode(in1_3, OUTPUT); pinMode(in2_3, OUTPUT); pinMode(pwm_1, OUTPUT);  pinMode(pwm_2, OUTPUT);  pinMode(pwm_3, OUTPUT);

  pinMode(encoderA_1, INPUT_PULLUP);  pinMode(encoderB_1, INPUT_PULLUP);
  pinMode(encoderA_2, INPUT_PULLUP);  pinMode(encoderB_2, INPUT_PULLUP);
  pinMode(encoderA_3, INPUT_PULLUP);  pinMode(encoderB_3, INPUT_PULLUP);

  pinMode(nutnhan_goc, INPUT);
  pinMode(reset, OUTPUT);
  digitalWrite(reset, LOW);

  attachInterrupt(0, DemEncoder_1, FALLING);
  attachInterrupt(1, DemEncoder_2, FALLING);
  attachInterrupt(5, DemEncoder_3, FALLING);
  delay(500);
  Timer1.initialize(80000);
  Timer1.attachInterrupt(docVantoc_dc);
  time = millis();
}

//*********************** main ******************//
void loop()
{
  if ( (unsigned long) (millis() - time) > 5)
  {
    while (Serial3.available() > 0)
    {
      char inChar = (char)Serial3.read();
      data += inChar;
      if (inChar == '\n') {
        hoanthanh = true;
      }
    }
  }

  if (hoanthanh) {
    // !@#$% ^&
    x_do = data.substring(data.indexOf("!") + 1, data.indexOf("@"));
    y_do = data.substring(data.indexOf("@") + 1, data.indexOf("#"));
    x_xanh = data.substring(data.indexOf("#") + 1, data.indexOf("$"));
    y_xanh = data.substring(data.indexOf("$") + 1, data.indexOf("%"));

    x_mouse = data.substring(data.indexOf("%") + 1, data.indexOf("^"));
    y_mouse = data.substring(data.indexOf("^") + 1, data.indexOf("&"));

    xRobot_thuc = x_do.toFloat();
    yRobot_thuc = y_do.toFloat();
    xBall_thuc = x_xanh.toFloat();
    yBall_thuc = y_xanh.toFloat();

    xn_mouse = x_mouse.toFloat();
    yn_mouse = y_mouse.toFloat();

    data = "";
    hoanthanh = false;  // chia 4.248
  }
  thoigian = millis();


  xRobot_dat = 100 + 45 * sin(0.0003 * thoigian);
  yRobot_dat = 75 + 45 * cos(0.0003 * thoigian);
  //  xRobot_dat = 50;
  //  yRobot_dat = 50;
  //  xRobot_dat = xn_mouse;
  //  yRobot_dat = yn_mouse;

  tinhtoangoclech(); // goc lech
  bodieukhien(); pid_vantoc();

  hienthi();

  DC1((int)(out1));
  DC2((int)(out2));
  DC3((int)(out3));
}
void tinhtoangoclech()
{
  float goclech;
  dodai_x = abs(xBall_thuc - xRobot_thuc);
  dodai_y = abs(yBall_thuc - yRobot_thuc);
  if (xBall_thuc > xRobot_thuc)
  {
    if (yBall_thuc > yRobot_thuc) goclech = atan(dodai_y / dodai_x);
    else if (yBall_thuc < yRobot_thuc)  goclech = atan(dodai_x / dodai_y) + 4.71;
    else if (yBall_thuc == yRobot_thuc) goclech = 0;
  }
  else if (xBall_thuc < xRobot_thuc)
  {
    if (yBall_thuc > yRobot_thuc) goclech = atan(dodai_x / dodai_y) + 1.5708;
    else if (yBall_thuc < yRobot_thuc) goclech = atan(dodai_y / dodai_x ) + 3.1416;
    else if (yBall_thuc == yRobot_thuc) goclech = 3.1416;
  }
  else if (xBall_thuc == xRobot_thuc)
  {
    if (yBall_thuc > yRobot_thuc) goclech = 1.57;
    else if (yBall_thuc < yRobot_thuc) goclech = 4.71;
    else if (yBall_thuc == yRobot_thuc) goclech = 0; // luu y
  }
  goc_lech_rad = goclech;
}

void bodieukhien() {

  float theta_xoay  = pi / 2 + goc_lech_rad;
  float errx, erry, pre_errx, pre_erry, pd_x, pd_y, errx_1dot, erry_1dot;
  float Kp_vitri = 5, Kd_vitri = 1;
  float Kp_Theta = 0.3, Kd_Theta = 0;
  float errTheta, thetaDat = 0, thetaHientai, errTheta_1dot, pre_errTheta, pdTheta;

  thetaHientai = goc_lech_rad * 180 / 3.14;
  errTheta = thetaDat - thetaHientai;
  if (errTheta <= -180)
    errTheta = errTheta + 360;
  else if (errTheta >= 180)
    errTheta = 360 - errTheta;
  errTheta_1dot = errTheta - pre_errTheta;
  pdTheta = Kp_Theta * errTheta + Kd_Theta * errTheta_1dot;


  errx = xRobot_dat - xRobot_thuc ;
  erry = yRobot_dat - yRobot_thuc;
  //  errx = 10;
  //  erry = 10;

  errx_1dot = errx - pre_errx;
  erry_1dot = erry - pre_erry;

  pd_x = Kp_vitri * errx + Kd_vitri * errx_1dot;
  pd_y = Kp_vitri * erry + Kd_vitri * erry_1dot;

  // code neural
  float d1, d2, d3, d4, d5, w11, w12, w13, w14, w15, w21, w22, w23, w24, w25, deta1, deta2, deta3, deta4, deta5, m = 10, fx_hat, fy_hat, Kw = 0.5, to_x, to_y;
  float tamC[5] = { -10, -5, 0, 5, 10};



  d1 = (-(sqrt( ((errx - tamC[0]) * (errx - tamC[0])) + ((erry - tamC[0]) * (erry - tamC[0])))) * (sqrt( ((errx - tamC[0]) * (errx - tamC[0])) + ((erry - tamC[0]) * (erry - tamC[0])))) )   /  (2 * m * m);
  d2 = (-(sqrt( ((errx - tamC[1]) * (errx - tamC[1])) + ((erry - tamC[1]) * (erry - tamC[1])))) * (sqrt( ((errx - tamC[1]) * (errx - tamC[1])) + ((erry - tamC[1]) * (erry - tamC[1])))) )   /  (2 * m * m);
  d3 = (-(sqrt( ((errx - tamC[2]) * (errx - tamC[2])) + ((erry - tamC[2]) * (erry - tamC[2])))) * (sqrt( ((errx - tamC[2]) * (errx - tamC[2])) + ((erry - tamC[2]) * (erry - tamC[2])))) )   /  (2 * m * m);
  d4 = (-(sqrt( ((errx - tamC[3]) * (errx - tamC[3])) + ((erry - tamC[3]) * (erry - tamC[3])))) * (sqrt( ((errx - tamC[3]) * (errx - tamC[3])) + ((erry - tamC[3]) * (erry - tamC[3])))) )   /  (2 * m * m);
  d5 = (-(sqrt( ((errx - tamC[4]) * (errx - tamC[4])) + ((erry - tamC[4]) * (erry - tamC[4])))) * (sqrt( ((errx - tamC[4]) * (errx - tamC[4])) + ((erry - tamC[4]) * (erry - tamC[4])))) )   /  (2 * m * m);

  deta1 = exp(d1);
  deta2 = exp(d2);
  deta3 = exp(d3);
  deta4 = exp(d4);
  deta5 = exp(d5);

  w11 = Kw * deta1 * pd_x;
  w12 = Kw * deta2 * pd_x;
  w13 = Kw * deta3 * pd_x;
  w14 = Kw * deta4 * pd_x;
  w15 = Kw * deta5 * pd_x;
  w21 = Kw * deta1 * pd_y;
  w22 = Kw * deta2 * pd_y;
  w23 = Kw * deta3 * pd_y;
  w24 = Kw * deta4 * pd_y;
  w25 = Kw * deta5 * pd_y;

  fx_hat = w11 * deta1 + w12 * deta2 + w13 * deta3 + w14 * deta4 + w15 * deta5;
  fy_hat = w21 * deta1 + w22 * deta2 + w23 * deta3 + w24 * deta4 + w25 * deta5;

  fx_hat = constrain(fx_hat, -40, 40);
  fy_hat = constrain(fy_hat, -40, 40);

  to_x = pd_x + fx_hat;
  to_y = pd_y + fy_hat;

  //  to_x = pd_x ;
  //  to_y = pd_y ;

  // end code neural

  v1_d = to_x * cos(theta_xoay) + to_y * sin(theta_xoay) + pdTheta;
  v2_d = to_x * (-0.5 * cos(theta_xoay) - sqrt(3) * 0.5 * sin(theta_xoay)) + to_y * (-0.5 * sin(theta_xoay) + sqrt(3) * 0.5 * cos(theta_xoay)) + pdTheta ;
  v3_d = to_x * (-0.5 * cos(theta_xoay) + sqrt(3) * 0.5 * sin(theta_xoay)) + to_y * (-0.5 * sin(theta_xoay) - sqrt(3) * 0.5 * cos(theta_xoay)) + pdTheta ;

  //v1_d = 130;
  //v2_d = 130;
  //v3_d = 130;

  maxV = abs(v1_d);
  if (maxV < abs(v2_d))
    maxV = abs(v2_d);
  if (maxV < abs(v3_d))
    maxV = abs(v3_d);

  if (maxV >= 140)
  {
    v1_d = v1_d * vanToc_max / maxV;
    v2_d = v2_d * vanToc_max / maxV;
    v3_d = v3_d * vanToc_max / maxV;
  }


  pre_errx = errx;
  pre_erry = erry;
  pre_errTheta = errTheta;

//  Serial.print("dt1=");
//  Serial.print(deta1);
//  Serial.print(" dt2=");
//  Serial.print(deta2);
//  Serial.print(" dt3=");
//  Serial.print(deta3);
//  Serial.print(" dt4=");
//  Serial.print(deta4);
//  Serial.print(" dt5=");
//  Serial.print(deta5);
//
//  Serial.print(" fx=");
//  Serial.print(fx_hat);
//  Serial.print(" fy=");
//  Serial.println(fy_hat);



}
//------------------------------end di chuyen ---------------------//
//
//------------------------------Tính toán góc HMC8553l---------------------//

//---------------------------end tính góc----------------------//

void docVantoc_dc()
{
  /*
     Đo vận tốc
  */
  xung_hientai1 = dem_1;
  err_xung1 = xung_hientai1 - pre_xung1;
  pre_xung1 = xung_hientai1;
  Vtoc1 = (60 * err_xung1) / (T * xungVong);
  dem_1 = 0; xung_hientai1 = 0; err_xung1 = 0; pre_xung1 = 0;
  //-------------------//
  xung_hientai2 = dem_2;
  err_xung2 = xung_hientai2 - pre_xung2;
  pre_xung2 = xung_hientai2;
  Vtoc2 = (60 * err_xung2) / (T * xungVong );
  dem_2 = 0; xung_hientai2 = 0; err_xung2 = 0; pre_xung2 = 0;
  //-----------------------------//
  xung_hientai3 = dem_3;
  err_xung3 = xung_hientai3 - pre_xung3;
  pre_xung3 = xung_hientai3;
  Vtoc3 = (60 * err_xung3) / (T * xungVong);
  dem_3 = 0; xung_hientai3 = 0; err_xung3 = 0; pre_xung3 = 0;
  /*
     kết thúc đo vân tốc
  */
}
void pid_vantoc()
{

  //----------------- động cơ 1----------------//
  err1 = v1_d - Vtoc1;

  khauP1 = kp1 * err1;
  khauI1 += ki1 * (err1 + pre_err1) * T;
  khauI1 = constrain(khauI1, -100, 100);
  khauD1 = kd1 * (err1 - pre_err1) / T;
  out1 = pre_out1 + khauP1 + khauI1 + khauD1;
  if (out1 >= 255) out1 = 255;
  else if (out1 <= -255) out1 = -255;
  //if(err ==0) {khauI=0;}
  pre_err1 = err1;
  pre_out1 = 0;
  //-----------Động cơ 2----------------------//
  err2 = v2_d - Vtoc2;

  khauP2 = kp2 * err2;
  khauI2 += ki2 * (err2 + pre_err2) * T;
  khauI2 = constrain(khauI2, -100, 100);
  khauD2 = kd2 * (err2 - pre_err2) / T;
  out2 = pre_out2 + khauP2 + khauI2 + khauD2;
  if (out2 >= 255) out2 = 255;
  if (out2 <= -255) out2 = -255;
  //if(err ==0) {khauI=0;}
  pre_err2 = err2;
  pre_out2 = 0;
  //-----------End Động cơ 2----------------------//

  //-----------Động cơ 3----------------------//
  err3 = v3_d - Vtoc3;

  khauP3 = kp3 * err3;
  khauI3 += ki3 * (err3 + pre_err3) * T;
  khauI3 = constrain(khauI3, -100, 100);
  khauD3 = kd3 * (err3 - pre_err3) / T;
  out3 = pre_out3 + khauP3 + khauI3 + khauD3;
  if (out3 >= 255) out3 = 255;
  if (out3 <= -255) out3 = -255;
  //if(err ==0) {khauI=0;}
  pre_err3 = err3;
  pre_out3 = 0;
  //-----------End Động cơ 3----------------------//

}
////----------------------------------------------------------//
void DC1(int pwm) {
  if (pwm >= 0) {
    if (pwm > 255)
      pwm = 255;
    digitalWrite(in1_1, 0);
    digitalWrite(in2_1, 1);
    analogWrite(pwm_1, pwm);
  }
  else {
    pwm = -pwm;
    if (pwm > 255)
      pwm = 255;
    digitalWrite(in1_1, 1);
    digitalWrite(in2_1, 0);
    analogWrite(pwm_1, pwm);
  }
}

void DC2(int pwm) {
  if (pwm >= 0) {
    if (pwm > 255)
      pwm = 255;
    digitalWrite(in1_2, 0);
    digitalWrite(in2_2, 1);
    analogWrite(pwm_2, pwm);
  }
  else {
    pwm = -pwm;
    if (pwm > 255)
      pwm = 255;
    digitalWrite(in1_2, 1);
    digitalWrite(in2_2, 0);
    analogWrite(pwm_2, pwm);
  }
}

void DC3(int pwm) {
  if (pwm >= 0) {
    if (pwm > 255)
      pwm = 255;
    digitalWrite(in1_3, 1);
    digitalWrite(in2_3, 0);
    analogWrite(pwm_3, pwm);
  }
  else {
    pwm = -pwm;
    if (pwm > 255)
      pwm = 255;
    digitalWrite(in1_3, 0);
    digitalWrite(in2_3, 1);
    analogWrite(pwm_3, pwm);
  }
}

void DemEncoder_1()
{
  if (digitalRead(encoderB_1) == 0) {
    dem_1 = dem_1 + 1;
  } else dem_1 = dem_1 - 1;
}
void DemEncoder_2()
{
  if (digitalRead(encoderB_2) == 0) {
    dem_2 = dem_2 - 1;
  } else dem_2 = dem_2 + 1;
}
void DemEncoder_3()
{
  if (digitalRead(encoderB_3) == 0) {
    dem_3 = dem_3 + 1;
  } else dem_3 = dem_3 - 1;
}
void hienthi()
{
  // in luc chạy thiệt //
//  /*
    Serial.print("vtri_d*= ");
    Serial.print(xRobot_dat);
    Serial.print(" ! ");
    Serial.print(yRobot_dat);
    Serial.print(" @ ");
    //
    Serial.print("vtri_t=# ");
    Serial.print(xRobot_thuc);
    Serial.print(" $ ");
    Serial.print(yRobot_thuc);
    //

    Serial.print(" %tg= ");
    Serial.print(0.0003 * thoigian);
    Serial.println(" ^");
  //*/




}
