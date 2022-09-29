#include "LSM6DSOXSensor.h"
LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_H);
int on = 0;
int ong=0;
float PSI0=0.0;
float PHI0=0.0;
float THETA0=0.0;

float mean_gx = 361.62;  //300 sempre sotto 0  o 400 sarebbe 361< x <410
float mean_gy = -819.05; // 770 sempre sopra zero  < 819
float mean_gz = -818.31;
int32_t acceleration[3];
int32_t rotation[3];
int32_t rotationprev[3];

float norm_g = 0.0;

int TrustGYro = 7; //range 5-20 of trust in gyro value

void setup() {

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  lsm6dsoxSensor.begin();
  
  // Enable accelerometer and gyroscope, and check success
  if (lsm6dsoxSensor.Enable_X() == LSM6DSOX_OK && lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK) {
    Serial.println("\n Success enabling accelero and gyro");
  } else {
    Serial.println("\n Error enabling accelero and gyro");
  }
  
  // Read ID of device and check that it is correct
  uint8_t id;
  lsm6dsoxSensor.ReadID(&id);
  if (id != LSM6DSOX_ID) {
    Serial.println("\n Wrong ID for LSM6DSOX sensor. Check that device is plugged");
  } else {
    Serial.println("\n Receviced correct ID for LSM6DSOX sensor");
  }


  // Set accelerometer scale at +- 2G. Available values are +- 2, 4, 8, 16 G
  lsm6dsoxSensor.Set_X_FS(2);
  // Set gyroscope scale at +- 125 degres per second. Available values are +- 125, 250, 500, 1000, 2000 dps
  lsm6dsoxSensor.Set_G_FS(125);
  // Set Accelerometer sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
  lsm6dsoxSensor.Set_X_ODR(12.0f);
  // Set Gyroscope sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
  lsm6dsoxSensor.Set_G_ODR(12.0f);
}

void loop() {

  // Read accelerometer
  uint8_t acceleroStatus;
  lsm6dsoxSensor.Get_X_DRDY_Status(&acceleroStatus);
  if (acceleroStatus == 1) { // Status == 1 means a new data is available
    
    lsm6dsoxSensor.Get_X_Axes(acceleration);
    // Plot data for each axe in mg
    /*float norm_g = sqrt(acceleration[0]*acceleration[0]+acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]);
    float x1 = acceleration[0] / norm_g;
    float y1 = acceleration[1] / norm_g;
    float z1 = acceleration[2] / norm_g;*/
    norm_g = sqrt(acceleration[0]*acceleration[0]+acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]);
    float x1 = acceleration[0] / norm_g;
    float y1 = acceleration[1] / norm_g;
    float z1 = acceleration[2] / norm_g;
    float Racc = sqrt(x1*x1+y1*y1+z1*z1);
    /*Serial.println(x1);
    Serial.println(y1);
    Serial.println(z1);
    Serial.print("Racc: (must be similar to 1) ");Serial.print(Racc);Serial.print("\n");*/
    /*Serial.print("Norm G");
    Serial.print(norm_g);
    Serial.print("\n Acceleration NORM \n");
    Serial.print("x= ");
    Serial.print(x1); 
    Serial.print(" g, y= ");
    Serial.print(y1);
    Serial.print(" g, z= "); 
    Serial.print(z1);
    Serial.print(" g ");*/
    float theta = atan(x1/sqrt((y1*y1)+(z1*z1)));  //asse x
    float psi = atan(y1/sqrt((x1*x1)+(z1*z1)));  //ass y 
    float phi = atan(sqrt((x1*x1)+(y1*y1))/z1); //asse z
    float Axr = acos(x1);
    float Ayr = acos(y1);
    float Azr = acos(z1);
    
    /*Serial.print("\n THETA x: ");
    Serial.print(theta);
    Serial.print("\n PSI y: ");
    Serial.print(psi);
    Serial.print("\n PHI z: ");
    Serial.print(phi);
    Serial.print("\n Axr x: ");
    Serial.print(Axr);
    Serial.print("\n Ayr y: ");
    Serial.print(Ayr);
    Serial.print("\n Azr z: ");
    Serial.print(Azr);*/
    

    // Read gyroscope
  uint8_t gyroStatus;
  lsm6dsoxSensor.Get_G_DRDY_Status(&gyroStatus);
  if (gyroStatus == 1) { // Status == 1 means a new data is available
    if(ong==1){
    float gyro_calibrated[3];
    lsm6dsoxSensor.Get_G_Axes(rotation);
    // Plot data for each axe in milli degres per second
    /*gyro_calibrated[0] = (rotation[0]- mean_gx)/1000;
    gyro_calibrated[1] = (rotation[1]- mean_gy)/1000;
    gyro_calibrated[2] = (rotation[2]- mean_gz)/1000;*/
    /*float RGYRO = sqrt(rotation[0]*rotation[0]+rotation[1]*rotation[1]+rotation[2]*rotation[2]);
    Serial.print("RGYRO: (must be similar to 1) ");Serial.print(RGYRO);Serial.print("\n");*/
    gyro_calibrated[0] = ((rotation[0]- mean_gx)/1000)+ rotationprev[0];  //il corrente andrebbe moltiplicato per T che è freq cam
    gyro_calibrated[1] = ((rotation[1]- mean_gy)/1000)+ rotationprev[1];
    gyro_calibrated[2] = ((rotation[2]- mean_gz)/1000)+ rotationprev[2];
    float RGYRO = sqrt(gyro_calibrated[0]*gyro_calibrated[0]+gyro_calibrated[1]*gyro_calibrated[1]+gyro_calibrated[2]*gyro_calibrated[2]);
    //Serial.print("RGYRO: (must be similar to 1) ");Serial.print(RGYRO);Serial.print("\n");
    
    rotationprev[0] = rotation[0];
    rotationprev[1] = rotation[1];
    rotationprev[2] = rotation[2];
    /*Serial.print("\n Rotation \n"); 
    Serial.print("x= ");
    Serial.print(gyro_calibrated[0]);
    Serial.print(" dps, y= "); 
    Serial.print(gyro_calibrated[1]);
    Serial.print(" dps, z= "); 
    Serial.print(gyro_calibrated[2]); 
    Serial.print(" dps \n");*/
    
    float RxGyro = sin(gyro_calibrated[1]) / sqrt(1 + (cos(gyro_calibrated[1])*cos(gyro_calibrated[1])) *
                                                          (tan(gyro_calibrated[0])*tan(gyro_calibrated[0])) );
    float RyGyro = sin(gyro_calibrated[0]) / sqrt(1 + (cos(gyro_calibrated[0])*cos(gyro_calibrated[0])) * 
                                                           (tan(gyro_calibrated[1])* tan(gyro_calibrated[1])));
    
    float RyEst = 0.0;
    float RxEst = 0.0;
    /*if(x1>0){
      RxEst = (x1 + sqrt(RxGyro*RxGyro)* TrustGYro)/(1+TrustGYro);
    }else{
      RxEst = (x1 + RxGyro* TrustGYro)/(1+TrustGYro);
    }
    if (x1<0 && RxEst>0){
      RxEst = RxEst*-1;
    }

    if(y1>0){
      RyEst = (y1 + sqrt(RyGyro*RyGyro)* TrustGYro)/(1+TrustGYro);
    }else{
      RyEst = (y1 + RyGyro* TrustGYro)/(1+TrustGYro);
    }
    if(y1<0 && RyEst>0){
      RyEst = RyEst*-1;
    }*/
    
    if (RyGyro<1.5 and RyGyro >-1.5){ //filtro rumore su y
      RyGyro = 0.0;
    }
    if (RxGyro<1.5 and RxGyro >-1.5){//filtro rumore su x
      RxGyro = 0.0;
    }
    //Serial.print("\nRxGyro : ");Serial.print(RxGyro);Serial.print("\n");
    //Serial.print("RyGyro : ");Serial.print(RyGyro);Serial.print("\n");
    RxEst = (x1 + RxGyro* TrustGYro)/(1+TrustGYro);
    RyEst = (y1 + RyGyro* TrustGYro)/(1+TrustGYro);
    float RzEst = 1; // 1 or -1 
    float REst = sqrt(RxEst*RxEst+RyEst*RyEst+RzEst*RzEst);
    float RRx = RxEst/REst;
    float RRy = RyEst/REst;
    Serial.print("\nRxEst : ");Serial.print(RxEst);Serial.print("\n");
    Serial.print("RyEst : ");Serial.print(RyEst);Serial.print("\n");
    /*Serial.print("REst: ");Serial.print(REst);
    Serial.print("\n");
    Serial.print("RRx: ");Serial.print(RRx);Serial.print("\n");
    Serial.print("RRy: ");Serial.print(RRy);Serial.print("\n");*/
    
    }else{
      ong = 1;
      lsm6dsoxSensor.Get_G_Axes(rotationprev);
      rotationprev[0] = (rotationprev[0]- mean_gx)/1000;
      rotationprev[1] = (rotationprev[1]- mean_gy)/1000;
      rotationprev[2] = (rotationprev[2]- mean_gz)/1000;
    }
  }
  } //TO GO BACK REMOVE ROTATIONPREV
} 
