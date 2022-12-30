#include "LSM6DSOXSensor.h"
LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_H);
int32_t acceleratio[3];
float acceleration[3];
int32_t rotatio[3];
int32_t rotation[3];
float RwEst[3];
float RwGyro[3];
uint8_t firstSample = 1;
float Awz[2];
float mean[3];
float mean_gx = 361.62;  //300 sempre sotto 0  o 400 sarebbe 361< x <410
float mean_gy = -819.05; // 770 sempre sopra zero  < 819
float mean_gz = -818.31;

float norm_g;
long int old_t = 0;


int wGyro = 10; //range 5-20 of trust in gyro value

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
    Serial.print(id);
  } else {
    Serial.println("\n Receviced correct ID for LSM6DSOX sensor: ");
    Serial.print(id);
    }


  // Set accelerometer scale at +- 2G. Available values are +- 2, 4, 8, 16 G
  lsm6dsoxSensor.Set_X_FS(2);
  // Set gyroscope scale at +- 125 degres per second. Available values are +- 125, 250, 500, 1000, 2000 dps
  lsm6dsoxSensor.Set_G_FS(125);
  // Set Accelerometer sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
  lsm6dsoxSensor.Set_X_ODR(12.0f);
  // Set Gyroscope sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
  lsm6dsoxSensor.Set_G_ODR(12.0f);

  mean[0] = 361.62;  
  mean[1] = -819.05; 
  mean[2] = -818.31;
}

void loop() {

  // Read accelerometer
  uint8_t acceleroStatus;
  uint8_t gyroStatus;
  lsm6dsoxSensor.Get_X_DRDY_Status(&acceleroStatus);
  lsm6dsoxSensor.Get_G_DRDY_Status(&gyroStatus);
  if (acceleroStatus == 1 && gyroStatus == 1) { // Status == 1 means a new data is available
    
  lsm6dsoxSensor.Get_X_Axes(acceleratio);
  lsm6dsoxSensor.Get_G_Axes(rotatio);
  long int current_t = millis();
  long int t = current_t-old_t;
  old_t = current_t;
  norm_g = sqrt(acceleratio[0]*acceleratio[0]+acceleratio[1]*acceleratio[1]+acceleratio[2]*acceleratio[2]);
  acceleration[1]=( acceleratio[2]*-1) / norm_g;
  acceleration[2]=acceleratio[0] / norm_g;
  acceleration[0]=(acceleratio[1]*-1) / norm_g;
  rotation[0] = rotatio[1]*-1;
  rotation[1] = rotatio[2]*-1;
  rotation[2] = rotatio[0];
  /*rotation[0] -= mean[0];
  rotation[1] -= mean[1];
  rotation[2] -= mean[2];*/
 
  /*Serial.print("\n Acceleration  \n");
    Serial.print("x= ");
    Serial.print(acceleration[0]); 
    Serial.print(" g, y= ");
    Serial.print(acceleration[1]);
    Serial.print(" g, z= "); 
    Serial.print(acceleration[2]);
  Serial.print("\n Gyro  \n");
    Serial.print("x= ");-
    Serial.print(rotation[0]/1000); 
    Serial.print(" g, y= ");
    Serial.print(rotation[1]/1000);
    Serial.print(" g, z= "); 
    Serial.print(rotation[2]/1000);  */
  if (firstSample==1){
    for(int w=0;w<=2;w++) RwEst[w] = acceleration[w]; 
    old_t = millis();
    firstSample=0;
  }
  else
  {
    if(abs(RwEst[2]) < 0.01){
      //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
      //in this case skip the gyro data and just use previous estimate
      for(int w=0;w<=2;w++) RwGyro[w] = RwEst[w];
    }
    else
    {
      //Serial.print("is doing");
      Awz[0] = atan2(RwEst[0],RwEst[2]) * 180 / PI;   //get angle and convert to degrees        
      Awz[0] += ((rotation[1])*(t/1000) );                     //get updated angle according to gyro movement

      Awz[1] = atan2(RwEst[1],RwEst[2]) * 180 / PI;   //get angle and convert to degrees        
      Awz[1] += ((rotation[0])*(t/1000));                     //get updated angle according to gyro movement

    }
    //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
    //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
    float signRzGyro = ( cos(Awz[0] * PI / 180) >=0 ) ? 1 : -1;
    //Serial.println(signRzGyro);
    //float signRxGyro = ( cos(Awz[1] * PI / 180) >=0 ) ? 1 : -1;
    //Serial.println(signRxGyro);
    //reverse calculation of RwGyro from Awz angles, for formula deductions see  http://starlino.com/imu_guide.html
    
    RwGyro[0] = sin(Awz[0] * PI / 180);
    RwGyro[0] /= sqrt( 1 + pow(cos(Awz[0] * PI / 180),2) * pow((tan(Awz[1] * PI / 180)),2 ));
    RwGyro[1] = sin(Awz[1] * PI / 180);
    RwGyro[1] /= sqrt( 1 + pow(cos(Awz[1] * PI / 180),2) * pow((tan(Awz[0] * PI / 180)),2 ));

    RwGyro[2] = signRzGyro * sqrt(1 - pow(RwGyro[0],2) - pow(RwGyro[1],2));
    //combinazione acc + gyro
    for(int w=0;w<=2;w++) RwEst[w] = (acceleration[w]+RwGyro[w]*wGyro)/(1+wGyro);
    //normalizzazione
    float R = sqrt(RwEst[0]*RwEst[0]+RwEst[1]*RwEst[1]+RwEst[2]*RwEst[2]);
    for(int w=0;w<=2;w++) RwEst[w] /= R;
    
  }
  Serial.print("\nRxEst : ");Serial.print(RwEst[0]);Serial.print("\n");
  Serial.print("RyEst : ");Serial.print(RwEst[1]*-1);Serial.print("\n");
  }
}



 
  
