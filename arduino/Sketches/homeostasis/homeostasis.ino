#include <helper_3dmath.h>
#include <MPU6050.h>
//#include <MPU6050_6Axis_MotionApps20.h>
//#include <MPU6050_9Axis_MotionApps41.h>

#include <MatrixMath.h>
#include <math.h>

#define NUM_M (1)
#define NUM_S (1)

// learning rates
float epsA = 0.1;
float epsC = 0.1;

// create matrices
float M_X[NUM_S][1];
float M_XPred[NUM_S][1];
float M_XError[NUM_S][1];

float M_Y[1][NUM_M];

float M_A[NUM_S][NUM_M];
float M_B[NUM_S][1];

float M_C[NUM_M][NUM_S];
float M_H[NUM_M][1];

float M_EpsA[NUM_S][NUM_S];
float M_EpsC[NUM_M][NUM_M];

// timing variables
long lastMillis = 0;
long interval = 20; // 20ms = 50Hz

void setup() {
  // start serial communication
  Serial.begin(115200);

  initializeMatrices();

  Serial.println("Initialization finished!");
}

void loop() {
  // call every [interval] ms
  if(lastMillis + interval < millis()){
    // save milliseconds
    lastMillis = millis();

    // read sensor values
    readSensors();

    // calculate prediction error
    Matrix.Subtract((float*)M_X, (float*)M_XPred, NUM_S, 1, (float*)M_XError);

    // train model
    trainModel();

    // train controller
    trainController();
  }
}

// initialize all matrices
void initializeMatrices(){
  Serial.print("Initializing Matrices...\t");
  
  // initialize matrices that correspond to the number of sensors
  for(int s = 0; s < NUM_S; s++){
    M_X[s][0] = 0;
    M_XPred[s][0] = 0;
    M_XError[s][0] = 0;
    M_B[s][0] = 0;

    M_EpsA[s][s] = epsA;
  }

  // initialize matrices that correspond to the number of motors
  for(int m = 0; m < NUM_M; m++){
    M_Y[m][0]=0;
    M_H[m][0]=0;

    M_EpsC[m][m] = epsC;
  }

  // initalize matrices that correspond to the numer of sensors and the number of motors
  for(int s = 0; s < NUM_S; s++){
    for(int m = 0; m < NUM_M; m++){
      M_A[s][m] = 0;
      M_C[m][s] = 0;
    }
  }

  Serial.println("OK");
}

void readSensors(){
  // read position sensor of servo (0-1024) and normalize it
  M_X[0][0] = (float)analogRead(A0)/(float)1024;
}

/**
 * This function trains the Model from the Prediction error and the learning rate. This calculates the formulas
 * A += epsA * XError * y
 * b += epsA * XError
 */
void trainModel(){
  float M_Tmp[NUM_S][1];
  float M_Tmp2[NUM_S][NUM_M];

  Matrix.Multiply((float*)M_EpsA, (float*)M_XError, NUM_S, NUM_S, 1, (float*)M_Tmp);

  // add Tmp to B
  Matrix.Add((float*)M_B, (float*)M_Tmp, NUM_S, 1, (float*) M_B);

  // multiply Y to it
  Matrix.Multiply((float*)M_Tmp, (float*)M_Y, NUM_S, 1, NUM_M, (float*)M_Tmp2);

  // add Tmp2 to A
  Matrix.Add((float*)M_A, (float*)M_Tmp2, NUM_S, NUM_M, (float*)M_A);
}

/**
 * This function trains the Controler.
 */
void trainController(){
  float M_GZ[NUM_M][1];
  float M_Eta[NUM_S][1];
  float M_EpsC_Eta[NUM_S][1];

  // calculate Z (in the variable gz)
  Matrix.Multiply((float*)M_C, (float*)M_X, NUM_M, NUM_S, 1, (float*)M_GZ); // Z = C * X
  Matrix.Add((float*)M_GZ, (float*)M_H, NUM_M, 1, (float*)M_GZ); // Z += H

  // calculate g_z (tanh'(z) = 1 - tanh^2(z))
  tanh_prime_matrix((float*)M_GZ, NUM_M, 1);

  // calculate Eta eta_i = sum(j){A_ij * gz_i * err_i)
  for(int i; i < NUM_S; i++){
    M_Eta[i][i] = 0;
    for(int j; j < NUM_M; j++){
      M_Eta[i][i] += M_A[i][j] * M_GZ[j][1] * M_XError[i][1];
    }
  }

  // calculate epsC * Eta
  Matrix.Multiply((float*)M_EpsC, (float*)M_Eta, NUM_S, NUM_S, 1, (float*)M_EpsC_Eta);

  // Add the result to h
  Matrix.Add((float*)M_A, (float*) M_EpsC_Eta, NUM_S, 1, (float*) M_A);

  // Multiply the result by x
  // TODO: Progress
  
}

// from https://github.com/f32c/arduino/blob/master/hardware/fpga/f32c/system/src/math/tanh.c
float tanh(float x)
{
  float x0 = exp(x);
  float x1 = 1.0 / x0;

  return ((x0 + x1) / (x0 - x1));
}

float tanh_prime(float x){
  float tanh_ = tanh(x);
  return 1 - tanh_ * tanh_;
}

void tanh_prime_matrix(float* A, int m, int n){
  for (int i=0;i<m;i++)
  {
    for(int j=0;j<n;j++)
    {
      A[n*i+j] = tanh_prime(A[n*i+j]);
    }
  }
}
