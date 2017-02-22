#include <MatrixMath.h>

#define NUM_M (2)
#define NUM_S (3)

float A[NUM_M][NUM_S];
float AT[NUM_S][NUM_M];
float B[NUM_M][1];
float AT_B[NUM_S][1];

void setup() {
  //initialize matrices
  A[0][0]=0; A[0][1]=1; A[0][2]=2;
  A[1][0]=3; A[1][1]=4; A[1][2]=5;

  B[0][0]=0.5; B[1][0]=1.5;

  Serial.begin(9600);
}

void loop() {
  if(lastUpdateMillis + interval < millis()){
  
  Matrix.Transpose((float*)A, NUM_M, NUM_S, (float*)AT);
  Matrix.Print((float*)AT, NUM_S, NUM_M, "AT");

  Matrix.Multiply((float*)AT, (float*)B, NUM_S, NUM_M, 1, (float*)AT_B);
  Matrix.Print((float*)AT_B, NUM_S, 1,"AT_B");
  while(1){;}
}
