#include <iostream>
#include <bits/stdc++.h>

//• Initial Velocity: 30m/s
//• Initial Angle: 46 deg

using namespace std;
#define pi 3.14159
float mpu6050[10] = {0.0, 11.68, 18.95, 23.56, 25.72, 25.38, 22.65, 18.01, 10.14, -0.26}; // accuracy 79% [A]
float bno55[10] = {0.0,9.49, 16.36, 21.2, 23.16, 22.8, 19.5, 14.85, 6.79, -2.69}; // accuracy 92% [B]
float p_A=0.79,p_B=0.92; // probability of accuracy for both measurements
float x_hat=0.0; // The state variable estimate

/// all data below are assumed
float P=1.0; // Error Covariance
float Q=0.01; // assuming process noise covariance with this value which reflects how much you trust the model's predictions
float R_A=0.01,R_B=0.01;  //Measurement covariance of Sensor A and B depends on how much we trust the accuracy of the sensor
//note that A larger value in the covariance matrix indicates greater uncertainty or noise in the sensor's measurements


float Initial_Angle= 46;
int Initial_Velocity= 30*sin(Initial_Angle*pi/180);

void get_measurements();
int main()
{
    cout << "let's go\n";
    get_measurements();
    cout << "End of program ;)" << endl;
    return 0;
}

void get_measurements()
{
    for(int i=0; i<10; i++)
    {
        //predicting
        P=P+Q;
        //cout << "predicted is : " << x_hat << endl;

        // Kalman gains
        float K_A = P / (P + R_A*p_A);
        float K_B = P / (P + R_B*p_B);
        //cout << "kalman gains are : " << K_A << " : " << K_B << endl;


        // innovations
        float y_A = mpu6050[i] - x_hat;
        float y_B = bno55[i] - x_hat;
        //cout << "innovations are : " << y_A << " : " << y_B << endl;

        // Updating state estimate with weighted contributions
        x_hat = x_hat + K_A * y_A + K_B * y_B * p_B;
        //cout << "estimate is : " << x_hat << endl;

        // Updating estimate covariance with weighted contibutions
        P = (1 - K_A*p_A)* (1 - K_B*p_B) * P;

        //cout << "updated estimate covariance is : " << P << endl << endl;
        //cout << "el K hateb2a be : " << K_A << " we " << K_B << endl << endl;
        cout << "Estimate of "<< mpu6050[i] << " and " << bno55[i]
        << " is : " << x_hat << endl << endl;
    }
}
