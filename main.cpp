/*
 EW202 Lab 5.3
 Proportional-integral control of EW202 elevator
 D Evangelista, 2019
 */

#include "mbed.h"
#include "rtos.h"
#include "Motor.h"

// some defines
#define DT 20         // loop timing in ms
#define CAL_A -59.7382
#define CAL_B 79.7480   
#define CAL_C -2.9079
#define PI_KP 0.1   // proportional control gain K_P
#define PI_KI 0.1   // integral control gain K_I

// hardware objects
Serial pc(USBTX,USBRX);
AnalogIn sensor(p20);
Motor motor(p26,p30,p29); 
DigitalOut heartbeat(LED1);

// function prototypes
float calibration(float);
float pi_control(float, float);





int main(void){
  int n_samples, i;
  float ref, ymeas, e, u;
  float ie=0; 
  
  while(1){
    motor.speed(0.0);  // turn off motor while awaiting commands
    pc.scanf("%d,%f",&n_samples,&ref); // get command from terminal

    // the actual control loop is here
    for (i=0; i<n_samples; i++){
      ymeas = calibration(sensor.read());
      e = ref-ymeas;
      ie = ie+e*DT; // keep running track of the integral of error
      u = pi_control(e,ie);
      motor.speed(u);

      heartbeat = !heartbeat;
      pc.printf("%d,%f,%f\n",i,ymeas,u);
      ThisThread::sleep_for(DT); 
    } // for n_samples
  } // while(1)
} // main()





/** @brief Applies calibration for GPD212 sensor
    The calibration is inches = Ay^2+By+C
    
    @param y is a float AnalogIn 0.0-1.0 
    @return height in inches as a float
*/
float calibration(float y){
  return CAL_A*y*y+CAL_B*y+CAL_C; 
}





/** @brief Implements proportional control

    @param e is the error, ref-meas in inches, float
    @return motor pwm value, a float from 0.0-1.0
*/
float pi_control(float e, float ie){
  return PI_KP*e + PI_KI*ie; 
}
