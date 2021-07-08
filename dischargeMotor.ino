

// const int VOLT_pin = A0; // voltage read. must be below 5v
// const int val = 50; // sets the motor speed. don't change for now. 

// bool discharging; // boolean for charging action
// bool discharged; 



// // add this to where ever you want to call the discharge function.
// void dischargeInitialization(){
//   voltageCheck();
//   motorInit();
//   motorBeginDischarge();
//   motorShutdown();
// } 

// // function to check the voltage of the battery. 
// void voltageCheck() {
//   int value;
//   float volt;
//   float voltAverage = 0.0;
//   value = analogRead( VOLT_pin ); 
//   volt = value * 5.0 / 1023.0;
//   for( int i =0; i< 10; i++){
//     voltAverage = voltAverage + volt; // concatinates
//   }
//   voltAverage = voltAverage / 10; // calculates the agerage voltage
//   Serial.print( " Battery volt average " );
//   Serial.println( voltAverage );
//   delay(500);
  
//   if (voltAverage > 3.98) { // for loop to check if the average voltage is above the cut off voltage
//     discharging = true; // set discharging to true
//     Serial.print( "discharging set to true: " ); 
//     Serial.println( discharging );
//     delay(500); 
//   } else {
//     discharging = false; //sets discharging to false
//     Serial.print( "discharging set to false: " );
//     Serial.println( discharging );
//     delay(500);
//   }
// }

// // // initialise motor function
// // void motorInit() {
// //   myServo.write(10); // don't change this  
// //   delay(5000); // don't change this. Mandatory wait time
// // }

// // begin motor and set speed. 
// void motorBeginDischarge() {
//   int motorSpeed = val;
//   voltageCheck();
//   delay(500);
//   while (discharging == true) { 
//     Serial.println( "Discharging" );
//     myServo.write(motorSpeed);
//     voltageCheck();
//   }
//   Serial.print( "Discharging: "); 
//   Serial.print( discharging );
//   Serial.println( " Battery voltage cuttoff reached, shutting down load bank" );
// }

// // shut down motor
// void motorShutdown(){
//     myServo.write(0); 
//     discharged = true; // sets discharged bool to true
//     delay(5000);
// }
