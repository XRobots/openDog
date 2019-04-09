// *********************** Kinematic model fuction ******************************************


double leg(double Z4, double Y4, double X4, double yaw, double pitch2, double roll, int side, int front) {
  
            // ************* XYZ consts and vars *****************************************************************
            #define HIPROD 118L           // offset from hip pivot to middle of leg
            #define HIPROD2 263L          // diagonal length from hip pivot to fixed actuator pivot 
            #define HIPROD3 150L          // length of hip actuator pivot 
            #define HIPOFFSETANGLE 32L    // DEGREES angle of diagnonal between hip pivot and actuator pivot
            double X;                     // X once offsets and inversion are taken ito account
            double X2;                    // X after yaw moves;
            double X3;                    // X at the roll calc stage  
            double Y;                     // Y to give to XYZ calcs
            double Y2;                    // Y after yaw moves
            double Z;                     // Z to give to XYZ cals
            double Z2;                    // Z after yaw moves
            double Z3;                     // Z at the roll calc stage
            double hipHypotenuse;         // the length between hip pivot and foot
            double hipAngle1;             // RADIANS - hip angle from vertical
            double hipAngle1a;             // DEGREES - hip angle from vertical
            double hipAngle2;             // RADIANS - angle at foot between middle of leg and hip pivot
            double hipAngle2a;             // DEGREES - angle at foot between middle of leg and hip pivot
            double hipAngle3;             // RADIANS - angle required from actual actuator
            double hipAngle3a;            // DEGREES - angle required from actual actuator
            double hipActuator;           // output length of ball screw
            double legHeight;             // height of leg in the 'sideways tilting plane'
  
            double shoulderOffset;        // RADIANS - offset angle to move the actual shoulder by, large angle moves the leg forward
            double shoulderOffset_a;       // DEGREES - offset angle to move the actual shoulder by, large angle moves the leg forward
            double legLength;              
         
            #define DIGITLENGTH 363L    // length of each top/bottom leg
            #define ELBOWROD 190L       // length of push rod
            #define ELBOWROD2 95L        // other side of push rod triangle
            #define ELBOWACTANGLE 15.8L   // angle between actuator and upper leg joint            
            double elbowAngle;           // the actual angle of the elbow between top and bottom sections
            double elbowAngle2;          // angle between bottom of leg and actuator
            double elbowAngle2a;          // angle between bottom of leg and actuator
            double elbowAngle3;          // other angle
            double elbowAngle3a;          // other angle
            double elbowAngle4;          // other angle
            double elbowAngle4a;          // other angle
            double elbowActuator;        // calculated length of actuator from joint

            #define SHOULDERROD 149L
            #define SHOUDLERROD2 102L
            #define SHOULDERANGLE2 15.8L
            #define SHOULDERANGLE3 17L
            double shoulderAngle;     // initial demand for shoulder (half elbow angle plus offset for moving the leg
            double shoulderAngle2;    // RADIANS - the actual angle we need from the shoulder actuator
            double shoulderAngle2a;   // DEGREES - the actual angle we need from the shoulder actuator
            double shoulderAngle3;    // RADIANS - first ancgle to solve
            double shoulderAngle3a;   // DEGREES - first ancgle to solve
            double shoulderAngle4;    // RADIANS - second angle to solve
            double shoulderAngle4a;    // Degrees - second angle to solve
            double shoulderActuator;  // actual output length of actuator

            // **************** Pitch Roll Yaw consts and vars **********************************************

            //  Pitch consts and vars - side view
            #define BODYRODY 287L      // half the distance from shoulder to shoulder on the Y side plane/view   
            double pitch;             // pitch after switching around for front/back of robot
            double differenceZ;         // difference in height between center point and new shoulder height, in Y axis
            double shoulderHeight;    // resulting new height of shoulder from ground as body tilts in pitch, in a vertical axis from the ground
            double legLength2;        // length of actual leg we need to calculate the eventual new Y and Z
            double distanceY;         // the new distance from the centre to the shoulder joint, taking into account the pitch angle
            double differenceY;       // the difference between foot placement and new distance from centre to shoulder joint
            double shoulderAngle5;    // RADIANS - angle of leg from vertical
            double shoulderAngle5a;    // DEGREES - angle of leg from vertical
            double shoulderAngle6;    // RADIANS - angle of leg from new Z
            double shoulderAngle6a;    // DEGREES - angle of leg from new Z

            // Roll consts and vars - end view
            #define BODYRODX 110L     // half the distance from hip pivot to hip pivot in the X axis
            double differenceZ2;      // difference in height between the centre point and new hip neight, in an X axis
            double hipHeight;         // resulting new height of hip from ground as the tits in roll, in a vertical line from the ground
            double legLength3;        // the new ***length of the VIRTUAL leg*** we need to calulate the new Y and Z
            double distanceX;         // the new distance from the centre to the hip joint, taking into account the roll angle
            double differenceX;       // the difference between foot placement and the new distance from the centre to the hip joint
            double hipAngle5;         // RADIANS the angle of the *VIRTUAL leg* from vertical
            double hipAngle5a;        // DEGREES the angle of the *VIRTUAL leg* from vertical
            double hipAngle6;         // RADIANS the angle of the *VIRTUAL* leg from the new Z
            double hipAngle6a;        // DEGREES the angle of the *VIRTUAL* leg from the new Z

            // Yaw consts and vars - top view
            double radius;            // radius/hypotenuse of triangle of leg from top XY view
            double yawAngle1;         // RADIANS - the origin angle of the radius using existing stick posotions
            double yawAngle1a;        // DEGREES - the origin angle of the radius using existing stick posotions
            double yawAngle2;         // RADIANS - the new demand angle of the radius 
            double yawAngle2a;        // DEGREES - the new demand angle of the radius 

            if (front == 1) {
              Y4 = Y4 + BODYRODY;
            }
            else if (front == 0) {
              Y4 = Y4 - BODYRODY;
            }
            if (side == 1) {
              X4 = X4 + (BODYRODX + HIPROD);
            }
            if (side == 0) {
              X4 = X4 - (BODYRODX + HIPROD);
            }

            yaw = (yaw * 71) / 4068;                // convert to RADIANS

            yawAngle1 = atan(X4/Y4);                  // calc existing angle of leg from centre
            yawAngle1a = (yawAngle1 * 4068) / 71;   // DEGREES - for debug            

            radius = X4/sin(yawAngle1);              // calc radius from centre

            yawAngle2 = yawAngle1 + yaw;
            yawAngle2a = (yawAngle2 * 4068) / 71;   // DEGREES - for debug

            X2 = radius * sin(yawAngle2);           // calc new X and Y based on new yaw angle
            Y2 = radius * cos(yawAngle2);
  
            if (front == 1) {                       // remnove offsets again to give results centred around zero
              Y2 = Y2 - BODYRODY;
            }
            else if (front == 0) {
              Y2 = Y2 + BODYRODY;
            }
            if (side == 1) {
              X2 = X2 - (BODYRODX + HIPROD);
            }
            else if (side == 0){
              X2 = X2 + (BODYRODX + HIPROD);
            }   

            Z2 = Z4;                              // pass through Z as it isn't needed in yaw calcs
                     
            // switch round X and roll for each side of the robot
            if (side == 1) {                                      //right leg
              X3 = HIPROD+X2;                                       
            }
            else if (side == 0) {                                 //left leg
              X3 = HIPROD-X2; 
            }

            // switch around pitch angle for back or front of robot
            if (front == 1) {                                     // front of robot
              pitch = pitch2*-1;
            }
            else if (front == 0) {                                // back of robot
              pitch = pitch2;
            }            

            // **************** Pitch calcs **********************************************

            pitch = (pitch * 71) / 4068;                        // convert pitch to RADIANS
            differenceZ = sin(pitch) * BODYRODY;                // calc opposite side of triangle
            distanceY = cos(pitch) * BODYRODY;                  // calc adjacent side of triangle

            shoulderHeight = Z2 + differenceZ;                   // calc new shoulder height as pitch changes
            differenceY = Y2 + (BODYRODY - distanceY);           // distance from original foot position and vertical line to shoulder

            if (front == 1) {                                     // switch around the data if it's the front of the robot
              differenceY = differenceY*-1;
            }

            shoulderAngle5 = atan(differenceY/shoulderHeight);    // RADIANS angle between leg and vetical
            shoulderAngle5a = (shoulderAngle5 * 4068) / 71;      // DEGREES convert for debug
            legLength2 = shoulderHeight/cos(shoulderAngle5);      // calc out new length of leg
            
            shoulderAngle6 = pitch+shoulderAngle5;               // RADIANS calc angle for new Z to 90' from body

            if (front == 1) {                                    // switch around the data if it's the front of the robot
              shoulderAngle6 = shoulderAngle6*-1;
            }
            

            shoulderAngle6a = (shoulderAngle6 * 4068) / 71;      // DEGREES convert for debug

            Z3 = legLength2*cos(shoulderAngle6);                  // new Z to pass on to roll calcs
            Y = legLength2*sin(shoulderAngle6);                   // new Y to pass on to roll cals 

            // **************** Roll calcs ***************************************************

            roll = ((roll * 71) / 4068)*-1;                       // convert roll to RADIANS
            differenceZ2 = sin(roll) * BODYRODX;                  // calc opposite side of triangle
            distanceX = cos(roll) * BODYRODX;                     // calc adjacent side of triangle
            
            if (side == 0) {                                      // *********** switch around for each side of the robot
                hipHeight = Z3 - differenceZ2;                    // new height of the hip from the ground taking into account roll
            }
            if (side == 1) {
                hipHeight = Z3 + differenceZ2;
            }
            
            differenceX = X3 + (BODYRODX - distanceX);            // diatance from original foot postion and vertical line to shoulder.

            if (side == 1) {                                      // *********** switch around for each side of the robot
              differenceX = differenceX*-1;
            }        

            hipAngle5 = atan(differenceX/hipHeight);              // RADIANS angle between *virtual* leg and vertical
            hipAngle5a = (hipAngle5 * 4068) / 71;                 // DEGREES convert for debug 
           
            legLength3 = hipHeight/cos(hipAngle5);                // calc new *virtual leg* length

            hipAngle6 = roll+hipAngle5;                          // RADIANS calc angle for new Z to 90' from body
            hipAngle6a = (hipAngle6 * 4068) / 71;                // DEGREES convert for debug
       
            if (side == 1) {                                     // *********** switch around for each side of the robot
              hipAngle6 = hipAngle6 *-1;
            }
                           
            X = legLength3*sin(hipAngle6);                        // new X to pass onto XYZ calcs        
            Z = legLength3*cos(hipAngle6);                        // new Z to pass onto XYZ calcs          
                      
            
            //************ X & Z axis calc for each leg ***************   
                         
            if (X == 0) { X = 0.1; }                             // avoid divide by zero
  
            hipAngle1 = atan(X / Z);                             // calc hip angle from vertical
            hipAngle1a = (hipAngle1 * 4068) / 71;
            hipHypotenuse = X / sin(hipAngle1);                  // calc distance from hip pivot to foot

            hipAngle2 = asin(HIPROD / hipHypotenuse);             // calc angle at foot between middle of leg and hip pivot
            hipAngle2a = (hipAngle2 * 4068) / 71;                 // convert to degrees for debug

            legHeight = HIPROD / tan(hipAngle2);                  // leg height from 'sideways tilting plane' of leg

            hipAngle3a = (90 - HIPOFFSETANGLE) + (hipAngle1a + (180-90-hipAngle2a)) -90 ;        // DEGREES - angle required from actuator
            hipAngle3 = (hipAngle3a * 71) / 4068;                 // convert to radians

            //a2 = b2 + c2 − 2bc cosA            
            hipActuator =  sqrt( sq(HIPROD3)+sq(HIPROD2) - (2 * HIPROD3 * HIPROD2 * cos(hipAngle3)) )-223;      // work out actuator length relative to mid positon           

            //************ Y axis calcs **************

            if (Y == 0) { Y = 0.1; }
            // avoid divide by zero
            shoulderOffset = atan((Y*-1) / legHeight);                // calc shoulder angle offset
            shoulderOffset_a = (shoulderOffset * 4068) / 71;     // covert radians to degrees
            legLength = (Y*-1) / sin(shoulderOffset);                 // calc hypotenuse of triangle to make actual leg length 
            
            //************ elbow calcs ***************
            
            elbowAngle = acos ( (sq(DIGITLENGTH) + sq(DIGITLENGTH) - sq(legLength)) / (2 * DIGITLENGTH * DIGITLENGTH) );   
            elbowAngle = (elbowAngle * 4068) / 71;                          // convert radians to degrees

            elbowAngle2a = elbowAngle - ELBOWACTANGLE;                       // take away known angle        
            elbowAngle2 = (elbowAngle2a * 71) / 4068;                        // convert degrees to radians

            elbowAngle3 = asin ( (ELBOWROD2 * sin(elbowAngle2)/ELBOWROD));
            elbowAngle3a = (elbowAngle3 * 4068 / 71);                        // convert radians to degrees

            elbowAngle4a = 180 - elbowAngle2a - elbowAngle3a;                  // we now know all four angles        
            elbowAngle4 = (elbowAngle4a * 71) / 4068;                        // convert degrees to radians

            elbowActuator = (((sin(elbowAngle4)) * ELBOWROD)) / sin(elbowAngle2); 

            //************ shoulder calcs **************
            
            shoulderAngle = (180-elbowAngle)/2 + shoulderOffset_a;    // initial demand for shoulder (half elbow angle plus offset for moving the leg)
            shoulderAngle2a = 90-shoulderAngle-SHOULDERANGLE2+SHOULDERANGLE3;      // calc actual angle required from shoulder
            shoulderAngle2 = (shoulderAngle2a * 71) / 4068;                        // convert degrees to radians

            shoulderAngle3 = asin ((SHOUDLERROD2 * sin(shoulderAngle2))  /  SHOULDERROD) ;     //calc first unknown angle
            shoulderAngle3a = (shoulderAngle3 * 4068 / 71);                     // convert radians to degrees

            shoulderAngle4a = 180 - shoulderAngle2a - shoulderAngle3a;             // calc out remaining angle
            shoulderAngle4 = (shoulderAngle4a * 71) / 4068;                     // convert degrees to radians
            
            shoulderActuator =  ( (sin(shoulderAngle4)) * SHOULDERROD ) / sin(shoulderAngle2); 

            

           // ********************** output variables *********************************************                

              if (side == 1 && front == 1) {                     // front right leg
                mydata_front.hipR = hipActuator;
                mydata_front.shoulderR = shoulderActuator;
                mydata_front.elbowR = elbowActuator;
             
              }                             
              else if (side == 1 && front == 0) {               // back right leg
                mydata_back.hipR = hipActuator; 
                mydata_back.shoulderR = shoulderActuator;             
                mydata_back.elbowR = elbowActuator; 
            
              }              
              else if (side == 0 && front  == 1) {              // front right leg
                mydata_front.hipL = hipActuator;
                mydata_front.shoulderL = shoulderActuator;
                mydata_front.elbowL = elbowActuator;
            
              }               
              else if (side == 0 && front == 0) {               // back right leg
                mydata_back.hipL = hipActuator; 
                mydata_back.shoulderL = shoulderActuator;             
                mydata_back.elbowL = elbowActuator; 
              
              } 

              // constrain data so we don't run the actuator end stops

              mydata_front.shoulderR = constrain(mydata_front.shoulderR, 80,200);
              mydata_front.shoulderL = constrain(mydata_front.shoulderL, 80,200);
              mydata_front.elbowR = constrain(mydata_front.elbowR, 80,200);
              mydata_front.elbowL = constrain(mydata_front.elbowL, 80,200);
              mydata_front.hipR = constrain(mydata_front.hipR, -35,35);
              mydata_front.hipL = constrain(mydata_front.hipL, -35,35);

              mydata_back.shoulderR = constrain(mydata_back.shoulderR, 80,200);
              mydata_back.shoulderL = constrain(mydata_back.shoulderL, 80,200);
              mydata_back.elbowR = constrain(mydata_back.elbowR, 80,200);
              mydata_back.elbowL = constrain(mydata_back.elbowL, 80,200);
              mydata_back.hipR = constrain(mydata_back.hipR, -35,35);
              mydata_back.hipL = constrain(mydata_back.hipL, -35,35);

              // work out the encoder counts per mm

              mydata_front_count.hipR = mydata_front.hipR * 3490;
              mydata_front_count.hipL = mydata_front.hipL * 3490;
              mydata_front_count.shoulderR = mydata_front.shoulderR * 3490;
              mydata_front_count.shoulderL = mydata_front.shoulderL * 3490;
              mydata_front_count.elbowR = mydata_front.elbowR * 3490;
              mydata_front_count.elbowL = mydata_front.elbowL * 3490;

              mydata_back_count.hipR = mydata_back.hipR * 3490;
              mydata_back_count.hipL = mydata_back.hipL * 3490;
              mydata_back_count.shoulderR = mydata_back.shoulderR * 3490;
              mydata_back_count.shoulderL = mydata_back.shoulderL * 3490;
              mydata_back_count.elbowR = mydata_back.elbowR * 3490;
              mydata_back_count.elbowL = mydata_back.elbowL * 3490;
          
              // remove the number of counts that represent the physical position of the actuator switch / zero position.

              mydata_front_count_corr.hipR =  mydata_front_count.hipR; // no offset to remove
              mydata_front_count_corr.hipL =  mydata_front_count.hipL; // no offset to remove
              mydata_front_count_corr.shoulderR = 0 - (715450 - mydata_front_count.shoulderR);
              mydata_front_count_corr.shoulderL = 0 - (715450 - mydata_front_count.shoulderL);
              mydata_front_count_corr.elbowR = 0 - (715450 - mydata_front_count.elbowR);
              mydata_front_count_corr.elbowL = 0 - (715450 - mydata_front_count.elbowL);

              mydata_back_count_corr.hipR =  mydata_back_count.hipR; // no offset to remove
              mydata_back_count_corr.hipL =  mydata_back_count.hipL; // no offset to remove
              mydata_back_count_corr.shoulderR = 0 - (715450 - mydata_back_count.shoulderR);
              mydata_back_count_corr.shoulderL = 0 - (715450 - mydata_back_count.shoulderL);
              mydata_back_count_corr.elbowR = 0 - (715450 - mydata_back_count.elbowR);
              mydata_back_count_corr.elbowL = 0 - (715450 - mydata_back_count.elbowL);

              // filter values
              
              mydata_front_filtered.hipR = filter(mydata_front_count_corr.hipR, mydata_front_filtered.hipR);
              mydata_front_filtered.hipL = filter(mydata_front_count_corr.hipL, mydata_front_filtered.hipL);              
              mydata_front_filtered.shoulderR = filter(mydata_front_count_corr.shoulderR, mydata_front_filtered.shoulderR);
              mydata_front_filtered.shoulderL = filter(mydata_front_count_corr.shoulderL, mydata_front_filtered.shoulderL);
              mydata_front_filtered.elbowR = filter(mydata_front_count_corr.elbowR, mydata_front_filtered.elbowR);
              mydata_front_filtered.elbowL = filter(mydata_front_count_corr.elbowL, mydata_front_filtered.elbowL);

              mydata_back_filtered.hipR = filter(mydata_back_count_corr.hipR, mydata_back_filtered.hipR);
              mydata_back_filtered.hipL = filter(mydata_back_count_corr.hipL, mydata_back_filtered.hipL);              
              mydata_back_filtered.shoulderR = filter(mydata_back_count_corr.shoulderR, mydata_back_filtered.shoulderR);
              mydata_back_filtered.shoulderL = filter(mydata_back_count_corr.shoulderL, mydata_back_filtered.shoulderL);
              mydata_back_filtered.elbowR = filter(mydata_back_count_corr.elbowR, mydata_back_filtered.elbowR);
              mydata_back_filtered.elbowL = filter(mydata_back_count_corr.elbowL, mydata_back_filtered.elbowL);

              // write data to ODrives              

              if(startupFlag == 0) {
                    previousStartupmillis = currentMillis;  // start filter settling timer 
                                        
                    // turn up motor speeds a bit 
                    Serial1 << "w axis" << 0 << ".controller.config.vel_limit " << 440000.0f << '\n';
                    Serial1 << "w axis" << 1 << ".controller.config.vel_limit " << 400000.0f << '\n';

                    Serial2 << "w axis" << 0 << ".controller.config.vel_limit " << 440000.0f << '\n';
                    Serial2 << "w axis" << 1 << ".controller.config.vel_limit " << 400000.0f << '\n';                    
    
                    Serial3 << "w axis" << 0 << ".controller.config.vel_limit " << 192000.0f << '\n';
                    Serial3 << "w axis" << 1 << ".controller.config.vel_limit " << 192000.0f << '\n';

                    Serial4 << "w axis" << 0 << ".controller.config.vel_limit " << 440000.0f << '\n';
                    Serial4 << "w axis" << 1 << ".controller.config.vel_limit " << 400000.0f << '\n';

                    Serial5 << "w axis" << 0 << ".controller.config.vel_limit " << 440000.0f << '\n';
                    Serial5 << "w axis" << 1 << ".controller.config.vel_limit " << 400000.0f << '\n';                    
    
                    Serial6 << "w axis" << 0 << ".controller.config.vel_limit " << 192000.0f << '\n';
                    Serial6 << "w axis" << 1 << ".controller.config.vel_limit " << 192000.0f << '\n';
                        

                    startupFlag = 1; // only do this lot once!
              }

              if (currentMillis - previousStartupmillis > 3000) {   // makes sure filter has settled the first time


                    Serial.print(mydata_front_filtered.shoulderR);

                    /*
                    Serial.print(" , ");
                    Serial.print(mydata_front_filtered.elbowR);
                    Serial.print(" , ");
                    Serial.print(mydata_front_filtered.shoulderL);
                    Serial.print(" , ");
                    Serial.print(mydata_front_filtered.elbowL);
                    Serial.print(" , ");
                    Serial.print(mydata_back_filtered.shoulderR);
                    Serial.print(" , ");
                    Serial.print(mydata_back_filtered.elbowR);
                    Serial.print(" , ");
                    Serial.print(mydata_back_filtered.shoulderL);
                    Serial.print(" , ");
                    Serial.println(mydata_back_filtered.elbowL);
                    */


                    odrive1.SetPosition(0, mydata_front_filtered.shoulderR);     // output position to actuator                            
                    odrive1.SetPosition(0, mydata_front_filtered.shoulderR);     // output position to actuator                               
                    odrive1.SetPosition(1, mydata_front_filtered.elbowR);        // output position to actuator
                    odrive2.SetPosition(0, mydata_front_filtered.shoulderL);     // output position to actuator
                    odrive2.SetPosition(1, mydata_front_filtered.elbowL);        // output position to actuator
                    odrive3.SetPosition(0, mydata_front_filtered.hipL);          // output position to actuator
                    odrive3.SetPosition(1, mydata_front_filtered.hipR);          // output position to actuator

                    odrive4.SetPosition(0, mydata_back_filtered.shoulderR);     // output position to actuator
                    odrive4.SetPosition(1, mydata_back_filtered.elbowR);        // output position to actuator
                    odrive5.SetPosition(0, mydata_back_filtered.shoulderL);     // output position to actuator
                    odrive5.SetPosition(1, mydata_back_filtered.elbowL);        // output position to actuator
                    odrive6.SetPosition(0, mydata_back_filtered.hipL);          // output position to actuator
                    odrive6.SetPosition(1, mydata_back_filtered.hipR);          // output position to actuator  


              }     

}   // end of leg fuction

// filter function

long filter(long lengthOrig, long currentValue) {
  int filter = 100;
  long lengthFiltered =  (lengthOrig + (currentValue * filter)) / (filter + 1);
  return lengthFiltered;  
}

            


