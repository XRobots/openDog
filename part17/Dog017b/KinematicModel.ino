// ********************************************** Kinematic model fuction **********************************************
double leg(double Z4, double Y4, double X4, double yaw, double pitch2, double roll, int side, int front) {

// ********************************************** XYZ Consts and Vars **********************************************
#define HIPROD 118L                                   // offset from hip pivot to middle of leg
#define HIPROD2 263L                                  // diagonal length from hip pivot to fixed actuator pivot
#define HIPROD3 150L                                  // length of hip actuator pivot
#define HIPOFFSETANGLE 32L                            // DEGREES angle of diagnonal between hip pivot and actuator pivot
const double DEGTORAD = 71/4068;                      // Degrees to Radians Constant
const double RADTODEG = 4068/71;                      // Radians to Degrees Constant
double X;                                             // X once offsets and inversion are taken ito account
double X2;                                            // X after yaw moves;
double X3;                                            // X at the roll calc stage
double Y;                                             // Y to give to XYZ calcs
double Y2;                                            // Y after yaw moves
double Z;                                             // Z to give to XYZ cals
double Z2;                                            // Z after yaw moves
double Z3;                                            // Z at the roll calc stage
double hipHypotenuse;                                 // the length between hip pivot and foot
double hipAngle1;                                     // RADIANS - hip angle from vertical
double hipAngle1_DEG;                                 // DEGREES - hip angle from vertical
double hipAngle2;                                     // RADIANS - angle at foot between middle of leg and hip pivot
double hipAngle2_DEG;                                 // DEGREES - angle at foot between middle of leg and hip pivot
double hipAngle3;                                     // RADIANS - angle required from actual actuator
double hipAngle3_DEG;                                 // DEGREES - angle required from actual actuator
double hipActuator;                                   // output length of ball screw
double legHeight;                                     // height of leg in the 'sideways tilting plane'

double shoulderOffset;                                // RADIANS - offset angle to move the actual shoulder by, large angle moves the leg forward
double shoulderOffset_DEG;                            // DEGREES - offset angle to move the actual shoulder by, large angle moves the leg forward
double legLength;

#define DIGITLENGTH 363L                              // length of each top/bottom leg
#define ELBOWROD 190L                                 // length of push rod
#define ELBOWROD2 95L                                 // other side of push rod triangle
#define ELBOWACTANGLE 15.8L                           // angle between actuator and upper leg joint
double elbowAngle;                                    // the actual angle of the elbow between top and bottom sections
double elbowAngle_DEG;                                // the actual angle of the elbow between top and bottom sections
double elbowAngle2;                                   // angle between bottom of leg and actuator
double elbowAngle2_DEG;                               // angle between bottom of leg and actuator
double elbowAngle3;                                   // other angle
double elbowAngle3_DEG;                               // other angle
double elbowAngle4;                                   // other angle
double elbowAngle4_DEG;                               // other angle
double elbowActuator;                                 // calculated length of actuator from joint

#define SHOULDERROD 149L
#define SHOUDLERROD2 102L
#define SHOULDERANGLE2 15.8L
#define SHOULDERANGLE3 17L
double shoulderAngle_DEG;                             // initial demand for shoulder (half elbow angle plus offset for moving the leg
double shoulderAngle2;                                // RADIANS - the actual angle we need from the shoulder actuator
double shoulderAngle2_DEG;                            // DEGREES - the actual angle we need from the shoulder actuator
double shoulderAngle3;                                // RADIANS - first ancgle to solve
double shoulderAngle3_DEG;                            // DEGREES - first ancgle to solve
double shoulderAngle4;                                // RADIANS - second angle to solve
double shoulderAngle4_DEG;                            // Degrees - second angle to solve
double shoulderActuator;                              // actual output length of actuator

// ********************************************** Pitch Roll Yaw Consts and Vars **********************************************
//  Pitch consts and vars - side view
#define BODYRODY 287L                                 // half the distance from shoulder to shoulder on the Y side plane/view
double pitch;                                         // pitch after switching around for front/back of robot
double differenceZ;                                   // difference in height between center point and new shoulder height, in Y axis
double shoulderHeight;                                // resulting new height of shoulder from ground as body tilts in pitch, in a vertical axis from the ground
double legLength2;                                    // length of actual leg we need to calculate the eventual new Y and Z
double distanceY;                                     // the new distance from the centre to the shoulder joint, taking into account the pitch angle
double differenceY;                                   // the difference between foot placement and new distance from centre to shoulder joint
double shoulderAngle5;                                // RADIANS - angle of leg from vertical
double shoulderAngle5_DEG;                            // DEGREES - angle of leg from vertical
double shoulderAngle6;                                // RADIANS - angle of leg from new Z
double shoulderAngle6_DEG;                            // DEGREES - angle of leg from new Z

// Roll consts and vars - end view
#define BODYRODX 110L                                 // half the distance from hip pivot to hip pivot in the X axis
double differenceZ2;                                  // difference in height between the centre point and new hip neight, in an X axis
double hipHeight;                                     // resulting new height of hip from ground as the tits in roll, in a vertical line from the ground
double legLength3;                                    // the new ***length of the VIRTUAL leg*** we need to calulate the new Y and Z
double distanceX;                                     // the new distance from the centre to the hip joint, taking into account the roll angle
double differenceX;                                   // the difference between foot placement and the new distance from the centre to the hip joint
double hipAngle5;                                     // RADIANS the angle of the *VIRTUAL leg* from vertical
double hipAngle5_DEG;                                 // DEGREES the angle of the *VIRTUAL leg* from vertical
double hipAngle6;                                     // RADIANS the angle of the *VIRTUAL* leg from the new Z
double hipAngle6_DEG;                                 // DEGREES the angle of the *VIRTUAL* leg from the new Z

// Yaw consts and vars - top view
double radius;                                        // radius/hypotenuse of triangle of leg from top XY view
double yawAngle1;                                     // RADIANS - the origin angle of the radius using existing stick posotions
double yawAngle1_DEG;                                 // DEGREES - the origin angle of the radius using existing stick posotions
double yawAngle2;                                     // RADIANS - the new demand angle of the radius
double yawAngle2_DEG;                                 // DEGREES - the new demand angle of the radius

// ********************************************** Interpolate values **********************************************
Z4 = interpolationZ(Z4,800);
X4 = interpolationX(X4,800);
Y4 = interpolationY(Y4,800);

// ********************************************** End of Interpolation **********************************************
// front - default is zero
switch (front) {
  case 1:
    Y4 = Y4 + BODYRODY;
    break;
  default:
    Y4 = Y4 - BODYRODY;
    break;
}

// side - default is zero
switch (side) {
  case 1:
    X4 = X4 + (BODYRODX + HIPROD);
    break;
  default:
    X4 = X4 - (BODYRODX + HIPROD);
    break;
}

yaw = yaw*DEGTORAD;                                   // convert to RADIANS

yawAngle1 = atan(X4/Y4);                              // calc existing angle of leg from centre
yawAngle1_DEG = yawAngle1*RADTODEG;                   // DEGREES - for debug

radius = X4/sin(yawAngle1);                           // calc radius from centre

yawAngle2 = yawAngle1 + yaw;
yawAngle2_DEG = yawAngle2*RADTODEG;                   // DEGREES - for debug

X2 = radius * sin(yawAngle2);                         // calc new X and Y based on new yaw angle
Y2 = radius * cos(yawAngle2);

// remove offsets again to give results centred around zero
// front - default is zero
switch (front) {
  case 1:
    Y2 = Y2 - BODYRODY;
    break;
  default:
    Y2 = Y2 + BODYRODY;
    break;
}

// side - default is zero
switch (side) {
  case 1:
    X2 = X2 - (BODYRODX + HIPROD);
    break;
  default:
    X2 = X2 + (BODYRODX + HIPROD);
    break;
}

Z2 = Z4;                                              // pass through Z as it isn't needed in yaw calcs

// switch round X and roll for each side of the robot
switch (side) {
  case 1:
    X3 = HIPROD + X2;                                 // right leg
    break;
  default:
    X3 = HIPROD - X2;                                 // left leg
    break;
}

// switch around pitch angle for back or front of robot
switch (front) {
  case 1:
    pitch = pitch2*-1;                                // front of robot
    break;
  default:
    pitch = pitch2;                                   // back of robot
    break;
}

// ********************************************** Pitch calcs **********************************************

pitch = pitch*DEGTORAD;                               // convert pitch to RADIANS
differenceZ = sin(pitch) * BODYRODY;                  // calc opposite side of triangle
distanceY = cos(pitch) * BODYRODY;                    // calc adjacent side of triangle

shoulderHeight = Z2 + differenceZ;                    // calc new shoulder height as pitch changes
differenceY = Y2 + (BODYRODY - distanceY);            // distance from original foot position and vertical line to shoulder

if (front == 1) {                                     // switch around the data if it's the front of the robot
  differenceY = differenceY*-1;
}

shoulderAngle5 = atan(differenceY/shoulderHeight);    // RADIANS angle between leg and vetical
shoulderAngle5_DEG = shoulderAngle5*RADTODEG;            // DEGREES convert for debug
legLength2 = shoulderHeight/cos(shoulderAngle5);      // calc out new length of leg

shoulderAngle6 = pitch+shoulderAngle5;                // RADIANS calc angle for new Z to 90' from body

if (front == 1) {                                     // switch around the data if it's the front of the robot
  shoulderAngle6 = shoulderAngle6*-1;
}

shoulderAngle6_DEG = shoulderAngle6*RADTODEG;         // DEGREES convert for debug

Z3 = legLength2*cos(shoulderAngle6);                  // new Z to pass on to roll calcs
Y = legLength2*sin(shoulderAngle6);                   // new Y to pass on to roll cals

// ********************************************** Roll Calcs **********************************************
roll = (roll*DEGTORAD)*-1;                            // convert roll to RADIANS
differenceZ2 = sin(roll) * BODYRODX;                  // calc opposite side of triangle
distanceX = cos(roll) * BODYRODX;                     // calc adjacent side of triangle

if (side == 0) {                                      // *********** switch around for each side of the robot
    hipHeight = Z3 - differenceZ2;                    // new height of the hip from the ground taking into account roll
}
if (side == 1) {
    hipHeight = Z3 + differenceZ2;
}

differenceX = X3 + (BODYRODX - distanceX);            // distance from original foot postion and vertical line to shoulder.

if (side == 1) {                                      // *********** switch around for each side of the robot
  differenceX = differenceX*-1;
}

hipAngle5 = atan(differenceX/hipHeight);              // RADIANS angle between *virtual* leg and vertical
hipAngle5_DEG = hipAngle5*RADTODEG;                   // DEGREES convert for debug

legLength3 = hipHeight/cos(hipAngle5);                // calc new *virtual leg* length

hipAngle6 = roll+hipAngle5;                           // RADIANS calc angle for new Z to 90' from body
hipAngle6_DEG = hipAngle6*RADTODEG;                  // DEGREES convert for debug

if (side == 1) {                                      // *********** switch around for each side of the robot
  hipAngle6 = hipAngle6 *-1;
}

X = legLength3*sin(hipAngle6);                        // new X to pass onto XYZ calcs
Z = legLength3*cos(hipAngle6);                        // new Z to pass onto XYZ calcs

// ********************************************** X & Z axis calc for each leg **********************************************
if (X == 0) { X = 0.1; }                                                                            // avoid divide by zero
hipAngle1 = atan(X / Z);                                                                            // calc hip angle from vertical
hipAngle1_DEG = hipAngle1*RADTODEG;                                                                    // convert to degrees for debug
hipHypotenuse = X / sin(hipAngle1);                                                                 // calc distance from hip pivot to foot

hipAngle2 = asin(HIPROD / hipHypotenuse);                                                           // calc angle at foot between middle of leg and hip pivot
hipAngle2_DEG = hipAngle2*RADTODEG;                                                                 // convert to degrees for debug

legHeight = HIPROD / tan(hipAngle2);                                                                // leg height from 'sideways tilting plane' of leg

hipAngle3_DEG = (90 - HIPOFFSETANGLE) + (hipAngle1_DEG + (180-90-hipAngle2_DEG)) -90 ;              // DEGREES - angle required from actuator
hipAngle3 = hipAngle3_DEG*DEGTORAD;                                                                    // convert to radians

//a2 = b2 + c2 − 2bc cosA
hipActuator =  sqrt( sq(HIPROD3)+sq(HIPROD2) - (2 * HIPROD3 * HIPROD2 * cos(hipAngle3)) )-223;      // work out actuator length relative to mid positon

// ********************************************** Y axis calcs **********************************************
if (Y == 0) { Y = 0.1; }                                                                            // avoid divide by zero
shoulderOffset = atan((Y*-1) / legHeight);                                                          // calc shoulder angle offset
shoulderOffset_DEG = shoulderOffset*RADTODEG;                                                       // convert radians to degrees
legLength = (Y*-1) / sin(shoulderOffset);                                                           // calc hypotenuse of triangle to make actual leg length

// ********************************************** Elbow Calcs **********************************************

elbowAngle = acos ( (sq(DIGITLENGTH) + sq(DIGITLENGTH) - sq(legLength)) / (2 * DIGITLENGTH * DIGITLENGTH) );
elbowAngle_DEG = elbowAngle_DEG*RADTODEG;                                                          // convert radians to degrees

elbowAngle2_DEG = elbowAngle_DEG - ELBOWACTANGLE;                                                   // take away known angle
elbowAngle2 = elbowAngle2_DEG*DEGTORAD;                                                             // convert degrees to radians

elbowAngle3 = asin ( (ELBOWROD2 * sin(elbowAngle2)/ELBOWROD));
elbowAngle3_DEG = elbowAngle3*RADTODEG;                                                             // convert radians to degrees

elbowAngle4_DEG = 180 - elbowAngle2_DEG - elbowAngle3_DEG;                                          // we now know all four angles
elbowAngle4 = elbowAngle4_DEG*DEGTORAD;                                                             // convert degrees to radians

elbowActuator = (((sin(elbowAngle4)) * ELBOWROD)) / sin(elbowAngle2);

// ********************************************** shoulder calcs **********************************************
shoulderAngle_DEG = (180-elbowAngle_DEG)/2 + shoulderOffset_DEG;                                    // initial demand for shoulder (half elbow angle plus offset for moving the leg)
shoulderAngle2_DEG = 90-shoulderAngle_DEG-SHOULDERANGLE2+SHOULDERANGLE3;                            // calc actual angle required from shoulder
shoulderAngle2 = shoulderAngle2_DEG*DEGTORAD;                                                       // convert degrees to radians

shoulderAngle3 = asin ((SHOUDLERROD2 * sin(shoulderAngle2))  /  SHOULDERROD) ;                      // calc first unknown angle
shoulderAngle3_DEG = shoulderAngle3*RADTODEG;                                                       // convert radians to degrees

shoulderAngle4_DEG = 180 - shoulderAngle2_DEG - shoulderAngle3_DEG;                                 // calc out remaining angle
shoulderAngle4 = shoulderAngle4_DEG*DEGTORAD;                                                     // convert degrees to radians

shoulderActuator =  ( (sin(shoulderAngle4)) * SHOULDERROD ) / sin(shoulderAngle2);



// ********************************************** output variables **********************************************
  if (side == 1 && front == 1) {                      // front right leg
    mydata_front.hipR = hipActuator;
    mydata_front.shoulderR = shoulderActuator;
    mydata_front.elbowR = elbowActuator;

  }
  else if (side == 1 && front == 0) {                 // back right leg
    mydata_back.hipR = hipActuator;
    mydata_back.shoulderR = shoulderActuator;
    mydata_back.elbowR = elbowActuator;

  }
  else if (side == 0 && front  == 1) {              // front right leg
    mydata_front.hipL = hipActuator;
    mydata_front.shoulderL = shoulderActuator;
    mydata_front.elbowL = elbowActuator;

  }
  else if (side == 0 && front == 0) {                 // back right leg
    mydata_back.hipL = hipActuator;
    mydata_back.shoulderL = shoulderActuator;
    mydata_back.elbowL = elbowActuator;

  }

  // constrain data so we don't run the actuator end stops
  mydata_front.shoulderR = constrain(mydata_front.shoulderR, 110,200);
  mydata_front.shoulderL = constrain(mydata_front.shoulderL, 110,200);
  mydata_front.elbowR = constrain(mydata_front.elbowR,110,200);
  mydata_front.elbowL = constrain(mydata_front.elbowL, 110,200);
  mydata_front.hipR = constrain(mydata_front.hipR, -35,35);
  mydata_front.hipL = constrain(mydata_front.hipL, -35,35);

  mydata_back.shoulderR = constrain(mydata_back.shoulderR, 110,200);
  mydata_back.shoulderL = constrain(mydata_back.shoulderL, 110,200);
  mydata_back.elbowR = constrain(mydata_back.elbowR, 110,200);
  mydata_back.elbowL = constrain(mydata_back.elbowL, 110,200);
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
  mydata_front_count_corr.hipR =  mydata_front_count.hipR;                                          // no offset to remove
  mydata_front_count_corr.hipL =  mydata_front_count.hipL;                                          // no offset to remove
  mydata_front_count_corr.shoulderR = 0 - (715450 - mydata_front_count.shoulderR);
  mydata_front_count_corr.shoulderL = 0 - (715450 - mydata_front_count.shoulderL);
  mydata_front_count_corr.elbowR = 0 - (715450 - mydata_front_count.elbowR);
  mydata_front_count_corr.elbowL = 0 - (715450 - mydata_front_count.elbowL);

  mydata_back_count_corr.hipR =  mydata_back_count.hipR;                                            // no offset to remove
  mydata_back_count_corr.hipL =  mydata_back_count.hipL;                                            // no offset to remove
  mydata_back_count_corr.shoulderR = 0 - (715450 - mydata_back_count.shoulderR);
  mydata_back_count_corr.shoulderL = 0 - (715450 - mydata_back_count.shoulderL);
  mydata_back_count_corr.elbowR = 0 - (715450 - mydata_back_count.elbowR);
  mydata_back_count_corr.elbowL = 0 - (715450 - mydata_back_count.elbowL);

  // filter final position values
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
        previousStartupmillis = currentMillis;                                                      // start filter settling timer

        // Front
        Serial1 << "w axis" << 0 << ".controller.config.vel_limit " << 220000.0f << '\n';           // Front Shoulder R
        Serial1 << "w axis" << 1 << ".controller.config.vel_limit " << 220000.0f << '\n';           // Front Elbow R
        Serial2 << "w axis" << 0 << ".controller.config.vel_limit " << 220000.0f << '\n';           // Front Shoulder L
        Serial2 << "w axis" << 1 << ".controller.config.vel_limit " << 220000.0f << '\n';           // Front ELbow L
        Serial3 << "w axis" << 0 << ".controller.config.vel_limit " << 200000.0f << '\n';           // Front Undercarriage L
        Serial3 << "w axis" << 1 << ".controller.config.vel_limit " << 200000.0f << '\n';           // Front Undercarriage R
        // Back
        Serial4 << "w axis" << 0 << ".controller.config.vel_limit " << 220000.0f << '\n';           // Back Shoulder R
        Serial4 << "w axis" << 1 << ".controller.config.vel_limit " << 220000.0f << '\n';           // Back Elbow R
        Serial5 << "w axis" << 0 << ".controller.config.vel_limit " << 220000.0f << '\n';           // Back Shoulder L
        Serial5 << "w axis" << 1 << ".controller.config.vel_limit " << 220000.0f << '\n';           // Back Elbow L
        Serial6 << "w axis" << 0 << ".controller.config.vel_limit " << 220000.0f << '\n';           // Back undercarriage L
        Serial6 << "w axis" << 1 << ".controller.config.vel_limit " << 220000.0f << '\n';           // Back undercarriage R

        startupFlag = 1;                                                                            // only do the stuff above once!
  }

  if (currentMillis - previousStartupmillis > 3000) {                                               // makes sure filter has settled the first time

        odrive1.SetPosition(0, mydata_front_filtered.shoulderR);                                    // output position to actuator
        odrive1.SetPosition(1, mydata_front_filtered.elbowR);                                       // output position to actuator
        odrive2.SetPosition(0, mydata_front_filtered.shoulderL);                                    // output position to actuator
        odrive2.SetPosition(1, mydata_front_filtered.elbowL);                                       // output position to actuator
        odrive3.SetPosition(0, mydata_front_filtered.hipL);                                         // output position to actuator
        odrive3.SetPosition(1, mydata_front_filtered.hipR);                                         // output position to actuator

        odrive4.SetPosition(0, mydata_back_filtered.shoulderR);                                     // output position to actuator
        odrive4.SetPosition(1, mydata_back_filtered.elbowR);                                        // output position to actuator
        odrive5.SetPosition(0, mydata_back_filtered.shoulderL);                                     // output position to actuator
        odrive5.SetPosition(1, mydata_back_filtered.elbowL);                                        // output position to actuator
        odrive6.SetPosition(0, mydata_back_filtered.hipL);                                          // output position to actuator
        odrive6.SetPosition(1, mydata_back_filtered.hipR);                                          // output position to actuator

  }

}                                                                                                   // end of leg fuction

// filter function
long filter(long lengthOrig, long currentValue) {
  int filter = 15;
  long lengthFiltered =  (lengthOrig + (currentValue * filter)) / (filter + 1);
  return lengthFiltered;
}
