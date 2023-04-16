/***************************************/
/*   Group C                           */
/*   04/14/2023                        */
/*   In Class Performance Test 1         */
/***************************************/


#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHBuzzer.h>
#include <stdio.h>
#include <FEHservo.h>


// Motor Inputs
DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P0_2);
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor1,9.0);
FEHServo arm_servo(FEHServo::Servo7);


// Bumper Switch Inputs
DigitalInputPin frontSwitchLeft(FEHIO::P1_2);
DigitalInputPin frontSwitchRight(FEHIO::P1_4);


// CdS Cell Input
AnalogInputPin sensor(FEHIO::P1_0);


// Line Following Inputs
AnalogInputPin leftLineF(FEHIO::P2_0); // BLUES
AnalogInputPin centerLineF(FEHIO::P2_3);  // GREYS
AnalogInputPin rightLineF(FEHIO::P2_7);  // REDS


//Define cool constants
#define LEFT_TURN_PERCENT 40
#define RIGHT_TURN_PERCENT 40
#define LEFT_RAMPING_SPEED 50
#define RIGHT_RAMPING_SPEED 50
#define PULSE_DRIVE 20
#define PULSE_TURN 20
#define PULSE_DRIVE_TIME 0.1
#define PULSE_TURN_TIME 0.1
#define FUEL_LEVER_DOWN 35
#define FUEL_LEVER_UP 70


void NO() // This is a permanant while loop that stops all code
{
    LCD.Clear(SCARLET);
    while(true)
    {
        LCD.WriteLine("NO");
    }
}


void drive(int FoB, int percent, float inches)
{
    // drive(0) will drive backwards
    // drive(1) will drive forwards
    // Added inches to counter converter, so
    // we will only need to enter the inches we need to travel
    // this can be extended to RPS units once we know the conversion


    // Declare counts
    float counts;


    // convert inches to tic counts
    counts = (inches*318)/11;


    // Set forward (1) or backward (0)
    if(FoB==1)
    {
        percent = percent;
    }
    if(FoB==0)
    {
        percent = -percent;
    }


    // Set counts to zero
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();


    //Set both motors to percent speed WILL NEED ADJUSTMENT FOR MOTORS
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);


    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2 < counts);


    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}


void driveTime(int FoB, int percent, float inches, float duration)
{
    // drive(0) will drive backwards
    // drive(1) will drive forwards
    // Added inches to counter converter, so
    // we will only need to enter the inches we need to travel
    // this function also has a time out


    // set up time parameter
    float startTime = TimeNow();


    // Declare counts
    float counts;


    // convert inches to tic counts
    counts = (inches*318)/11;


    // Set forward (1) or backward (0)
    if(FoB==1)
    {
        percent = percent;
    }
    if(FoB==0)
    {
        percent = -percent;
    }


    // Set counts to zero
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();


    //Set both motors to percent speed WILL NEED ADJUSTMENT FOR MOTORS
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);


    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while(((left_encoder.Counts() + right_encoder.Counts()) / 2 < counts) && ((TimeNow() - startTime) < duration));


    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}


void turnDegree(int direction, int degree) //This function turns the robot a set degree using one of its wheels as a pivot point
{
    // turn(0) turns to the left (counterclockwise)
    // turn(1) turns to the right (clockwise)
   
    // reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();


    // creates a variable that sets the maximum amount of counts (318 counts in a full rotation of a wheel)
    int maxCounts;


    //Variable for the width of the robot, which acts as the turning radius
    float radius=7;


    //convert degree into radians
    float radian;
    radian=degree*3.14159265358979323846264338327/180;


    //Calculate maxCounts
    maxCounts=radius*radian*318/11;


    if(direction==0)
    {
        // set right motor power full speed forward
        right_motor.SetPercent(RIGHT_TURN_PERCENT);        
    }
    else
    {
        // set left motor power full speed backward
        left_motor.SetPercent(LEFT_TURN_PERCENT);
    }
   
    //Turn through the maxCounts
    while((left_encoder.Counts() + right_encoder.Counts())<maxCounts);


    // Turn off motors once maxCounts are reached
    right_motor.Stop();
    left_motor.Stop();
}


void turnDegreeCenter(int direction, int degree) //This function turns the robot a set degree using one of its wheels as a pivot point
{
    // turn(0) turns to the left (counterclockwise)
    // turn(1) turns to the right (clockwise)
   
    // reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();


    // creates a variable that sets the maximum amount of counts (318 counts in a full rotation of a wheel)
    int maxCounts;


    //Variable for the width of the robot, which acts as the turning radius
    float radius=3.5;


    //convert degree into radians
    float radian;
    radian=degree*3.14159265358979323846264338327/180;


    //Calculate maxCounts
    maxCounts=radius*radian*318/11;


    if(direction==0)
    {
        // set right motor power full speed forward
        right_motor.SetPercent(RIGHT_TURN_PERCENT);
        left_motor.SetPercent(-LEFT_TURN_PERCENT);        
    }
    else
    {
        // set right motor power full speed backward
        left_motor.SetPercent(LEFT_TURN_PERCENT);
        right_motor.SetPercent(-RIGHT_TURN_PERCENT);
    }
   
    //Turn through the maxCounts
    while(((left_encoder.Counts() + right_encoder.Counts())/2)<maxCounts);


    // Turn off motors once maxCounts are reached
    right_motor.Stop();
    left_motor.Stop();
}


void turnDegreeBackwards(int direction, int degree) //This function turns the robot a set degree backwards using one of its wheels as a pivot point
{
    // turn(0) turns to the left (clockwise)
    // turn(1) turns to the right (counterclockwise)
   
    // reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();


    // creates a variable that sets the maximum amount of counts (318 counts in a full rotation of a wheel)
    int maxCounts;


    //Variable for the width of the robot, which acts as the turning radius
    float radius=7;


    //convert degree into radians
    float radian;
    radian=degree*3.14159265358979323846264338327/180;


    //Calculate maxCounts
    maxCounts=radius*radian*318/11;


    if(direction==0)
    {
        // set right motor
        right_motor.SetPercent(-RIGHT_TURN_PERCENT);        
    }
    else
    {
        // set left motor power
        left_motor.SetPercent(-LEFT_TURN_PERCENT);
    }
   
    //Turn through the maxCounts
    while((left_encoder.Counts() + right_encoder.Counts())<maxCounts);


    // Turn off motors once maxCounts are reached
    right_motor.Stop();
    left_motor.Stop();
}


void checkHeading(float cDegree, float duration) // This function will check the RPS heading and make corrections
{
    // Set up time parameter
    float startTime=TimeNow();


    if(RPS.Heading() >= 0) // Make sure you aren't in the deadzone
    {
        if(cDegree == 0) // Special case if the heading is zero
        {
            while(((RPS.Heading() < 355) && (RPS.Heading() > 5)) && ((TimeNow()-startTime) < duration)) // Parameters for turning
            {
                if((RPS.Heading() > 5) && (RPS.Heading() < 180)) // Bigger heading, need to rotate clockwise
                {
                    left_motor.SetPercent(PULSE_TURN); // Pulse
                    right_motor.SetPercent(-PULSE_TURN);


                    Sleep(PULSE_TURN_TIME); // Wait a sec


                    left_motor.Stop(); //Stop
                    right_motor.Stop();
                }
                else if((RPS.Heading() < 355) && (RPS.Heading() > 180)) // Smaller heading, need to rotate counterclockwise
                {
                    left_motor.SetPercent(-PULSE_TURN); // Pulse
                    right_motor.SetPercent(PULSE_TURN);


                    Sleep(PULSE_TURN_TIME); // Wait a sec


                    left_motor.Stop(); //Stop
                    right_motor.Stop();
                }
                else
                {
                    // do nothing
                    LCD.Clear();
                }
            }
        }
        else if(cDegree != 0) // Case for all the NORMAL degrees out there
        {
            while(((RPS.Heading() < (cDegree-5)) || (RPS.Heading() > (cDegree+5))) && ((TimeNow()-startTime) < duration)) // Parameters for turning
            {
                if(RPS.Heading() > (cDegree+5)) // Bigger heading, need to rotate clockwise
                {
                    left_motor.SetPercent(PULSE_TURN); // Pulse
                    right_motor.SetPercent(-PULSE_TURN);


                    Sleep(PULSE_TURN_TIME); // Wait a sec


                    left_motor.Stop(); //Stop
                    right_motor.Stop();
                }
                else if(RPS.Heading() < (cDegree-5)) // Smaller heading, need to rotate counterclockwise
                {
                    left_motor.SetPercent(-PULSE_TURN); // Pulse
                    right_motor.SetPercent(PULSE_TURN);


                    Sleep(PULSE_TURN_TIME); // Wait a sec


                    left_motor.Stop(); //Stop
                    right_motor.Stop();
                }
                else
                {
                    // do nothing
                    LCD.Clear();
                }
            }
        }
        else // You messed up lol
        {
            LCD.Clear();
            LCD.WriteLine("Invalid heading");
        }
    }
}


void checkX(float correctX, float duration, int facing) // RPS check in the x direction, facing=0 for -x direction and facing=1 for +x
{
    // Set up time parameter
    float startTime=TimeNow();


    // Establish direction
    int pulse;
    if(facing == 0)
    {
        //You are facing the -x direction
        pulse = -PULSE_DRIVE;
    }
    else
    {
        //You are facing the -x direction
        pulse = PULSE_DRIVE;
    }


    if(RPS.Heading() >= 0.0) // Make sure you aren't in the deadzone
    {
        while(((RPS.X() < (correctX-1.0)) || (RPS.X() > (correctX+1.0))) && ((TimeNow()-startTime) < duration)) // parameters for checking
        {
            if(RPS.X() < (correctX-1.0)) //need to increase X coord
            {
                left_motor.SetPercent(pulse);
                right_motor.SetPercent(pulse);


                Sleep(PULSE_DRIVE_TIME);


                left_motor.Stop();
                right_motor.Stop();
            }
            else
            {
                left_motor.SetPercent(-pulse);
                right_motor.SetPercent(-pulse);


                Sleep(PULSE_DRIVE_TIME);


                left_motor.Stop();
                right_motor.Stop();
            }
        }
    }
}


void checkY(float correctY, float duration, int facing) // RPS check in the y direction, facing=0 for -y direction and facing=1 for +y
{
    // Set up time parameter
    float startTime=TimeNow();


    // Establish direction
    int pulse;
    if(facing == 0)
    {
        //You are facing the -y direction
        pulse = -PULSE_DRIVE;
    }
    else
    {
        //You are facing the -y direction
        pulse = PULSE_DRIVE;
    }


    if(RPS.Heading() >= 0.0) // Make sure you aren't in the deadzone
    {
        while(((RPS.Y() < (correctY-1.0)) || (RPS.Y() > (correctY+1.0))) && ((TimeNow()-startTime) < duration)) // parameters for checking
        {
            if(RPS.Y() < (correctY-1.0)) //need to increase X coord
            {
                left_motor.SetPercent(pulse);
                right_motor.SetPercent(pulse);


                Sleep(PULSE_DRIVE_TIME);


                left_motor.Stop();
                right_motor.Stop();
            }
            else
            {
                left_motor.SetPercent(-pulse);
                right_motor.SetPercent(-pulse);


                Sleep(PULSE_DRIVE_TIME);


                left_motor.Stop();
                right_motor.Stop();
            }
        }
    }  
}


void displaySensorValues() //This function records and displays our values from the Cds cells and line following sensors, for testing purposes
{
    //Declare variables
    float CdS, leftLine, centerLine, rightLine;
    char s[30];


    while(true)
    {
        //Set variables equal to their respective values
        CdS=sensor.Value();
        leftLine=leftLineF.Value();
        centerLine=centerLineF.Value();
        rightLine=rightLineF.Value();


        //Clear the screen and print the values
        LCD.Clear();
        sprintf(s, "%f %f %f %f", CdS,leftLine,centerLine,rightLine);
        LCD.WriteLine(s);


        //Sleep so you got time to read
        Sleep(0.2);
    }
}


void bumperTest() //This function tests the bumper plate switches by displaying their values to the screen
{
    //Declare variables and clear screen
    int leftSwitch, rightSwitch;
    char s[30];
    LCD.Clear();


    while(true) //Take values from the switches and then print them to the screen continuously
    {
        leftSwitch=frontSwitchLeft.Value();
        rightSwitch=frontSwitchRight.Value();


        sprintf(s, "Left = %i and Right = %i", leftSwitch, rightSwitch);
        LCD.WriteLine(s);
    }
}


int determineColor() //This function determines the color of the light for the boarding pass and then moves to the correct light and displays it
{
    //Read the value of the CdS sensor
    float CdS;
    int return_color;


    //Sleep to get good reading
    Sleep(1.0);


    //read the value
    CdS=sensor.Value();


    //For Red
    if(CdS < 0.5)
    {
        //Make screen red
        LCD.Clear(RED);
        return_color = 0;
    }
    //For Blue
    else if(CdS > 0.5)
    {
        //Make screen blue
        LCD.Clear(BLUE);  
        return_color = 1;
    }
    else //error
    {
        return_color = 2;
    }
    return return_color;
}


void goToWall(int percent, float T) //This function drives straight until we hit a wall, or cuts out after a certain time
{
    LCD.Clear();
    LCD.WriteLine("DRIVING to waLL");
    float startTime=TimeNow();
    while((frontSwitchLeft.Value() || frontSwitchRight.Value()) && ((TimeNow()-startTime)<T))
    {
        //Drive forward with percent power
        left_motor.SetPercent(percent+1);
        right_motor.SetPercent(percent);


        // Print ellapsed time status
        LCD.Clear();
        LCD.WriteLine("wall stuff");
        LCD.WriteLine(TimeNow()-startTime);
    }


    //Stop Motors
    left_motor.Stop();
    right_motor.Stop();


    //Kowalski, status report
    LCD.Clear();
    LCD.WriteLine("Skipper, I appear to have hit a wall");


    //Sleep for a sec
    Sleep(0.5);
}


void searchForLight() // This function drives forward until the robot detects the light
{
    int CdS;
    float startTime=TimeNow();


    while((CdS=sensor.Value()>2.5) && (TimeNow()-4.0 < startTime))
    {
        //Drive until you find the light
        left_motor.SetPercent(20);
        right_motor.SetPercent(20);
    }


    //Stop motors
    left_motor.Stop();
    right_motor.Stop();
}


void initializeServo()
{
    // set servo min and max
    arm_servo.SetMin(500);
    arm_servo.SetMax(2500);
}


void hitLever(int cLever) // This function hits the correct lever
{
    // Take a breather before the lever
    LCD.Clear();
    LCD.WriteLine("About to hit lever");
    Sleep(1.0);


    // Cases for each lever
    if(cLever == 0) // Lever A
    {
        // back up a set amount
        drive(0, 30, 7);


        // turn backwards to get near the lever
        turnDegreeBackwards(1, 95);


        // drive forward a bit
        drive(1, 30, 1);
        checkY(24.0, 2.0, 0);


        // Flip the lever down
        LCD.Clear(); // Status Report
        LCD.WriteLine("Flipping Lever A");
        arm_servo.SetDegree(FUEL_LEVER_DOWN); // Lower Arm to flip lever down
        Sleep(0.5);


        // Drive back to let arm under the lever
        drive(0, 20, 3);
        arm_servo.SetDegree(15);


        //Wait 5 seconds
        LCD.Clear();
        LCD.WriteLine("Lever A Down, waiting 5 seconds");
        Sleep(5.0);
       
        // drive forward and raise arm to lift lever
        drive(1, 30, 3);
        arm_servo.SetDegree(FUEL_LEVER_UP);


        LCD.Clear(); // Status Report
        LCD.WriteLine("Lever A Back Up!");


        //Sleep a bit
        Sleep(1.0);
    }
    else if(cLever == 1) // Lever A1
    {
        // back up a set amount
        drive(0, 30, 4);


        // turn backwards to get near the lever
        turnDegreeBackwards(1, 95);


        // drive forward a bit
        drive(1, 30, 1);
        checkY(24.0, 2.0, 0);


        // flip the lever down
        LCD.Clear(); // Status Report
        LCD.WriteLine("Flipping Lever A1");
        arm_servo.SetDegree(FUEL_LEVER_DOWN); //flip lever now
        Sleep(0.5);


        // Drive back to let arm under the lever
        drive(0, 20, 3);
        arm_servo.SetDegree(15);


        //Wait 5 seconds
        LCD.Clear();
        LCD.WriteLine("Lever A1 Down, waiting 5 seconds");
        Sleep(5.0);


        // drive forward and raise arm to lift lever
        drive(1, 30, 3);
        arm_servo.SetDegree(FUEL_LEVER_UP);


        LCD.Clear(); // Status Report
        LCD.WriteLine("Lever A1 Back Up!");


        //Sleep a bit
        Sleep(1.0);
    }
    else if(cLever == 2) //Lever B
    {
        // back up a set amount
        //drive(0, 30, 1);


        // turn backwards to get near the lever
        turnDegreeBackwards(1, 95);


        // drive forward a bit
        drive(1, 30, 1);
        checkY(24.0, 2.0, 0);


        // Flip the lever
        LCD.Clear(); // Status Report
        LCD.WriteLine("Flipping Lever B");
        arm_servo.SetDegree(FUEL_LEVER_DOWN); // Flip Lever Down
        Sleep(0.5);


        // Drive back to let arm under the lever
        drive(0, 20, 3);
        arm_servo.SetDegree(15);


        //Wait 5 seconds
        LCD.Clear();
        LCD.WriteLine("Lever B Down, waiting 5 seconds");
        Sleep(5.0);


        // drive forward and raise arm to lift lever
        drive(1, 30, 3);
        arm_servo.SetDegree(FUEL_LEVER_UP);


        LCD.Clear(); // Status Report
        LCD.WriteLine("Lever B Back Up!");


        //Sleep a bit
        Sleep(1.0);
    }
    else //Error
    {
       LCD.Clear(); // Status Report
       LCD.WriteLine("Invalid Lever");
    }


    // Back up a bit in preparation to leave
    drive(0, 30, 4);


    // Raise arm back up
    arm_servo.SetDegree(180);
}


void rpsInfo() // Displays RPS Data
{
    while(true)
    {
        LCD.Clear();
        LCD.Write(RPS.X());
        LCD.Write(RPS.Y());
        LCD.Write(RPS.Heading());
        LCD.Write(RPS.GetCorrectLever());
        Sleep(2.0);
    }
}


void initialize() // This contains all the initialization code for the robot
{
    // Set Min and Max for Servo and initialize RPS
    initializeServo();
    arm_servo.SetDegree(180);
    RPS.InitializeTouchMenu();
}


void CDSstart() // This contains the code to start with the CDS cell input
{
    //Declare CdS variable
    float CdS=sensor.Value();


    // code for start with light
    LCD.Clear();


    // Set up time parameter
    float startTime = TimeNow();


    // read the value of sensor
    while((CdS>2) && ((TimeNow() - startTime) < 30.0))
    {
        CdS=sensor.Value();
        LCD.Clear(); // Status report
        LCD.WriteLine("Waiting for start light");
        Sleep(1.0);
    }
    LCD.WriteLine("Starting!"); //Status report
}


void luggage() // This function deposits the luggage in the big ol box (idk if we're doing high or low yet)
{
    // Drop Servo
    arm_servo.SetDegree(100);


    // Sleep and status
    LCD.Clear();
    LCD.WriteLine("The package has been delivered");
    Sleep(0.5);


    // Drive backwards a bit
    drive(0, 30, 6);


    // Raise arm
    arm_servo.SetDegree(180);
}


void MEGAFLIP() // Flips the passport lever up really hard so it bounces back down
{
    Sleep(0.5);
    arm_servo.SetDegree(180);
    drive(0,15,1);
    Sleep(0.5);
}


void hitBoardingPassButton(int color) // Hit the proper passport button bassed on the read color
{
    //Go backwards a bit then turn 90 degrees backwards
    drive(0,30,3);
    turnDegreeBackwards(0,95);


    //Hit the proper button
    if(color == 0) //red
    {
        //status report
        LCD.Clear();
        LCD.WriteLine("going to red button");


        //Drive to button
        drive(1,30,9);
        turnDegree(0,90);
        goToWall(30, 1.5);


        //status report
        LCD.Clear();
        LCD.WriteLine("ayo Mr. White I hit the red button");
    }
    else if(color == 1) //blue
    {
        //status report
        LCD.Clear();
        LCD.WriteLine("going to blue button");


        //Drive to button
        drive(1,30,5);
        turnDegree(0,90);
        goToWall(30, 1.5);


        //status report
        LCD.Clear();
        LCD.WriteLine("ayo Mr. White I hit the blue button");
    }
    else //error, just go blue
    {
        //Status report
        LCD.Clear(BLACK);
        LCD.WriteLine("Error:invalid color");


        Sleep(1.0);


        //status report
        LCD.Clear();
        LCD.WriteLine("going to blue button");


        //Drive to button
        drive(1,30,5);
        turnDegree(0,90);
        goToWall(30, 1.5);


        //status report
        LCD.Clear();
        LCD.WriteLine("ayo Mr. White I hit the blue button");
    }
}


int main(void)
{  
    // Startup
    initialize();


    // Set up touch variables
    float a, b;


    // Wait for the CDS signal to start
    LCD.Clear(); // status
    LCD.WriteLine("Touch to initiate CDS start");
    while(!LCD.Touch(&a,&b)); // Wait for touch to signal final start
    while(LCD.Touch(&a,&b));
    CDSstart();


    // Get the data for the lever from RPS
    int cLever = RPS.GetCorrectLever();


    // Drive forward 9 inches
    drive(1, 30, 9);


    // Turn a bit to the right
    turnDegree(1, 52);


    // go to the wall
    goToWall(30, 2.0);


    // Deposit Luggage
    luggage();


    // Turn backwards 90 degrees & check heading
    turnDegreeBackwards(1, 97);
    checkHeading(85, 5.0);


    // Drive towards wall
    goToWall(30, 4.0);


    // Navigate into position and hit the correct lever
    hitLever(cLever);


    // Drive all the way to the wall by the start button
    turnDegree(0, 96); // Orients to face wall
    drive(1, 50, 12); // Go forward
    goToWall(30, 2.0); // Hit the wall


    // Orient to go up the ramp
    drive(0, 30, 1); // back up a bit
    turnDegreeBackwards(1, 97); // Turn backwards


    // Approach and scale the ramps
    drive(1, 30, 3); // Approach
    checkY(20, 3.0, 1); // Check
    drive(1, 50, 14); // Go up
    drive(1, 45, 6); // Go forward to clear ramp


    // Navigate towards passport
    arm_servo.SetDegree(10); // Lower arm
    drive(1,15,2);
    turnDegree(0,15);
    drive(1,20,3.5);
    turnDegree(1,15);
    driveTime(1, 15, 2, 1.5);


    // Pull the lever kronk
    MEGAFLIP();


    // Navigate to the far wall
    drive(0, 30, 5);
    turnDegree(0, 95);
    goToWall(30, 3.5);


    // After hitting wall, drive backwards to get into place to go to the light
    drive(0, 20, 9.3);
    checkX(19.4, 3.0, 0);
    turnDegree(1, 97);


    // Go to the light
    searchForLight();


    // Determine the color of the light and then display it and go hit the correct button
    int color;
    color=determineColor();
    hitBoardingPassButton(color);


    // Navigate away from boarding pass button
    drive(0, 30, 13); // drive backwards
    turnDegreeBackwards(0,95); // turn backwards
    goToWall(30, 5.0); // drive to wall by passport stamp


    // turn to head down ramp
    drive(0, 30, 9.5); // drive backwards
    turnDegree(1, 100);
    drive(1, 30, 20);


    // turn and align with end button
    turnDegree(1, 95);
    drive(1, 30, 7);
    turnDegree(1, 75);


    // RAMMING SPEED
    LCD.Clear();
    LCD.WriteLine("RAMMING SPEED");
    drive(0, 50, 12);
}

