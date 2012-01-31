/*-----------------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.                                    */
/* Open Source Software - may be modified and shared by FRC teams. The code          */
/* must be accompanied by the FIRST BSD license file in the root directory of        */
/* the project. Stalk Matt Reis. - Matt Roy, Chief Programmer, Class of 2011         */
/*-----------------------------------------------------------------------------------*/

package org.buzzrobotics;

//kyle
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
//import edu.wpi.first.wpilibj.Ultrasonic;



//List of Autonomous Modes, Button Layouts, and PWM ports
/**
 * Autonomous Mode:
 *  - 0 = Do Nothing
 *  - 1 = DO NOT RUN
 *  - 2 = Drive forward on straight line, raise elevator/arm, stop and score (Hasn't been run)
 *  - 3 = Line track the straight line and score (Our main automode)
 *  - 4 = Line Track the forked line, move left with lateral, and score (needs to be fully debugged)
 *  - 5 = Line Track the forked line, move right with lateral, and score (needs to be fully debugged)
 *
 * Autonomous Yaled:
 *  - 0 = No Delay
 *  - 1 = 0.5 Second Delay
 *  - 2 = 1 Second Delay
 *  - 3 = 2 Second Delay
 *  - 4 = 3 Second Delay
 *  - 5 = 4 Second Delay
 *
 * This code assumes the following connections:
 * - Driver Station:
 *  - USB 1 - Driver. Arcade Drive.    (Right Stick)
 *  - USB 2 - Elevator.                (Left Stick)
 *  - USB 3 - Autonomous Box.
 *  - USB 4 - Stop Button.
 *
 *
 * - Left Elevator Joystick Buttons:
 *   - 1) Enable Elevator Lift
 *   - 2) Top Peg                         7) To Vertical
 *   - 3) Up/Down Elevator 4"             8)
 *   - 4) Grip Tube                       9) 
 *   - 5) Release Tube                   10) Minibot Deploy
 *   - 6) To Human Player                11) Minibot Arm
 *
 *   - X-Axis) 
 *   - Y-Axis) UP/DOWN Elevator Lift
 *   - Z-Axis) Bring all the way down to disable the fluxCapacitor. You then are
 *             allowed to deploy the minibot whenever.
 *
 *
 * - Right Drive Joystick Buttons:
 *   - 1) Enable Lateral Drive
 *   - 2) Auto Load Floor Tube            7) Floor pickup arm to load
 *   - 3) Close floor arm                 8)
 *   - 4) Floor pickup arm to floor       9)
 *   - 5) Floor pickup arm to stow       10) Reset the Encoders
 *   - 6) Open floor arm                 11) Print Data to Laptop
 *
 *   - X-Axis) Drive Left/Right/Lateral
 *   - Y-Axis) Drive Forward/Backwards
 *
 *
 * - Robot Electronics Layout:
 *   - PWM Ports on Digital Sidecar, Module 4 -
 *     PWM Out:
 *       - PWM 1 + 3 - Right Drive Motors (Jag)          - PWM 7 - Lateral Drive (Victor)
 *       - PWM 2 + 4 - Left Drive Motors (Jag)           - PWM 8 -
 *       - PWM 5     - Elevator Arm (Jag)                - PWM 9 -
 *       - PWM 6     - Floor Tube Arm (Jag)             - PWM 10 -
 *
 *     Digital I/O:
 *       - PWM 1 - Pressure Switch                       - PWM 8 - 
 *       - PWM 2 - Left Drive Encoder Source A           - PWM 9 - 
 *       - PWM 3 - Left Drive Encoder Source B          - PWM 10 - Peg Line Sensor 1 (Retroreflective)
 *       - PWM 4 - Right Drive Encoder Source A         - PWM 11 - Left Floor Line Sensor 2
 *       - PWM 5 - Right Drive Encoder Source B         - PWM 12 - Middle Floor Line Sensor 3
 *       - PWM 6 - Lateral Drive Encoder Source A       - PWM 13 - Right Floor Line Sensor 4
 *       - PWM 7 - Lateral Drive Encoder Source B       - PWM 14 -
 *
 *     Relay Ports:
 *       - PWM 1 -                                       - PWM 5 -
 *       - PWM 2 -                                       - PWM 6 -
 *       - PWM 3 -                                       - PWM 7 -
 *       - PWM 4 -                                       - PWM 8 - Compressor (Spike)
 *
 *
 * - PWM Ports on Digital Sidecar, Slot 4 -
 *   - Solenoid Relay Ports, Module 7:
 *     - PWM 1     - Lateral Drive (Single Solenoid)
 *     - PWM 2     - Minibot Deploy/Retract (Single Solenoid)
 *     - PWM 3 + 4 - Minibot Arm Deploy/Retract 1st (Double Solenoid)
 *     - PWM 5 + 6 - Floor Grabber (Double Solenoid)
 *     - PWM 7 + 8 - 
 *     
 *   - Solenoid Relay Ports, Module 8:
 *     - PWM 1 + 2 - Tube Grabber (Double Solenoid)
 *     - PWM 3     -
 *     - PWM 4 + 5 - 8" Elevator Deploy/Retract (Double Solenoid)
 *     - PWM 6 + 7 - Main Elevator Deploy/Retract (Double Solenoid)
 *     - PWM 8     -
 * 
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class DefaultRobot extends IterativeRobot {

/********************************** Global Variables *************************************/
//Global Variables - can be axcessed anywhere in the code

           //Joystick
                final static int driverJoystick = 1;        //Right Stick
                final static int elevatorJoystick = 2;      //Left Stick

           //RIGHT Joystick Buttons
                final static int rightButton1 = 1;
                final static int rightButton2 = 2;
                final static int rightButton3 = 3;
                final static int rightButton4 = 4;
                final static int rightButton5 = 5;
                final static int rightButton6 = 6;
                final static int rightButton7 = 7;
                final static int rightButton8 = 8;
                final static int rightButton9 = 9;
                final static int rightButton10 = 10;
                final static int rightButton11 = 11;
                static double rightAxisZ; //used to read the nob on the bottom of the joystick

           //LEFT Joystick Buttons
                final static int leftButton1 = 1;
                final static int leftButton2 = 2;
                final static int leftButton3 = 3;
                final static int leftButton4 = 4;
                final static int leftButton5 = 5;
                final static int leftButton6 = 6;
                final static int leftButton7 = 7;
                final static int leftButton8 = 8;
                final static int leftButton9 = 9;
                final static int leftButton10 = 10;
                final static int leftButton11 = 11;
                static double leftAxisZ; //used to read the nob on the bottom of the joystick

           //PWM Ports (Module 4 Digital Sidecar)
                final static int jaguarElevatorArmPWM = 5;     //the elevator arm that runs off a motor (to vertical)
                final static int jaguarFloorArmPWM = 6;        //floor arm that picks up tubes from floor
                final static int victorLateralDrivePWM = 7;    //lateral drive

           //RELAY PWM Ports (Module 4 Digital Sidecar)                
                final static int compressorPWM = 8;       //Compressor PWM Port
                
           //DIGITAL I/O (Module 4 Digital Sidecar)
                final static int pressureSwitchPWM = 1;      //Used to tell when to stop the compressor
//                final static int ultrasonicSensorPing = 9;
//                final static int ultrasonicSensorEcho = 8;
                final static int lineSensor1 = 10;           //If we found the peg all lights on
                final static int lineSensor2 = 11;           //left line sensor
                final static int lineSensor3 = 12;           //middle line sensor
                final static int lineSensor4 = 13;           //right line sensor
                //Encoders
                    final static int encoderLeftDriveSourceA = 2;
                    final static int encoderLeftDriveSourceB = 3;
                    final static int encoderRightDriveSourceA = 4;
                    final static int encoderRightDriveSourceB = 5;
                    final static int encoderLateralDriveSourceA = 6;
                    final static int encoderLateralDriveSourceB = 7;
                    static int encoderCountsLeft;                       //Left Drive Encoder Counts
                    static int encoderCountsRight;                      //Right Drive Encoder Counts
                    static int encoderCountsLateralDrive;               //Lateral Drive Encoder Counts
                                           
           //SOLENOID PORTS (Module 7)
                final static int solenoidLateralPort = 1;                       //Lateral
                final static int solenoidMiniBotDeployPort = 2;                 //Minibot Deploy
                final static int solenoidMiniBotArmFirstSolenoidDeployPort = 3; //Minibot Arm
                final static int solenoidMiniBotArmFirstSolenoidRetractPort = 4;
                final static int solenoidFloorGrabberDeployPort = 5;            //Floor Grabber
                final static int solenoidFloorGrabberRetractPort = 6;
//                final static int solenoidMiniBotArmSecondSolenoidDeployPort = 5;
//                final static int solenoidMiniBotArmSecondSolenoidRetractPort = 6;
                
           //SOLENOID PORTS (Module 8)
                final static int solenoidTubeGrabberDeployPort = 1;             //Tube Grabber
                final static int solenoidTubeGrabberRetractPort = 2;
                final static int solenoidElevatorUpDown4DeployPort = 4;         //4" Elevator
                final static int solenoidElevatorUpDown4RetractPort = 5;
                final static int solenoidMainElevatorDeployPort = 6;            //Main Elevator
                final static int solenoidMainElevatorRetractPort = 7;
                
           //AnalogChannel
                final static int analogPotElevatorArmChannel = 1;       //pot on the elevator arm
                final static int analogPotFloorArmChannel = 2;          //pot on the floor arm
                
           //Watchdog Expiration
                final static double watchdogExpiration = 5;             //5 seconds

          //Teleop Delay
                double lateralDelay = 0.1;

           //Autonomous Mode
                double autoMode = 0;             //which autonomous mode we are in
                double autoCase = 0;             //which autonomous "case" we are in(currently used for mode 1)
                double autoYaled = 0;            //we can use the "yaled" switch to "delay" the start of any autoMode
                double Yaled = 0;                //the time delay set by autoYaled's reading
                double autoDelay = 0;            //used to set the amount of time (in seconds) that we can wait (first delay time)
                double autoDelay2 = 0;           //how long we will wait (second delay time)
                double autoDelay3 = 0;           //how long we will wait (third delay time)
                double autoForwardDrivingSpeed = 0.65;  //the speed we are driving forward in autonomous                
                double autoTurnLeftSpeed = -0.3;         //left speed in autonomous                
                double autoTurnRightSpeed = 0.3;       //right speed in autonomous
                double fluxCapacitor = 0;               //Our field time (sec)
                double fluxCapacitorZero = 0;           //Our start time (when teleop starts)
                double autoModeState = 0;
                double autoCounter = 0;
                double autoCounter2 = 0;

           //Pot Limits
                //main
                final static double potElevatorArmUpperLimit = 1.3;        //Pot Elevator Arm Upper Limit   (Vertical and ready to score)
                final static double potElevatorArmHumanLimitUpper = 3.58;  //Pot Elevator Arm human player upper Limit (when moveing down)
                final static double potElevatorArmHumanLimitLower = 3.66;  //Pot Elevator Arm human player lower Limit (when moveing up)
                final static double potElevatorArmLowerLimit = 3.88;       //Pot Elevator Arm Lower Limit (Not all the way down to the frame)
                //floor
                final static double potFloorArmUpperLimit = 5;             //Pot Floor Arm Upper Limit (stow poss)
                final static double potFloorArmLoadLimitUpper = 4.29;      //Pot Floor Arm load upper Limit
                final static double potFloorArmLoadLimitLower = 4.16;      //4.78;     //Pot Floor Arm load lower Limit
                final static double potFloorArmOutOfTheWayLimit = 4;       //4.29;     //Pot floor arm avoid the elevator arm
                final static double potFloorArmLowerLimit = 1.38;          //Pot Floor Arm Lower Limit (to floor)
                double elevatorLift;   //Takes voltage from pot and sets it as "elevatorLift"
                double floorLift;


/********************************** Integers/Booleans (Local Variables) *******************/
//Local Variables - INT/BOOLEAN (Some used to count the number of periodic loops performed)


           //INTEGERS
                int m_autoPeriodicLoops;
                int m_disabledPeriodicLoops;
                int m_telePeriodicLoops;

           //DOUBLE
                //Ultrasonic Sensor
//                double maxRange;
//                double minRange;

           //BOOLEANS
                boolean m_solenoidMainElevatorTopPegLatch = false;
                boolean m_previousStateTopPeg = false;
                boolean m_solenoidElevatorUpDown8Latch = false;
                boolean m_previousStateElevatorUpDown8 = false;
                boolean m_solenoidMiniBotArmLatch = false;
                boolean m_previousStateMinibotArm = false;                                                
                //we haven't found any pegs to start off with
                boolean m_lineSensor1PegFound = true;      
                boolean m_lineSensor2LeftDrive = true;
                boolean m_lineSensor3MiddleDrive = true;
                boolean m_lineSensor4RightDrive = true;
                boolean m_lateralFoundPeg = false;
                boolean m_weHitTheT = false;
                boolean m_readyToLateral = false;
                boolean backingUp = false;   //we are not backing up :P
                boolean endOfAutoMode = false;
                //we haven't pressed any buttons yet
                boolean leftButton1Pressed = false;
                boolean leftButton6Pressed = false;     //we haven't pressed button yet
                boolean leftButton7Pressed = false;                                                
                boolean rightButton2Pressed = false;                               
                //if we have a floor tube we can use button 3
                boolean m_doWeHaveAFloorTubeToLoad = false;

                
                
/********************************** Declare Variables *************************************/
//Declare Variables

           //Drive
                RobotDrive robotDriveTrain;     

           //Camera (Axis is still alive!!!)
                AxisCamera robotCamera = AxisCamera.getInstance();      

           //Compressors
                Compressor robotCompressor;

           //Joysticks
                Joystick rightStick;    //Driver Joystick
                Joystick leftStick;     //Elevator) Joystick
                
           //Jaguars
                Jaguar robotJaguarFloorArm;
                Jaguar robotJaguarElevatorArm;

           //Victors
                Victor roboVictorLateralDrive;
                
           //Encoders
                Encoder robotEncoderLeftDrive;
                Encoder robotEncoderRightDrive;
                Encoder robotEncoderLateralDrive;

           //Line Trackers (On the D I/O module)
                DigitalInput robotDigitalInPegFound;     //Digital input for peg found?
                DigitalInput robotDigitalInLeft;         //Digital input for Left tracking sensor
                DigitalInput robotDigitalInMiddle;       //Digital input for middle tracking sensor
                DigitalInput robotDigitalInRight;        //Digital input for right tracking sensor
                    
           //Solenoids
                //(Module 7)
                    Solenoid robotSolenoidLateralDrive;                 //lateral drive
                    Solenoid robotSolenoidMinibotDeploy;                //minibot deploy
                    DoubleSolenoid robotDoubleSolenoidMinibotArmSolenoidOne;  //minibot arm
                //(Module 8)
                    DoubleSolenoid robotDoubleSolenoidMainElevator;     //main elevator
                    DoubleSolenoid robotDoubleSolenoidElevatorUpDown4;  //4" elevator
                    DoubleSolenoid robotDoubleSolenoidTubeGrabber;      //tube grabber
                    DoubleSolenoid robotDoubleSolenoidFloorGrabber;     //floor grabber
                
           //Analog Channels
                AnalogChannel robotPotElevatorArm;   //pot used to make sure we are at level to obtain a tube and have a fully vertical elevator
                AnalogChannel robotPotFloorArm;      //pot used to make sure we don't overextend the floor arm

           //Ultrasonic
//                Ultrasonic robotUltrasonicSensor;

           //Driver Station (used to access the driver station object(DS update stuff))
                DriverStation m_ds_Buzz;
                int m_dsPacketsReceivedInCurrentSecond;
    

    static final int NUM_JOYSTICK_BUTTONS = 16;
    boolean[] m_rightStickButtonState = new boolean[(NUM_JOYSTICK_BUTTONS+1)];
    boolean[] m_leftStickButtonState = new boolean[(NUM_JOYSTICK_BUTTONS+1)];


    
/********************************** DefaultRobot *************************************/
    public DefaultRobot() {
        System.out.println("BuiltinDefaultCode Constructor Started\n");
       
            //Arcade Drive
                robotDriveTrain = new RobotDrive(1, 2); //PWM 1,2 for left and right drive

            //Compressor
                robotCompressor = new Compressor(pressureSwitchPWM, compressorPWM);  
                
            //Joystick
                rightStick = new Joystick(driverJoystick);     //USB 1
                leftStick = new Joystick(elevatorJoystick);    //USB 2
            
            //Jaguar
                robotJaguarFloorArm = new Jaguar(4, jaguarFloorArmPWM);
                robotJaguarElevatorArm = new Jaguar(4, jaguarElevatorArmPWM);

            //Victor
                roboVictorLateralDrive = new Victor(4, victorLateralDrivePWM);
                
            //Encoders
                robotEncoderLeftDrive = new Encoder(4, encoderLeftDriveSourceA, 4, encoderLeftDriveSourceB);
                robotEncoderRightDrive = new Encoder(4, encoderRightDriveSourceA, 4, encoderRightDriveSourceB);
                robotEncoderLateralDrive = new Encoder(4, encoderLateralDriveSourceA, 4, encoderLateralDriveSourceB);
    
            //D I/O
                //Ultrasonic Sensor
//                    robotUltrasonicSensor = new Ultrasonic(4, ultrasonicSensorPing, 4, ultrasonicSensorEcho);   //slot 4, D I/O 8 for ping/slot 9 for echo
                //Digital Inputs LINE TRACKER
                    robotDigitalInPegFound = new DigitalInput(lineSensor1); //found peg to score on
                    robotDigitalInLeft = new DigitalInput(lineSensor2);    //left line track
                    robotDigitalInMiddle = new DigitalInput(lineSensor3); //middle line track
                    robotDigitalInRight = new DigitalInput(lineSensor4);  //right line track

            //Double Solenoids
                 //(Module 8)
                    robotDoubleSolenoidMainElevator = new DoubleSolenoid(8, solenoidMainElevatorDeployPort, solenoidMainElevatorRetractPort);
                    robotDoubleSolenoidElevatorUpDown4 = new DoubleSolenoid(8, solenoidElevatorUpDown4DeployPort, solenoidElevatorUpDown4RetractPort);
                    robotDoubleSolenoidTubeGrabber = new DoubleSolenoid(8, solenoidTubeGrabberDeployPort, solenoidTubeGrabberRetractPort);
                 //(Module 7)
                    robotSolenoidLateralDrive = new Solenoid(7, solenoidLateralPort);   //single acting
                    robotSolenoidMinibotDeploy = new Solenoid(7, solenoidMiniBotDeployPort);
                    robotDoubleSolenoidMinibotArmSolenoidOne = new DoubleSolenoid(7, solenoidMiniBotArmFirstSolenoidDeployPort, solenoidMiniBotArmFirstSolenoidRetractPort);
                    robotDoubleSolenoidFloorGrabber = new DoubleSolenoid(7, solenoidFloorGrabberDeployPort, solenoidFloorGrabberRetractPort);
//                    robotDoubleSolenoidTubeGrabber = new DoubleSolenoid(7,7,8); //bread board code ONLY
                    
            //Analogs --- (Slot 1 on cRIO)
                robotPotElevatorArm = new AnalogChannel(1, analogPotElevatorArmChannel);
                robotPotFloorArm = new AnalogChannel(1, analogPotFloorArmChannel);           
                //Holly is awesome -Matt
            //Driver Station
                m_dsPacketsReceivedInCurrentSecond = 0;
                //Kelsie is awesome -Matt   Matt is awesome -Kelsie
                m_ds_Buzz = DriverStation.getInstance();

                
            // Iterate over all the buttons on each joystick, setting state to false for each
                int buttonNum = 1;			// start counting buttons at button 1
                    for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
                            m_rightStickButtonState[buttonNum] = false;
                            m_leftStickButtonState[buttonNum] = false;
                }

            // Initialize counters to record the number of loops completed in autonomous and teleop modes
                m_autoPeriodicLoops = 0;
                m_disabledPeriodicLoops = 0;
                m_telePeriodicLoops = 0;

            //Print out to the computer "BuiltinDefaltCode Constructor Completed/n"
                System.out.println("DefaultRobot Constructor Completed\n");                
	}


    
/********************************** Init Routines *************************************/
// Actions which would be performed once (and only once) upon initialization of the
// robot would be put here.

    public void robotInit() {

                System.out.println("RobotInit() completed.\n");
                Timer.delay(7.0);        //Should this be shortened???
            //Start the Watchdog
                Watchdog.getInstance().setExpiration(watchdogExpiration);
                Watchdog.getInstance().feed();
            //Start the Compressor
                robotCompressor.start();
            //Start the Camera
                robotCamera = AxisCamera.getInstance();
                robotCamera.writeResolution(AxisCamera.ResolutionT.k320x240);   //The Resolution of the camera(640x480) is the highest, 160x120 is the lowest)
                robotCamera.writeBrightness(0);
                robotCamera.writeCompression(80);           

    }

    
    public void disabledInit() {
                m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
                Watchdog.getInstance().feed();
    }


    public void autonomousInit() {  
                m_autoPeriodicLoops = 0;	// Reset the loop counter for autonomous mode
                Watchdog.getInstance().feed();

            //Read the analog box voltages, determine which autonomous mode to use,
            //and figure out what our Yaled is
                autonomusBoxReadingMode();   //find what automMode we are in                
                autonomusBoxReadingYaled();  //find Yaled AKA the delay

            //Start Autonomous with the drive stopped
                robotDriveTrain.arcadeDrive(0, 0);

            //Start Autonomous with all Jaguars and Victors stoped
                robotJaguarElevatorArm.set(0);
                robotJaguarFloorArm.set(0);
                roboVictorLateralDrive.set(0);

            //Reset all encoders and then start the encoders
                robotEncoderLeftDrive.start();                        
                robotEncoderLeftDrive.reset();                        
                robotEncoderRightDrive.start();                       
                robotEncoderRightDrive.reset();                       
                robotEncoderLateralDrive.start();                     
                robotEncoderLateralDrive.reset();                                

            //Start Autonomous with the main elevator down, the 4" elevator not extended,
            //the tube grabber not grabbing, the lateral up, the minibot arm and deploy
            //not out/deployed, and the floor grabber grabbing. All are solenoids
                robotDoubleSolenoidMainElevator.set(DoubleSolenoid.Value.kReverse);     //main elevator
                robotDoubleSolenoidElevatorUpDown4.set(DoubleSolenoid.Value.kReverse);  //4" elevator
                robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kForward);      //tube grabber(grip tube
                robotDoubleSolenoidFloorGrabber.set(DoubleSolenoid.Value.kReverse);     //floor grabber
                robotSolenoidLateralDrive.set(false);                                   //lateral drive
                robotSolenoidMinibotDeploy.set(false);   //minibot deploy
                robotDoubleSolenoidMinibotArmSolenoidOne.set(DoubleSolenoid.Value.kReverse); //minibot arm

            //We haven't found the peg yet
                m_lateralFoundPeg = false;   
                
    }


    public void teleopInit() {
                m_autoPeriodicLoops = 0;	// Reset the loop counter for autonomous mode
                Watchdog.getInstance().feed();

            //Stop autonomous from running in teleop
                autoMode = 0;
                Yaled = 0; //set the delay to 0 just to be safe
                autoCase = 0;
                m_lateralFoundPeg = false;
                backingUp = false;
                fluxCapacitorZero = Timer.getFPGATimestamp();

            //Start Teleop with the drive stopped
                robotDriveTrain.arcadeDrive(0, 0);

            //Start Teleop with all Jaguars and Victors stoped
                robotJaguarElevatorArm.set(0);
                robotJaguarFloorArm.set(0);
                roboVictorLateralDrive.set(0);

            //Reset all encoders and then start the encoders
                robotEncoderLeftDrive.start();
                robotEncoderLeftDrive.reset();
                robotEncoderRightDrive.start();
                robotEncoderRightDrive.reset();
                robotEncoderLateralDrive.start();
                robotEncoderLateralDrive.reset();
            
            //Start Teleop with the main elevator down, the 4" elevator not extended,
            //the tube grabber not grabbing, the lateral up, the minibot arm and deploy
            //not out/deployed, and the floor grabber grabbing. All are solenoids
                robotDoubleSolenoidMainElevator.set(DoubleSolenoid.Value.kReverse);     //main elevator
                robotDoubleSolenoidElevatorUpDown4.set(DoubleSolenoid.Value.kReverse);  //4" elevator
                robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kReverse);      //tube grabber
                robotDoubleSolenoidFloorGrabber.set(DoubleSolenoid.Value.kReverse);     //floor grabber(release)
                robotSolenoidLateralDrive.set(false);                                   //lateral drive
                robotSolenoidMinibotDeploy.set(false);   //minibot deploy
                robotDoubleSolenoidMinibotArmSolenoidOne.set(DoubleSolenoid.Value.kReverse); //minibot arm

                //Read the ultrasonic sensor after starting it from pinging and enableing it
//                    robotUltrasonicSensor.setAutomaticMode(true);
//                    robotUltrasonicSensor.setEnabled(true); //enable                
    }


    
/********************************** Periodic Routines *************************************/
static int printSec = (int)((Timer.getUsClock() / 1000000.0) + 1.0);    //Just getting the time...in seconds
static final int startSec = (int)(Timer.getUsClock() / 1000000.0);

    public void disabledPeriodic()  {
           Watchdog.getInstance().feed();

            //Increment the number of disabled periodic loops completed.
            m_disabledPeriodicLoops++;  //???

            // While disabled, printout the duration of current disabled mode in seconds on this computer.
            if ((Timer.getUsClock() / 1000000.0) > printSec) {
                System.out.println("Disabled seconds: " + (printSec - startSec) + "\r\n");
                printSec++;
            }
    }



/********************************** Autonomous Periodic *************************************/
//Autonomous Code
//BEGIN AUTONOMOUS MODE!!! Use the analog volatage number select the autonomous mode.

    public void autonomousPeriodic() {

        // feed the user watchdog at every period when in autonomous
        Watchdog.getInstance().feed();
        m_autoPeriodicLoops++;
        //If autoMode is 0 do nothing
        if (autoMode == 0) return;             

        //SELECT OUR AUTONOMOUS mode baised on the result from belows method
        switch ((int)autoMode) {

             //Automonous mode 1:  Follow the forked line, lateral left, move to score, move backwards
             case 2://1
                 //delay the start of this mode by what ever "yaled" analog is set to
                 Timer.delay(Yaled);
                 Yaled = 0; //stop delaying everytime

                 robotDriveTrain.arcadeDrive(-0.6,1);
                 break; //end mode 1


                    
             //Automonous mode 2:  Drive forward (straight line), raise elevator/arm, stop and score
             case 3://2
                    //delay the start of this mode by what ever "yaled" analog is set to
                    Timer.delay(Yaled);
                    Yaled = 0; //stop delaying everytime

                    autonomousElevatorTopTop(); //extend elevator and move arm up to HIGHEST PEG
                    encoderCountsLeft = robotEncoderLeftDrive.getRaw(); //get the encoder counts for the left drive
//                    //Print data to screen
//                        System.out.println("Left Digital Input: " + robotDigitalInLeft.get());                  //Print Left Digital Input
//                        System.out.println("Middle Digital Input: " + robotDigitalInMiddle.get());              //Print Middle Digital Input
//                        System.out.println("Right Digital Input: " + robotDigitalInRight.get());                //Print Right Digital Input
//                        System.out.println("Left Drive Encoder Value: " + encoderCountsLeft);                 //Print Left Drive Value
//                          System.out.println("Right Drive Encoder Value: " + encoderRightDrive.getRaw());     //Print Right Drive Value
//                          System.out.println("Lateral Drive Encoder Value: " + encoderLateralDrive.getRaw()); //Print Lateral Drive Value

                    //if our left or right encoder counts are more than 4500 then stop the drive
                         if (((encoderCountsLeft) >= 4500)) {
                             robotDriveTrain.arcadeDrive(0, 0);               //stop drive
                             Timer.delay(autoDelay); //let us come to a stop
                             robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kReverse); //let go of the tube
                             Timer.delay(autoDelay); //let us drop the tube
                             autoDelay = 0; //don't delay again

                                if(encoderCountsLeft <= 0) { //we've hit where we started so stop
                                    robotDriveTrain.arcadeDrive(0,0);
                                } else {
                                    robotDriveTrain.arcadeDrive(0.75,0); //drive backwards
                                }
                             
                         } else { //drive forward, bring up are elevator arm/extend it, and grip the tube
                             robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kForward); //grip the tube
                             robotDriveTrain.arcadeDrive(-0.75, 0);  //drive 75% forward without any turn
                         }
             break;  //stop automonous mode 2

             
             ////////////////////////////////////////////////////////////
             //Automonous mode 3:  Line track the straight line and score
             case 4://3
                    //delay the start of this mode by what ever "yaled" analog is set to
                    Timer.delay(Yaled);
                    Yaled = 0; //stop delaying everytime

                    autonomousElevatorTopTop(); //extend elevator and move arm up to HIGHEST PEG
                    autonomousPrintOut();  //print data to the laptop
                    //use encoder to stop following the line instead of using time to stop
                    //us from following the line
                    encoderCountsRight = robotEncoderRightDrive.getRaw(); //get the encoder counts for the left drive
                    m_lineSensor2LeftDrive = robotDigitalInLeft.get();
                    m_lineSensor3MiddleDrive = robotDigitalInMiddle.get();
                    m_lineSensor4RightDrive = robotDigitalInRight.get();                    
                
                    //stop when we hit 4500 encoder counts(we are against the wall). get ready to score
                    if (((encoderCountsRight <= -10000) || (m_weHitTheT))) {
                          m_weHitTheT = true;
                        //if we are now ready to back up start to back up
                        if (backingUp) {
                            robotDriveTrain.arcadeDrive(-0.52,0);       //drive backwards to our start poss(original before 41111 change -0.55)
                        }
                        //if we are not backing up stop drive, and score, ect...
                        else  {
                            robotDriveTrain.arcadeDrive(0,0); //stop the drive before we let go of the tube
                            Timer.delay(autoDelay); //let the robot stop
                            robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kReverse);  //release the tuberobotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kForward); //let go of the tube
                            Timer.delay(autoDelay); //delay for a short time
                            autoDelay = 0;  //now that we have delayed set the delay to 0 that way we won't delay again
                            backingUp = true;   //we now have started backing up
                        }

                        
//                        if((backingUp) && (encoderCountsRight >= 0)) { //we've returned to our original spot
//                            robotDriveTrain.arcadeDrive(0,0);          //stop drive
//                        } else {   //drive backwards to our original spot
//                            robotDriveTrain.arcadeDrive(-0.55,0);       //drive backwards to our start poss
//
////                            autonomousBringElevatorArmBackDown();  //bring the elevator/arm back down
//                        }

                        
                    } else {  //we are tracking the line
                        robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kForward);  //grab the tube


                        //false says we found the line
                            //*.. = left turn
                            if (!m_lineSensor2LeftDrive && m_lineSensor3MiddleDrive && m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, autoTurnLeftSpeed); //hard turn left
                            }
                            //.*. = drive forward
                            else if (m_lineSensor2LeftDrive && !m_lineSensor3MiddleDrive && m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, 0);                                
                            }
                            //*** stop
                            else if (!m_lineSensor2LeftDrive && !m_lineSensor3MiddleDrive && !m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(0, 0); //stop  
                                m_weHitTheT = true;
                            }
                            //..* = right turn
                            else if (m_lineSensor2LeftDrive && m_lineSensor3MiddleDrive && !m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, autoTurnRightSpeed); //hard turn right
                            }
                            //we are lost...
                            else {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, 0); //just move forward in shame
                            }

                          }
             break;  //stop automonous mode 3

             
             //////////////////////////////////////////////////////////////////////////////////
             //Automonous mode 4: Line Track the forked line, move left with lateral, and score
             case 6://4
                    //delay the start of this mode by what ever "yaled" analog is set to
                    Timer.delay(Yaled);
                    Yaled = 0; //stop delaying everytime

                    autonomousElevatorTop(); //extend elevator and move arm up to TOP PEG
                    autonomousPrintOut();  //print data to the laptop
                    
                    //use encoder to stop following the line
                    encoderCountsRight = robotEncoderRightDrive.getRaw(); //get the encoder counts for the left drive
                    encoderCountsLateralDrive = robotEncoderLateralDrive.getRaw(); //get the encoder counts for the left drive
                    m_lineSensor1PegFound = robotDigitalInPegFound.get(); //have we found the peg or not? (boolean)
                    m_lineSensor2LeftDrive = robotDigitalInLeft.get();
                    m_lineSensor3MiddleDrive = robotDigitalInMiddle.get();
                    m_lineSensor4RightDrive = robotDigitalInRight.get();                    

                    //stop when we hit the Y or hit -6000 encoder counts
                    if ((encoderCountsRight <= -6000) || (m_readyToLateral)) {
                        m_readyToLateral = true;
//                        encoderLateralDrive.reset(); //reset the lateral Drive encoder

                        if ((encoderCountsLateralDrive <= -1200) || (!m_lineSensor1PegFound) || (m_lateralFoundPeg)) { //if lined up with the peg stop the lateral (encoders)
                            m_lateralFoundPeg = true; //We found the peg, so even if we lose it, don't go back to the if statement
                            roboVictorLateralDrive.set(0); //stop the lateral drive
                            Timer.delay(autoDelay2); //let the lateral drive stop driving
                            robotSolenoidLateralDrive.set(false);  //bring up the lateral drive
                            Timer.delay(autoDelay2); //let the lateral come up
                            autoDelay2 = 0; //set to 0 so it doesn't keep delaying
//                            encoderLeftDrive.reset(); //reset the left drive encoder

                            if ((encoderCountsRight <= -12000) || (!m_lineSensor2LeftDrive && !m_lineSensor3MiddleDrive && !m_lineSensor4RightDrive) || (endOfAutoMode)) { //stop when we hit the wall or we hit the T
                                endOfAutoMode = true;  //keep us in this section of the if/else statements

                                if (backingUp) {
                                    robotDriveTrain.arcadeDrive(-0.55,0);       //drive backwards to our start poss
                                } else {   //drive backwards to our original spot                                    
                                    robotDriveTrain.arcadeDrive(0, 0); //stop drive
                                    Timer.delay(autoDelay3); //let the drive come to a stop
                                    robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kReverse);  //release the tube
                                    Timer.delay(autoDelay3); //let the tube become released
                                    autoDelay3 = 0;  //set to 0 that way we won't delay again
                                    backingUp = true;   //we now have started backing up
        //                            autonomousBringElevatorArmBackDown();  //bring the elevator/arm back down
                                }


                            } else { //drive slowly until we hit the wall
                                robotDriveTrain.arcadeDrive(0.65, 0); //move forward to the wall
                            }


                            
                        } else { //we've hit the Y so stop drive, deploy lateral, and drive left
                            robotDriveTrain.arcadeDrive(0,0);   //stop the drive
                            Timer.delay(autoDelay); //Delay to put down the lateral
                            robotSolenoidLateralDrive.set(true);                         //drop the lateral drive
                            Timer.delay(autoDelay); //Delay before moving lateral
                            autoDelay = 0;   //Set to 0 so it doesnt keep delaying
                            roboVictorLateralDrive.set(1); //begin to move the lateral to the left until we hit our proper encoder counts
                        }


                    } else {  //we are tracking the line
                        robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kForward);  //grab the tube


                        //false says we found the line
                            //*.. = left turn
                            if (!m_lineSensor2LeftDrive && m_lineSensor3MiddleDrive && m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, autoTurnLeftSpeed); //hard turn left
                            }
                            //.*. = drive forward
                            else if (m_lineSensor2LeftDrive && !m_lineSensor3MiddleDrive && m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, 0);
                            }
                            //*** stop
                            else if (!m_lineSensor2LeftDrive && !m_lineSensor3MiddleDrive && !m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(0, 0); //stop                                
                            }
                            //..* = right turn
                            else if (m_lineSensor2LeftDrive && m_lineSensor3MiddleDrive && !m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, autoTurnRightSpeed); //hard turn right
                            }
                            //we are lost...
                            else {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, 0); //just move forward in shame
                            }
                    }
             break;      //stop automonous mode 4
             
             
             ///////////////////////////////////////////////////////////////////////////////////
             //Automonous mode 5: Line Track the forked line, move right with lateral, and score
             case 7://5
                    //delay the start of this mode by what ever "yaled" analog is set to
                    Timer.delay(Yaled);
                    Yaled = 0; //stop delaying everytime

                    autonomousElevatorTop(); //extend elevator and move arm up to TOP PEG
                    autonomousPrintOut();  //print data to the laptop

                    //use encoder to stop following the line
                    encoderCountsRight = robotEncoderRightDrive.getRaw(); //get the encoder counts for the left drive
                    encoderCountsLateralDrive = robotEncoderLateralDrive.getRaw(); //get the encoder counts for the left drive
                    m_lineSensor1PegFound = robotDigitalInPegFound.get(); //have we found the peg or not? (boolean)
                    m_lineSensor2LeftDrive = robotDigitalInLeft.get();
                    m_lineSensor3MiddleDrive = robotDigitalInMiddle.get();
                    m_lineSensor4RightDrive = robotDigitalInRight.get();

                    //stop when we hit the Y or hit -6000 encoder counts
                    if ((encoderCountsRight <= -6000) || (m_readyToLateral)) {
                        m_readyToLateral = true;
//                        encoderLateralDrive.reset(); //reset the lateral Drive encoder

                        if ((encoderCountsLateralDrive >= 1200) || (!m_lineSensor1PegFound) || (m_lateralFoundPeg)) { //if lined up with the peg stop the lateral (encoders)
                            m_lateralFoundPeg = true; //We found the peg, so even if we lose it, don't go back to the if statement
                            roboVictorLateralDrive.set(0); //stop the lateral drive
                            Timer.delay(autoDelay2); //let the lateral drive stop driving
                            robotSolenoidLateralDrive.set(false);  //bring up the lateral drive
                            Timer.delay(autoDelay2); //let the lateral come up
                            autoDelay2 = 0; //set to 0 so it doesn't keep delaying
//                            encoderLeftDrive.reset(); //reset the left drive encoder

                            if ((encoderCountsRight <= -12000) || (!m_lineSensor2LeftDrive && !m_lineSensor3MiddleDrive && !m_lineSensor4RightDrive) || (endOfAutoMode)) { //stop when we hit the wall or we hit the T
                                endOfAutoMode = true;  //keep us in this section of the if/else statements

                                if (backingUp) {
                                    robotDriveTrain.arcadeDrive(-0.55,0);       //drive backwards to our start poss
                                } else {   //drive backwards to our original spot
                                    robotDriveTrain.arcadeDrive(0, 0); //stop drive
                                    Timer.delay(autoDelay3); //let the drive come to a stop
                                    robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kReverse);  //release the tube
                                    Timer.delay(autoDelay3); //let the tube become released
                                    autoDelay3 = 0;  //set to 0 that way we won't delay again
                                    backingUp = true;   //we now have started backing up
        //                            autonomousBringElevatorArmBackDown();  //bring the elevator/arm back down
                                }


                            } else { //drive slowly until we hit the wall
                                robotDriveTrain.arcadeDrive(0.65, 0); //move forward to the wall
                            }



                        } else { //we've hit the Y so stop drive, deploy lateral, and drive right
                            robotDriveTrain.arcadeDrive(0,0);   //stop the drive
                            Timer.delay(autoDelay); //Delay to put down the lateral
                            robotSolenoidLateralDrive.set(true);                         //drop the lateral drive
                            Timer.delay(autoDelay); //Delay before moving lateral
                            autoDelay = 0;   //Set to 0 so it doesnt keep delaying
                            roboVictorLateralDrive.set(-1); //begin to move the lateral to the right until we hit our proper encoder counts
                        }


                    } else {  //we are tracking the line
                        robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kForward);  //grab the tube


                        //false says we found the line
                            //*.. = left turn
                            if (!m_lineSensor2LeftDrive && m_lineSensor3MiddleDrive && m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, autoTurnLeftSpeed); //hard turn left
                            }
                            //.*. = drive forward
                            else if (m_lineSensor2LeftDrive && !m_lineSensor3MiddleDrive && m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, 0);
                            }
                            //*** stop
                            else if (!m_lineSensor2LeftDrive && !m_lineSensor3MiddleDrive && !m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(0, 0); //stop                                
                            }
                            //..* = right turn
                            else if (m_lineSensor2LeftDrive && m_lineSensor3MiddleDrive && !m_lineSensor4RightDrive) {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, autoTurnRightSpeed); //hard turn right
                            }
                            //we are lost...
                            else {
                                robotDriveTrain.arcadeDrive(autoForwardDrivingSpeed, 0); //just move forward in shame
                            }
                    }
             break; //stop autonomous mode 5
        
    } //end of auto mode
    }


    
/********************************** Teleop Periodic *************************************/
//Teleop Periodic

    public void teleopPeriodic() {
                // feed the user watchdog at every period when in autonomous
                Watchdog.getInstance().feed();
                // increment the number of teleop periodic loops completed
                m_telePeriodicLoops++;
                m_dsPacketsReceivedInCurrentSecond++;		// increment DS packets received
                //Stop autonomous from running in teleop
                autoMode = 0;
                //Call the camera method  to begin running the camera code
                    camera(7);

                //Get our field time so we dont deploy minibot early
                    fluxCapacitor = Timer.getFPGATimestamp() - fluxCapacitorZero; //keep track of time so we know when to deploy the minibot

//                    System.out.println("Flux Capacitor " + fluxCapacitor);

//                    double range1 = robotUltrasonicSensor.getRangeInches(); //get the distance in inches
                    
//                    double range2 = robotUltrasonicSensor.getRangeInches(); //get the distance in inches
//
//                    double range3 = robotUltrasonicSensor.getRangeInches(); //get the distance in inches
//
//
//                    maxRange = 0;
//                    if (range1 > maxRange) {    //if the range is greater than it was before store as the max value
//                        maxRange = range1;
//                    }
//                    if (range2 > maxRange) {    //if the range is greater than it was before store as the max value
//                        maxRange = range2;
//                    }
//                    if (range3 > maxRange) {    //if the range is greater than it was before store as the max value
//                        maxRange = range3;
//                    }
//
//                    minRange = 10000000;
//                    if (range1 < minRange) { //if the range is smaller than it was before store it as the min value
//                        minRange = range1;   //store range as the min range value
//                    }
//                    if (range2 < minRange) { //if the range is smaller than it was before store it as the min value
//                        minRange = range2;   //store range as the min range value
//                    }
//                    if (range3 < minRange) { //if the range is smaller than it was before store it as the min value
//                        minRange = range3;   //store range as the min range value
//                    }

//DRIVER BUTTONS *********************************************************************************************************************************
                //ARCADE DRIVE/LATERAL DRIVE - DRIVER - BUTTON 1.
                    if (this.getRightButtonState(rightButton1)) {
                        robotDriveTrain.arcadeDrive(0, 0);          //stop the drive
                        Timer.delay(lateralDelay);                  //two second delay before the lateral can be used 0.1 sec
                        lateralDelay = 0;                           //set the lateral delay to zero
                        robotSolenoidLateralDrive.set(true);        //drop the lateral drive
                        roboVictorLateralDrive.set(rightStick.getX());

                    } else {
                        roboVictorLateralDrive.set(0);
                        robotSolenoidLateralDrive.set(false);           //bring up the lateral drive
                        lateralDelay = 0.1;                             //return the lateral delay back to 0.1
                        robotDriveTrain.arcadeDrive(rightStick, false); //arcade drive usb 1 drive
                    }                                                 
                                   

                //AUTO-DELIVER FLOOR TUBE TO ARM - DRIVER - BUTTON 2
                    if ((this.getRightButtonState(rightButton2)) && (m_doWeHaveAFloorTubeToLoad)) {
                        rightButton2Pressed = true;
                        autoModeState = 1;
                    }
                    if ((rightButton2Pressed)) {
                            //|| (rightButton2Pressed)) {
//                        rightButton2Pressed = true; //we hit the button
                        
                        switch ((int)autoModeState) {
                            case 0:
                                System.out.println("Case 0 hit");
                                autoCounter = 0;
                                break;
                            case 1: //bring elevator arm down
                                System.out.println("Case 1 hit");
                                elevatorLift = robotPotElevatorArm.getVoltage();   //Takes voltage from pot and sets it as "elevatorLift"
                                System.out.println("Elevator Arm pot: " + elevatorLift);
                                
                                    if ((elevatorLift < potElevatorArmLowerLimit)) { //move elevator arm down to load floor tube
                                        robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kReverse);    //release the tube
                                        robotDoubleSolenoidElevatorUpDown4.set(DoubleSolenoid.Value.kForward);      //extend the 4" elevator
                                        robotDoubleSolenoidMainElevator.set(DoubleSolenoid.Value.kReverse);         //retract the main elevator
                                        robotJaguarElevatorArm.set(1);   //drive the elevator arm downward (to grab the floor tube)
                                    } else { //our elevator is down. move to next case
                                        robotJaguarElevatorArm.set(0); //stop the elevator arm
                                        autoModeState = 2;
                                    }
                                break;
                            case 2: //bring the floor arm up to load
                                System.out.println("Case 2 hit");
                                floorLift = robotPotFloorArm.getVoltage();   //Takes voltage from pot and sets it as "floorLift"
                                    //figure out which direction to drive the floor arm to load
                                    //if you are above the lower limit and less than the human limit move upward
                                    if ((floorLift < potFloorArmLoadLimitLower)) {
                                        robotJaguarFloorArm.set(-0.75);  //drive the floor arm upward
                                    }
                                    else {
                                        robotJaguarFloorArm.set(0);  //stop the floor arm
                                        autoModeState = 3;
                                    }
                                break;
                            case 3: //release floor tube
                                System.out.println("Case 3 hit");
                                robotDoubleSolenoidFloorGrabber.set(DoubleSolenoid.Value.kReverse);  //open floor arm(let it drop in)
                                autoCounter++;
                                if (autoCounter > 11) { //small "delay" before we grip the tube with the main elevator
                                autoModeState = 4;
                                }
                                break;
                            case 4: //grab the tube with main arm
                                System.out.println("Case 4 hit");
                                robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kForward);    //grip the floor tube
                                autoCounter2++;  //small delay while we grip the tube
                                if (autoCounter2 > 25) {
                                robotDoubleSolenoidElevatorUpDown4.set(DoubleSolenoid.Value.kReverse);      //retract the 4" elevator
                                    if (autoCounter2 > 50) {
                                        autoModeState = 5;
                                    }
                                }
                                break;
                            case 5: //move floor arm out of the way of the main elevator
                                System.out.println("Case 5 hit");
                                floorLift = robotPotFloorArm.getVoltage();   //Takes voltage from pot and sets it as "floorLift"
                                 if (floorLift >= potFloorArmOutOfTheWayLimit) {
                                    robotJaguarFloorArm.set(0.75);  //drive the floor arm down
                                } else {
                                    robotJaguarFloorArm.set(0);  //stop floor arm
                                    autoModeState = 6;
                                }
                                break;
                            case 6: //return elevator arm to humn player level
                                System.out.println("Case 6 hit");
                                elevatorLift = robotPotElevatorArm.getVoltage();   //Takes voltage from pot and sets it as "elevatorLift
                                    //figure out which direction to drive the elevator arm tot the human player
                                    //if you are above the lower limit and less than the human limit move upward
                                    if (elevatorLift > potElevatorArmHumanLimitLower) {
                                        robotJaguarElevatorArm.set(-0.5);  //drive the elvator arm upward
                                    }
                                    else {
                                        robotJaguarElevatorArm.set(0);  //stop the elevator arm
                                        autoModeState = 7;
                                    }
                                break;
                            case 7: //end the state machine
                                System.out.println("Case 7 hit");
                                autoModeState = 0; //back to doing nothing
                                autoCounter = 0; //reset the counter
                                autoCounter2 = 0;
                                rightButton2Pressed = false;
                                m_doWeHaveAFloorTubeToLoad = false;
                                break;
                        }

                    }
//                    else { //ready to run for the next time
////                        rightButton2Pressed = false; //we didn't hit the button
//                        autoModeState = 1;
//                    }

//                        double elevatorLift = robotPotElevatorArm.getVoltage();   //Takes voltage from pot and sets it as "elevatorLift"

//                        if ((elevatorLift < potElevatorArmLowerLimit)) { //move elevator arm down to load floor tube
//
//                            robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kReverse);    //release the tube
//                            robotJaguarElevatorArm.set(1);   //drive the elevator arm downward (to grab the floor tube)
//                        } else { //we are ready to bring up the floor arm and load the tube arm
//
//                            if (m_readyToLetGoOfFloorTube) { //if the floor arm and the elevator are in the correct spots
//                            m_DeliverToArm = false; //don't make the floor arm start to come up to load
//                            robotDoubleSolenoidFloorGrabber.set(DoubleSolenoid.Value.kForward);  //open floor arm(let it drop in)
//                            robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kForward);    //grip the flor tube
//                            rightButton2Pressed = false; //we are done with this
//                            } else {
//                            robotJaguarElevatorArm.set(0);  //stop the elevator arm
//                            m_DeliverToArm = true; //make the floor arm start to come up to load
//                            }
//                        }
               


      //FLOOR ARM - DRIVER - being altered
                     //B4 move down
                     if((getRightButtonState(rightButton4))) {
                         floorLift = robotPotFloorArm.getVoltage();   //Takes voltage from pot and sets it as "floorLift"
                         rightButton2Pressed = false;                       //override the auto load function (B2)
                         autoModeState = 7;
                         if ((floorLift > potFloorArmLowerLimit)) {
                            robotJaguarFloorArm.set(0.75);  //drive the floor arm down
                        } else {
                            robotJaguarFloorArm.set(0);  //stop floor arm
                        }
                         
                     }
                     //B7 to load
                    else if((getRightButtonState(rightButton7))) {//|| (m_DeliverToArm)) {
                        rightButton2Pressed = false;                        //override the auto load function (B2)
                        autoModeState = 7;
                        floorArmToLoad(); //bring up the floor arm to load

                    }
                     //B5 move up
                    else if ((getRightButtonState(rightButton5))) {
                        floorLift = robotPotFloorArm.getVoltage();   //Takes voltage from pot and sets it as "floorLift"
                        rightButton2Pressed = false;                        //override the auto load function (B2)
                        autoModeState = 7;
                         if ((floorLift < potFloorArmUpperLimit)) {
                            robotJaguarFloorArm.set(-0.75);  //drive the floor arm up
                        } else {
                            robotJaguarFloorArm.set(0);  //stop floor arm
                        }
                    }
                     else {  //we haven't hit any button so don't move the floor arm
                    //Matt is awesome -Kelsie

                        if (!rightButton2Pressed) {
                            robotJaguarFloorArm.set(0);  //stop the floor arm
                        }

                     }
                    
                    //FLOOR PICK UP ARM OPEN - DRIVER - BUTTON 6
                     if ((this.getRightButtonState(rightButton6))) {
                         robotDoubleSolenoidFloorGrabber.set(DoubleSolenoid.Value.kReverse);  //open floor arm
                         m_doWeHaveAFloorTubeToLoad = false;
                     }
                    //FLOOR PICK UP ARM CLOSE - DRIVER - BUTTON 3
                     if ((this.getRightButtonState(rightButton3))) {
                         robotDoubleSolenoidFloorGrabber.set(DoubleSolenoid.Value.kForward);  //close floor arm
                         m_doWeHaveAFloorTubeToLoad = true;
                     }

                    //AUTOMODESTATE = 0 - DRIVER -BUTTON 8 - being altered
                    if (this.getRightButtonState(rightButton8)) {
                        autoModeState = 7; //stop the state machine
                    }
                    //Pete is awesome! 
                 //FLOOR ARM DOWN - DRIVER - BUTTON 4
                    //if we hit b4 and we are higher than the floor arm's lower limit
//                 if ((getRightButtonState(rightButton4))){
//                            robotJaguarFloorArm.set(0.75);   //drives the floor arm down
//                 }           
                 
                 //FLOOR ARM TO STOW - DRIVER - BUTTON 5
                 //if we hit b5 and we are lower than the floor arm's upper limit
                 //Luke is awesome?
//                 else if((getRightButtonState(rightButton5))) {
                    
//                       robotJaguarFloorArm.set(-0.75); //bring the arm up to be stowed

//                    }
//                 else { //we didn't hit any button/we hit a limit so stop
//                        robotJaguarFloorArm.set(0);
//                      }

                //RESET THE ENCODERS - DRIVER - BUTTON 10
                    if(this.getRightButtonState(rightButton10)) {
                        robotEncoderLeftDrive.reset();  //reset the left drive encoder
                        robotEncoderRightDrive.reset(); //reset the right drive encoder
                        robotEncoderLateralDrive.reset();  //reset the lateral drive encoder
                    }
                        
                //DATA PRINT TO THIS COMPUTER - DRIVER - BUTTON 11
                    if(this.getRightButtonState(rightButton11)) {
                        System.out.println("Elevator Arm Pot Value: " + robotPotElevatorArm.getVoltage());    //Print Elevator pot value
                        System.out.println("Floor Arm Pot Value: " + robotPotFloorArm.getVoltage());    //Print floor Arm value
                        System.out.println("Left Drive Encoder Value: " + robotEncoderLeftDrive.getRaw());   //Print Left Drive Value
                        System.out.println("Right Drive Encoder Value: " + robotEncoderRightDrive.getRaw()); //Print Right Drive Value
                        System.out.println("Lateral Drive Encoder Value: " + robotEncoderLateralDrive.getRaw()); //Print Lateral Drive Value
                        System.out.println("PegFoundDigitalIn Value: " + robotDigitalInPegFound.get()); //Print if we found the peg (boolean)
                        System.out.println("LeftDigitalIn Value: " + robotDigitalInLeft.get()); //Print the left digitalin value (boolean)
                        System.out.println("MiddleDigitalIn Value: " + robotDigitalInMiddle.get()); //Print the middle digitalin value (boolean)
                        System.out.println("RightDigitalIn Value: " + robotDigitalInRight.get()); //Print the right digitalin value (boolean)
                        System.out.println("Right Button 3" + getRightButtonState(rightButton3));
                        System.out.println("Right Button 4" + getRightButtonState(rightButton4));
                        System.out.println("Right Button 5" + getRightButtonState(rightButton5));
//                        System.out.println("Ultrasonic Sensor Max Range: " + range1); //Print the ultrasonic sensor
//                        System.out.println("Ultrasonic Sensor Min Range: " + minRange); //Print the ultrasonic sensor minRange value (double)
                    }


                     
//ELEVATOR BUTTONS *********************************************************************************************************************************                        
                //ENABLE RAISE AND LOWER ELEVATOR ARM - ELEVATOR - BUTTON 1
                    if ((this.getLeftButtonState(leftButton1)) && (!leftButton6Pressed) && (!leftButton7Pressed)) {

                        elevatorLift = robotPotElevatorArm.getVoltage();             //Takes voltage from pot and sets it as "elevatorLift"
                        double y = leftStick.getY();
                        leftButton1Pressed = true;  //we pressed button 1
                        rightButton2Pressed = false;                                //Override the auto-pick up
                        autoModeState = 7;

                            //if you want to go up and the pot value is greater than the upper limit go up
                            if ((y > 0.15) && (elevatorLift > potElevatorArmUpperLimit)) {
                                robotJaguarElevatorArm.set(-y);

                            } else if ((y < -0.15) && (elevatorLift < potElevatorArmLowerLimit)) { //if you want to go down and the pot value is less than the lower limit go down
                                robotJaguarElevatorArm.set(-y);

                            } else {
                                robotJaguarElevatorArm.set(0);
                            }

                    }


                    else {

                        if (!rightButton2Pressed) {
                            robotJaguarElevatorArm.set(0);   //stop the arm when not holding the button
                            leftButton1Pressed = false;  //we didn't pressed button 1
                        }
                    }


                //GO TO TOP PEG - ELEVATOR - BUTTON 2
                     if ((this.getLeftButtonState(leftButton2) ^ m_previousStateTopPeg)  & this.getLeftButtonState(leftButton2)){
                             m_solenoidMainElevatorTopPegLatch   = ! m_solenoidMainElevatorTopPegLatch;
                        }

                        m_previousStateTopPeg = this.getLeftButtonState(leftButton2);   //store the value (T or F) when we hit B2


                        if(m_solenoidMainElevatorTopPegLatch){      //if button 2 is true move the elevator upward                            
                            if (!rightButton2Pressed) {
                            robotDoubleSolenoidMainElevator.set(DoubleSolenoid.Value.kForward); //up
                            }
                        } else {                        //if button 2 is false move the elevator downward
                            if (!rightButton2Pressed) {
                            robotDoubleSolenoidMainElevator.set(DoubleSolenoid.Value.kReverse);  //down
                            }
                        }


                //GO UP/DOWN 4 INCHES - ELEVATOR - BUTTON 3
                     if ((this.getLeftButtonState(leftButton3) ^ m_previousStateElevatorUpDown8)  & this.getLeftButtonState(leftButton3)){
                             m_solenoidElevatorUpDown8Latch   = ! m_solenoidElevatorUpDown8Latch;
                        }

                        m_previousStateElevatorUpDown8 = this.getLeftButtonState(leftButton3);   //store the value (T or F) when we hit B3


                        if(m_solenoidElevatorUpDown8Latch){      //if button 3 is true move the elevator up 4 inches
                            if (!rightButton2Pressed) {
                            robotDoubleSolenoidElevatorUpDown4.set(DoubleSolenoid.Value.kForward); //extend
                            }
                        } else {                        //if button 3 is false move the elevator downward 4 inches
                            if (!rightButton2Pressed) {
                            robotDoubleSolenoidElevatorUpDown4.set(DoubleSolenoid.Value.kReverse); //retract
                            }
                        }


                //GRAB TUBE - ELEVATOR - BUTTON 4
                     if (this.getLeftButtonState(leftButton4)) {
                        robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kForward);  //grab the tube
                     }


                //RELEASE TUBE - ELEVATOR - BUTTON 5
                     if (this.getLeftButtonState(leftButton5)) {
                        robotDoubleSolenoidTubeGrabber.set(DoubleSolenoid.Value.kReverse);    //release the tube
                     }               


                //ELEVATOR ARM TO HUMAN PLAYER - ELEVATOR - BUTTON 6
                    if ((getLeftButtonState(leftButton6)) || (leftButton6Pressed)){
                        elevatorLift = robotPotElevatorArm.getVoltage();   //Takes voltage from pot and sets it as "elevatorLift"
                        leftButton6Pressed = true;  //we pressed the to human button
                        //figure out which direction to drive the elevator arm tot the human player
                        //if you are above the lower limit and less than the human limit move upward
                        if ((elevatorLift <= potElevatorArmLowerLimit) && (elevatorLift > potElevatorArmHumanLimitLower)) {
                            robotJaguarElevatorArm.set(-0.5);  //drive the elvator arm upward
                        }
                        //if you are lower than the upper limit and higher than the human limit move downward
                        else if ((elevatorLift >= potElevatorArmUpperLimit) && (elevatorLift < potElevatorArmHumanLimitUpper)) {
                            robotJaguarElevatorArm.set(0.5);   //drive the elevator arm downward
                        }
                        else {
                            robotJaguarElevatorArm.set(0);  //stop the elevator arm
                            leftButton6Pressed = false; //back to not pressing the arm to human button
                        }
                    }


                //ELEVATOR ARM TO VERTICAL - ELEVATOR - BUTTON 7
                     if ((this.getLeftButtonState(leftButton7)) || (leftButton7Pressed)) {
                        elevatorLift = robotPotElevatorArm.getVoltage();   //Takes voltage from pot and sets it as "elevatorLift"
                        leftButton7Pressed = true;  //we pressed the arm to vertical button

                            if ((elevatorLift > potElevatorArmUpperLimit)){
                            robotJaguarElevatorArm.set(-0.5);  //drive the elevator arm up the the vertical pos
                            } else {
                               robotJaguarElevatorArm.set(0);
                               leftButton7Pressed = false;  //back to not pressing the arm to vertical button
                            }
                     }
                

                        
                //DEPLOY MINIBOT TO POLE - ELEVATOR - BUTTON 10
                     leftAxisZ = leftStick.getZ();
//                     System.out.println("Left Stick Z Axis " + leftAxisZ);
                     //if (B10 pressed && last 10sec && minibot arm deployed) or (B10 pressed && fluxC disabled && minibot arm deployed)
                     if (((this.getLeftButtonState(leftButton10)) && (fluxCapacitor >= 110) && (m_solenoidMiniBotArmLatch)) || (((this.getLeftButtonState(leftButton10)) && (leftAxisZ == 1)) && (m_solenoidMiniBotArmLatch))) {
                        robotSolenoidMinibotDeploy.set(true);     //Deploy Minibot
                        Timer.delay(2.5);                          //While deploying wait before retracting
                        robotSolenoidMinibotDeploy.set(false);     //Retract Minibot
                     }


                //MINIBOT ARM OUT/IN - ELEVATOR - BUTTON 11
                     if ((this.getLeftButtonState(leftButton11) ^ m_previousStateMinibotArm)  & this.getLeftButtonState(leftButton11)){
                             m_solenoidMiniBotArmLatch   = ! m_solenoidMiniBotArmLatch;
                        }

                        m_previousStateMinibotArm = this.getLeftButtonState(leftButton11);   //store the state of B6 (did we hit it or not)


                        if(m_solenoidMiniBotArmLatch){      //if button 6 Deploy arm (it's latched)
                            robotDoubleSolenoidMinibotArmSolenoidOne.set(DoubleSolenoid.Value.kForward);

                        } else {                        //if button 6 Return arm
                            robotDoubleSolenoidMinibotArmSolenoidOne.set(DoubleSolenoid.Value.kReverse);
                        }
    }



    
    public void autonomousContinuous() {

    }
    public void teleopContinuous() {

    }
    public void disabledContinuous() {

    }


/********************************** Autonomous Box "Mode" *************************************/
//Autonomous box decides what autonomous mode we select.
//autoMode is what autonomous mode we are in. if 0 do nothing.
    public void autonomusBoxReadingMode() {
            autoMode = m_ds_Buzz.getAnalogIn(1);  //read the first analog in port 1 on the DS            
//            System.out.println("Auto Mode before calc " + autoMode);            

            autoMode = (int)Math.floor(autoMode * 2 + 0.9f);   //take the data from the auto box and convert it to useable numbers            
//            System.out.println("Auto Mode after calc " + autoMode);            

            switch ((int)autoMode) {      //"select" the automous case that we are going to used from the analog voltage from auto box

                case 2: //mode 1
                    System.out.println("Automonous mode 1:  ");
                    autoDelay = 0.5;
                    autoCase = 1;
                    break;
                case 3: //mode 2
                    System.out.println("Automonous mode 2:  Drive straight line, raise elevator/arm, stop and score (Encoders)");
                    autoDelay = 1;
                    break;
                case 4: //mode 3
                    System.out.println("Automonous mode 3:  Line track the straight line and score");
                    autoDelay = 0.75;
                    break;
                case 6: //mode 4
                    System.out.println("Automonous mode 4: Line Track forked line, move left with lateral, score");
                    autoDelay = 0.2;   //delay while we stop the drive and deploy the lateral
                    autoDelay2 = 0.2;  //delay while we stop the lateral drive and bring up the lateral
                    autoDelay3 = 0.75;  //delay while we stop the drive, drop the tube, and back up
                    break;
                case 7: //mode 5
                    System.out.println("Automonous mode 5: Line Track forked line, move right with lateral, score");
                    autoDelay = 0.2;   //delay while we stop the drive and deploy the lateral
                    autoDelay2 = 0.2;  //delay while we stop the lateral drive and bring up the lateral
                    autoDelay3 = 0.75;  //delay while we stop the drive, drop the tube, and back up
                    break;
               default:
                   System.out.println("Auto Mode 0:  Do nothing");      //The mode that simply does nothing...
                   autoMode = 0; //if it screws up it will do nothing
                   break;
            }
        }



/********************************** Autonomous Box "Yaled" *************************************/
//Autonomous box also decides the length of the delay before autonomous begins to run.
//autoYaled is the time (in Seconds) that we will wait before running autonomous. A mode is = 0 sec.
//Please note that "Yaled" is "Delay" spelled backwards.  :)
    public void autonomusBoxReadingYaled() {

            autoYaled = m_ds_Buzz.getAnalogIn(2);  //read the second analog on the DS (used to set begining delay for autoMode)
            System.out.println("Auto Yaled before calc " + autoYaled);
            
            autoYaled = (int)Math.floor(autoYaled * 2 + 0.9f); //take the data from the auto box and convert it to useable numbers
            System.out.println("Auto Yaled after calc " + autoYaled);

            switch ((int)autoYaled) {   //"select" the delay time that we want from the analog voltage from auto box

                case 2: //Yaled B:  Delay by 0.5 seconds
                    System.out.println("Yaled by 0.5 seconds");
                    Yaled = 0.5;
                    break;
                case 3: //Yaled C:  Delay by 1 seconds
                    System.out.println("Yaled by 1 seconds");
                    Yaled = 1;
                    break;
                case 4: //Yaled D:  Delay by 1.5 seconds
                    System.out.println("Yaled by 2 seconds");
                    Yaled = 2;
                    break;
                case 6: //Yaled E:  Delay by 2 second
                    System.out.println("Yaled by 3 second");
                    Yaled = 3;
                    break;
                case 7: //Yaled F:  Delay by 3 seconds
                    System.out.println("Yaled by 4 seconds");
                    Yaled = 4;
                    break;
               default:
                   System.out.println("No Yaled");          //0 is no time delay
                   Yaled = 0;
                   break;
            }
        }


/********************************** Up to TOP Peg in Autonomous Mode *************************************/
//Bring the elevator arm upward and extend it to the TOP PEG
    public void autonomousElevatorTop() {
        //bring up the arm and extend the elevator all the way up to the top most peg
        double armPotValue = robotPotElevatorArm.getVoltage();
                    if (((armPotValue) <= potElevatorArmUpperLimit)) {
                        robotJaguarElevatorArm.set(0);         //If arm is up stop arm, extend main elevator
                        robotDoubleSolenoidMainElevator.set(DoubleSolenoid.Value.kForward);
                    } else{
                        robotJaguarElevatorArm.set(-1);         //bring up the elevator arm
                   }
    }

    
/********************************** Up to HIGHEST PEG in Autonomous Mode *************************************/
//Bring the elevator arm upward and extend it to the HIGHEST PEG
    public void autonomousElevatorTopTop() {
        //bring up the arm and extend the elevator all the way up to the top most peg
        double armPotValue = robotPotElevatorArm.getVoltage();
                    if (((armPotValue) <= potElevatorArmUpperLimit)) {
                        robotJaguarElevatorArm.set(0);         //If arm is up stop arm, extend 4", extend main elevator
                        robotDoubleSolenoidMainElevator.set(DoubleSolenoid.Value.kForward);
                        robotDoubleSolenoidElevatorUpDown4.set(DoubleSolenoid.Value.kForward);
                    } else{
                        robotJaguarElevatorArm.set(-1);         //bring up the elevator arm
                   }
    }


/********************************** Bring the Elevator and the Arm back down in Autonomous Mode *************************************/
//Bring the elevator arm back down and unextend it
    public void autonomousBringElevatorArmBackDown() {
        //bring down the arm and collapse the elevator
        double armPotValue = robotPotElevatorArm.getVoltage();
                    if (((armPotValue) >= potElevatorArmLowerLimit)) {
                        robotJaguarElevatorArm.set(0);         //if the arm is back down then stop it, bring down the main elevator, unextend the 4"
                        robotDoubleSolenoidMainElevator.set(DoubleSolenoid.Value.kReverse);
                        robotDoubleSolenoidElevatorUpDown4.set(DoubleSolenoid.Value.kReverse);
                    } else {
                        robotJaguarElevatorArm.set(1);         //bring the elevator arm down
                   }
    }


/********************************** Print out data for Autonomous Mode *************************************/
//Print out the left, middle, and right line trackers (false = not found)
//Print out right, left, and lateral drive encoder values
    public void autonomousPrintOut() {
        //Print data to screen
            System.out.println("Left Digital Input: " + robotDigitalInLeft.get());                  //Print Left Digital Input
            System.out.println("Middle Digital Input: " + robotDigitalInMiddle.get());              //Print Middle Digital Input
            System.out.println("Right Digital Input: " + robotDigitalInRight.get());                //Print Right Digital Inpu                        System.out.println("Left Drive Encoder Value: " + encoderCountsLeft);                 //Print Left Drive Value
            System.out.println("Right Drive Encoder Value: " + robotEncoderRightDrive.getRaw());     //Print Right Drive Value
            System.out.println("Left Drive Encoder Value: " + robotEncoderLeftDrive.getRaw());
            System.out.println("Lateral Drive Encoder Value: " + robotEncoderLateralDrive.getRaw());
    }


   /********************************** Bring the floor arm up to load *************************************/
    public void floorArmToLoad() {
        floorLift = robotPotFloorArm.getVoltage();   //Takes voltage from pot and sets it as "floorLift"
                        //figure out which direction to drive the floor arm to load
                        //if you are above the lower limit and less than the human limit move upward
                        if ((floorLift >= potFloorArmLowerLimit) && (floorLift < potFloorArmLoadLimitLower)) {
                            robotJaguarFloorArm.set(-0.75);  //drive the floor arm upward
                        }
                        //if you are less than the upper limit and higher than the lower limit move downward
                        else if ((floorLift <= potFloorArmUpperLimit) && (floorLift > potFloorArmLoadLimitUpper)) {
                            robotJaguarFloorArm.set(0.75);   //drive the floor arm downward
                        }
                        else {
                            robotJaguarFloorArm.set(0);  //stop the floor arm
                        }
    }
    
/********************************** Camera (Axis...) *************************************/
//Camera code

    public void camera(int channelnum){
            Watchdog.getInstance().feed();

        try {
            if (robotCamera.freshImage()) {
            ColorImage image = robotCamera.getImage();
            Thread.yield();
            Thread.yield();
            image.free();

            }
            
        } catch (NIVisionException ex) {
            ex.printStackTrace();
        } catch (AxisCameraException ex) {
            ex.printStackTrace();
        }
    }

    int GetLoopsPerSec() {
        return 10000;
    }

      private boolean  getRightButtonState(int  buttonNo){
        return rightStick.getRawButton(buttonNo);
    }

      private boolean  getLeftButtonState(int  buttonNo){
        return leftStick.getRawButton(buttonNo);
    }
    
}
//SMAW!!!