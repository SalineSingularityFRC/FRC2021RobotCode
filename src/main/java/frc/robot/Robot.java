/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import frc.controller.*;
import frc.singularityDrive.*;
import frc.controller.controlSchemes.ArcadeDrive;
import frc.controller.controlSchemes.SmartArcadeDrive;
import frc.controller.motorControllers.Falcon;
import frc.controller.autonomous.*;
//import frc.controller.controlSchemes.Test;
import frc.robot.Canifier;
import frc.robot.Json;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.rmi.NoSuchObjectException;
import java.util.HashMap;
import org.json.simple.*;
import org.json.simple.parser.*;
import com.kauailabs.navx.frc.AHRS;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // stores the motor controller IDs
  int driveLeft1, driveLeft2, driveLeft3, driveRight1, driveRight2, driveRight3;
  int drivePneu1, drivePneu2;
  int colorSol1, colorSol2;
  int climberSol1, climberSol2;
  int colorSpinner;

  int flywheelMotor1, flywheelMotor2, flywheelMotor3;
  int conveyorMotor1, conveyorMotor2;
  int downMotorPort;

  // Declaration of our driving scheme, which can be initialized to
  // any ControlScheme in robotInit()
  ControlScheme currentScheme;
  

  Json json = new Json();

  String[] motorNames = { "FL_Angle", "FL_Wheel", "BL_Angle", "BL_Wheel", "FR_Angle", "FR_Wheel", "BR_Angle",
      "BR_Wheel", "Flywheel1", "Flywheel2", "FeederMotor" };

  HashMap<String, MotorController> motors = new HashMap<String, MotorController>();

  // Declaration of mechanisms
  SwerveDrive drive;
  // SmartSingDrive smartDrive; //if we want to use smart motion, change this to
  // SmartSingDrive
  // DrivePneumatics drivePneumatics;
  Flywheel flywheel;
  // Conveyor conveyor;
  // Climber climber;

  // Creates an all-knowing limelight
  LimeLight limeLight; // or CitrusSight?

  // Create a CANifier
  // Canifier canifier;

  // Create a ColorSensor
  // ColorSensor colorSensor;

  // Create a gyro
  AHRS gyro;
  // boolean gyroResetAtTeleop;


  // Compressor compressor;
  // Compressor compressor;

  /*
   * Falcon talon1; Falcon talon2; Falcon talon3; Falcon talon4;
   * 
   * Falcon talon5; Falcon talon6; Falcon talon7; Falcon talon8;
   */

  // SendableChoosers
  // SendableChooser<Integer> goalChooser;
  // SendableChooser<Integer> positionChooser;
  // SendableChooser<Integer> secondaryChooser;

  // SendableChooser autoChooser;

  // default ports of certain joysticks in DriverStation
  final int XBOX_PORT = 0;
  final int BIG_JOYSTICK_PORT = 1;
  final int SMALL_JOYSTICK_PORT = 2;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // TODO un-comment methods
    // initialize motor controller ports IDs
    // Uncomment to initialize motor controllers aswell - commented for texting
    // purposes
    setDefaultProperties();
    // CameraServer.getInstance().startAutomaticCapture();
    // CameraServer.getInstance().startAutomaticCapture();

    // initialize our driving scheme to a basic arcade drive
    currentScheme = new ArcadeDrive(XBOX_PORT, XBOX_PORT +1);

    //gyro = new AHRS(SPI.Port.kMXP);
    ((ArcadeDrive)currentScheme).initGyro();
    // gyroResetAtTeleop = true;

    // colorSensor = new ColorSensor(colorSpinner, colorSol1, colorSol2);

    // initialize all mechanisms on the robot
    // smartDrive = new SmartBasicDrive(driveLeft1, driveLeft2, driveLeft3,
    // driveRight1, driveRight2, driveRight3);

    /*
     * talon1 = new Falcon(1, .25, true); talon2 = new Falcon(2, .25, true); talon3
     * = new Falcon(3, .25, true); talon4 = new Falcon(4, .25, true);
     * 
     * talon5 = new Falcon(5, .25, true); talon6 = new Falcon(6, .25, true); talon7
     * = new Falcon(7, .25, true); talon8 = new Falcon(8, .25, true);
     */

    for (int i = 0; i < motorNames.length; i++) {
      try {
        motors.put(motorNames[i], json.getSwerveDriveMotors(motorNames[i]));
      } catch (NoSuchObjectException e) {
        
        //e.printStackTrace();
        try{
          motors.put(motorNames[i], json.getShooterMotor(motorNames[i]));

        } catch (NoSuchObjectException f){
          //e.printStackTrace();
        }
        
      }
    }



    
    if(json.isObject("SwerveDrive")){
      drive = new SwerveDrive(motors.get("FL_Angle").getCanID(), motors.get("FL_Wheel").getCanID(), motors.get("FR_Angle").getCanID(), motors.get("FR_Wheel").getCanID(), 
        motors.get("BL_Angle").getCanID(), motors.get("BL_Wheel").getCanID(), motors.get("BR_Angle").getCanID(), motors.get("BR_Wheel").getCanID(), 1, 1, 1, 1);
    }
    //drive = new BasicDrive(driveLeft1, driveLeft2, driveLeft3, driveRight1, driveRight2, driveRight3);
    // ^^^^^^^ change this to SmartBasicDrive if using SmartDrive
    //drivePneumatics = new DrivePneumatics(drivePneu1, drivePneu2);
    if(json.isObject("FLywheel")){
      flywheel = new Flywheel(motors.get("Flywheel1").getCanID(), motors.get("Flywheel2").getCanID(), motors.get("FeederMotor").getCanID());
      //flywheel = new Flywheel(flywheelMotor1, flywheelMotor2, flywheelMotor3);
    }
    
    //conveyor = new Conveyor(conveyorMotor1);



    //climber = new Climber(downMotorPort,0,0);//TODO THE LAST TWO NUNMBERS AREN'T CORRECT

    
    limeLight = new LimeLight();
    //limeLight.setCamMode(limeLight, 0.0);
    //DO NOT REMOVE PLZ - starts collecting data from drive cameras
    //start collecting data from drive cameras
    // This is not used if the raspberry pi is being used for image compression
    //CameraServer.getInstance().startAutomaticCapture();

    gyro = new AHRS(SPI.Port.kMXP);
    //gyroResetAtTeleop = true;

    //tutorial code for the sendableChooser in case it breaks
    /*autoChooser = new SendableChooser();
    autoChooser.addDefault("Default Auto", new TestAuton(drive, limeLight));
    autoChooser.addOption("SupremeAuto", new JustMove(drive, limeLight));
    SmartDashboard.putData("Auto mode", autoChooser);*/

    //compressor = new Compressor();
    //goalChooser = new SendableChooser<Integer>();
    //positionChooser = new SendableChooser<Integer>();
    //secondaryChooser = new SendableChooser<Integer>();

    //goalChooser.addDefault("Just Move",1);
    //goalChooser.addOption("Target and Trench", 0);
    //goalChooser.addOption("TestAuton", 2);

    //positionChooser.addDefault("Position 1", 0);
    //positionChooser.addOption("Position 2", 1);
    //positionChooser.addOption("Position 3", 2);

    //secondaryChooser.addDefault("Move Through Trench", 1);
    //secondaryChooser.addOption("Don't", 0);

    //SmartDashboard.putData("Position", positionChooser);
    //SmartDashboard.putData("Primary Goal", goalChooser);
    //SmartDashboard.putData("Secondary Goal", secondaryChooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    //compressor.start();
    //Falcon talon = new Falcon(3);
    


  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    //Positions 1,2,3
    //Goals
    //  0: Just Move
    //  1: Trench and Shoot
    //secondaryGoals
    //  0: Move through Trench
    //  1: Don't
   /*AutonControlScheme[] goals={ new Trench1(drive, limeLight, flywheel, conveyor),
                                 new Trench2(drive, limeLight, flywheel, conveyor),
                                 new Trench3(drive, limeLight, flywheel, conveyor)};
    
    SmartDashboard.putNumber("result of position", positionChooser.getSelected());
    SmartDashboard.putNumber("result of goals", goalChooser.getSelected());
    SmartDashboard.putNumber("result of secondary goals", secondaryChooser.getSelected());

    if(goalChooser.getSelected() == 2){
      SmartDashboard.putString("autoprogram", "Test Auton");
      new TestAuton(drive, limeLight, flywheel, conveyor);
    }
     else if(goalChooser.getSelected() == 1){
      SmartDashboard.putString("autoprogram", "MoveAndShoot");
      new ShootAndMove(drive, limeLight, flywheel, conveyor).moveAuton();
    }
    else{
      SmartDashboard.putString("autoprogram", "PrimaryGoal");
      goals[positionChooser.getSelected()].moveAuton();
    }
    if(secondaryChooser.getSelected() == 1 && goalChooser.getSelected() == 0){
      SmartDashboard.putString("autoprogram", "SecondaryGoal");
      new MoveThroughTrench(drive, limeLight, flywheel, conveyor).moveAuton();
    }
    
    /*AutonControlScheme hodl = new TestAuton(drive, limeLight, flywheel, conveyor, collector);
    hodl.moveAuton();*/
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //currentScheme.drive(drive, drivePneumatics);
    //currentScheme.ledMode(limeLight);
    
  }

  //Stuff to run when teleop is selected
  @Override
  public void teleopInit() {
    //drive.setInitialPosition();
    //Falcon talon = new Falcon(3);
    


  }

  /**
   * Function that contains everything that will run in the teleop perio/option in DS
   */
  @Override
  public void teleopPeriodic() {
    //SmartDashboard.putNumber("Big Number Gyro Angle", gyro.getAngle());
    
    System.out.println("\nFeedback Sensor Position: \t" + ((Falcon)motors.get("FL_Angle")).getFeedbackSensorPos());
    System.out.println("\nIntegrated Sensor Position: \t" + ((Falcon)motors.get("FL_Angle")).getIntegratedSensorPos());


    /*
    talon1.setSpeed(100);
    talon2.setSpeed(100);
    talon3.setSpeed(100);
    talon4.setSpeed(100);

    talon5.setSpeed(100);
    talon6.setSpeed(100);
    talon7.setSpeed(100);
    talon8.setSpeed(100);*/

    // Allow driver control based on current schem
/*
    boolean colorData[] = canifier.getPinData();
    int color = canifier.binToDecColor(colorData);
    int count = canifier.binToDecCount(colorData);
    String byteString = canifier.byteArrayToString(colorData);

    SmartDashboard.putNumber("Current Color: ", color);
    SmartDashboard.putNumber("Current Count: ", count);
    SmartDashboard.putString("Byte Transfered: ", byteString);
*/  
    //colorSensor.spinColorWheelColor(2);
    //currentScheme.colorSensor(colorSensor);

    //SmartDashboard.putNumber("EncoderPosition", drive.getCurrentPosition());
    //currentScheme.smartDrive(smartDrive, drivePneumatics);
    // partial autonomy via vision
    //currentScheme.ledMode(limeLight);
    //control other various mechanisms
    //currentScheme.limeLightDrive(limeLight, smartDrive, false);
    //currentScheme.conveyorFlywheel(conveyor, flywheel);
    //currentScheme.climber(climber);
    
    //SmartDashboard.getNumber("EncoderPosition", smartDrive.getCurrentPosition());
    currentScheme.swerveDrive(drive);
    //SmartDashboard.getNumber("Gyro Position", gyro.getAngle());

  }

  /**
   * Use test mode in drivers station to charge compressor.
   */
  @Override
  public void testPeriodic() {
    //compressor.start();
    //currentScheme.climberReset(climber);
  }

  
  /**
   * Assigning port numbers to motors, solenoids, etc.
   */
  private void setDefaultProperties() {
    
    // Drive Motors
    driveLeft1 = 4;
    driveLeft2 = 5;
    driveLeft3 = 6;
    driveRight1 = 1;
    driveRight2 = 2;
    driveRight3 = 3;

    colorSpinner = 14;

    // Flywheel motors
    flywheelMotor1 = 11;
    flywheelMotor2 = 12;
    flywheelMotor3 = 8;

    // Conveyor motors
    conveyorMotor1 = 7;

    // Climber Motor Ports
    downMotorPort = 13;


    //Pneumatics
    
    drivePneu1 = 1;
    drivePneu2 = 6;

    colorSol1 = 7;
    colorSol2 = 0;

    climberSol1 = 2;
    climberSol2 = 3;

    

  }



}
