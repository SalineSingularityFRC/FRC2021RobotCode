package frc.controller.controlSchemes;

import frc.controller.*;
import frc.robot.ColorSensor;
import frc.robot.CellCollector;
import frc.robot.Climber;
import frc.robot.Conveyor;
import frc.robot.DrivePneumatics;
import frc.robot.Flywheel;
import frc.robot.LimeLight;
import frc.singularityDrive.SingDrive;
import frc.singularityDrive.SmartSingDrive;
import frc.singularityDrive.SwerveDrive;
import frc.singularityDrive.SingDrive.SpeedMode;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

//Uncomment to enable gyro stuff
import com.kauailabs.navx.frc.AHRS;

/**
 * 
 * Main class to control the robot
 * Use only if using swerve drive or old power-based driving with sparks
 * For using sparks with velocity, use SmartArcadeDrive
 * 
 */
public class ArcadeDrive extends ControlScheme {

    //Create all objects & a speedMode object
    XboxController driveController, armController;

    SpeedMode speedMode;

    boolean lowGear;
    boolean climberExtended;
    boolean climberDown;

    double tx, tv;
    
    final double driveSpeedConstant = 0.3;
    final double txkP = 0.022;
    final double angleDifferencekP = 0.011;
    final double endDistance = 2.0;

    AHRS gyro;

    /**
     * 
     * @param driveControllerPort Controller port the drive controller is connected to, probably 0
     * @param armControllerPort Controller port the arm controller is connect to, problably 1
     */
    public ArcadeDrive(int driveControllerPort, int armControllerPort) {
        driveController = new XboxController(driveControllerPort);
        armController = new XboxController(armControllerPort);

        /*lowGear = true;
        climberExtended = false;
        speedMode = SpeedMode.SLOW;*/

    }

    public void initGyro(){
        gyro = new AHRS(SPI.Port.kMXP);
    
    }

    /*public void colorSensor(ColorSensor colorSensor){
        if(armController.getPOVUp()) {
            colorSensor.spinColorWheelColor();
            colorSensor.resetCount(false);
        } 
        
        else if (armController.getPOVDown()) {
            colorSensor.spinColorWheelRotations(26);
            colorSensor.resetCount(false);
        } 
        
        else if (armController.getPOVLeft()) {
            colorSensor.resetCount(true);
        } 
        
        else if (armController.getPOVRight()) {
            colorSensor.spinSpeed(ColorSensor.lowspeed);
            colorSensor.resetCount(false);
        } 
        
        else {
            colorSensor.stopSpinning();
            colorSensor.resetCount(false);
        }

        if(armController.getStartButton()) {
            colorSensor.extend();
        }

        else if(armController.getBackButton()) {
            colorSensor.retract();
        }
    }*/

    public double smooshGyroAngle(double gyroAngle) {

        if (gyroAngle <= 360 && gyroAngle >= 0) {
            return gyroAngle;
        }

        gyroAngle /= 360;
        gyroAngle -= (int) gyroAngle;
        gyroAngle *= 360;

        if (gyroAngle < 0) {
            gyroAngle += 360;
        }

        return gyroAngle;

    }

    /**
     * Drives arcade drive
     * 
     */
    public void swerveDrive(SwerveDrive drive) {
        //Switch speed mode object, set to low with left bumber and high with right bumper
        /*if(driveController.getLB()) {
            speedMode = SpeedMode.SLOW;
        }

        else if(driveController.getRB()) {
            speedMode = SpeedMode.FAST;
        }*/

        //Put current speedMode on SmartDashboard
        //SmartDashboard.putString("Speed Mode", "" + speedMode);


        //Change physical pneumatic gearing with the start button (high gear) and back button (low gear).
        //This sets a boolean lowGear
        /*if(driveController.getStartButton()) {
            lowGear = false;
        }

        else if(driveController.getBackButton()) {
            lowGear = true;
        }*/

        // lowGear is used to actually set the drive pneumatics to intended value.
        /*if(lowGear) {
            pneumatics.setLow();
        }

        else {
            pneumatics.setHigh();
        }*/

        //
        //IMPORTANT
        //
        //The only line actually needed to drive - takes in control sticks, speed mode, and drives based on BasicDrive.
        drive.swerveDrive(driveController.getRS_Y(), driveController.getRS_X(), 
            driveController.getLS_X(), 
            smooshGyroAngle(gyro.getAngle()));

        // Use the d-pad/POV hat on the gamepad to drive the robot slowly in any direction for precise adjustments.
        /*if(driveController.getPOVLeft()) {
            drive.arcadeDrive(0, -0.1, 0.0, false, SpeedMode.FAST);
        }
        else if (driveController.getPOVRight()) {
            drive.arcadeDrive(0.0, 0.1, 0.0, false, SpeedMode.FAST);
        }
        else if (driveController.getPOVDown()) {
            drive.arcadeDrive(-0.1, 0.0, 0.0, false, SpeedMode.FAST);
        }
        else if (driveController.getPOVUp()) {
            drive.arcadeDrive(0.1, 0, 0.0, false, SpeedMode.FAST);
        }*/

    }

    public void smartDrive(SmartSingDrive drive, DrivePneumatics pneumatics) {

    }

    /**
     * method that controls the conveyor, collector, and flywheel as the three need to move together
     * 
     */
    public void conveyorFlywheel(Conveyor conveyor, Flywheel flywheel) {
        //Flywheel shooter controlled independantly which allows it to ramp up to speed before shooting
        //Turns on when the left trigger is pressed, then turns off when released
        if(armController.getTriggerLeft() > .5) {
            flywheel.flywheelForward();
        }

        else {
            flywheel.flywheelOff();
        }

        //When LB is pressed, the intake turns on and the conveyor is moved simultaneously to feed up to the flywheel feed
        //turns of when released

        if(armController.getRB()) {
            conveyor.conveyorForward();
        }

        else {
            conveyor.conveyorOff();
        }



        //When the right trigger is pressed, the green wheel begins feeding power cells into the ramped up intake
        //Only allow power cells to be fed when the flywheel is running
        if(armController.getTriggerRight() > .5 && armController.getTriggerRight() > .5) {
            flywheel.flywheelFeedOn();
        }

        else if(armController.getBButton()) {
            flywheel.flywheelFeedReverse();
        }

        else {
            flywheel.flywheelFeedOff();
        }
    }

    

    public void climber(Climber climber) {

        if(driveController.getBButton()) {
            climber.rachetDown();
        }

        else {
            climber.rachetOffVel();
        }

        
    }

    public void climberReset(Climber climber) {
        if(driveController.getXButton()) {
            climber.rachetReset();
        }

        else if(driveController.getBButton()) {
            climber.rachetWind();
        }

        else {
            climber.rachetOffSpeed();
        } 

        
    }


    public void limeLightDrive(LimeLight limeLight, SmartSingDrive drive, boolean runLimeLight){
        if(armController.getAButton()){
            limeLight.runLimeLight(drive);
        }
        if(runLimeLight){
            limeLight.runLimeLight(drive);
        }
    }

    /**
     * Only turns on the painfully bright Limelight LEDs when they're being used
     * @param limelight takes in Limelight object
     */
    public void ledMode(LimeLight limeLight ){

        
        if(driveController.getAButton()){
            limeLight.ledOn(limeLight);
        }
        else{
            limeLight.ledOff(limeLight);;
        }
        
    }

    @Override
    public void drive(SingDrive drive, DrivePneumatics pneumatics) {
        // TODO Auto-generated method stub

    }

    @Override
    public void colorSensor(ColorSensor colorSensor) {
        // TODO Auto-generated method stub

    }

}

/**
 * Pseudocode for Limelight targeting 
 * Use a p controller
 */