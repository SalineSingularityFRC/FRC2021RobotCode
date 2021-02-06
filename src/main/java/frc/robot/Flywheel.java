package frc.robot;

import frc.controller.MotorController;
import frc.controller.motorControllers.Spark;


// Class to control the flywheel cell shooting mechanism on the Infinite Recharge robot

public class Flywheel {

    // Creates two generic motor controller objects to control the two motors (normal NEOs) on the flywheel
   Spark flywheel1, flywheel2, flywheel3;

    // Create two constant speed variables that run the motors forwards and backwars
    // Make these both final, so they can't be changed later, and private, so they're not influenced other places in the code
    // Set a constant speed here so it can be changed in one place when being adjusted
    private final double forwardSpeed = -0.75;
    private final double reverseSpeed = 0.75;
    private final double feedSpeed = 0.75;

    double kP = 6e-5; 
    double kI = 0;
    double kD = 0; 
    double kIz = 0; 
    double kFF = 0.000015; 
    double kMaxOutput = 1; 
    double kMinOutput = -1;
    double maxRPMFlywheel = 11000;
    double maxRPMFeed = 5700;

    // Init the flywheel object, taking int he two motors and ports and setting the flywheels to follow each other, reverse
    // Settings rampRate here to 0 so we get maximum firepower as fast as possible
    public Flywheel(int flywheel1Port, int flywheel2Port, int flywheel3Port) {
        flywheel1 = new Spark(flywheel1Port, true, 0.00, "Flywheel1", false, false, kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput);
        flywheel2 = new Spark(flywheel2Port, true, 0.00, "Flywheel2", false, false, kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput);
        flywheel3 = new Spark(flywheel3Port, true, 0.00);
        flywheel2.follow(flywheel1, true);

        flywheel1.setCoastMode(true);
        flywheel2.setCoastMode(true);
    }

    // Set the flywheels to shoot a cell forward
    public void flywheelForward() {
        flywheel1.setVelocity(-maxRPMFlywheel);
    }

    // Set the flywheels to go revserse - not sure if needed, but have it in-case
    public void flywheelReverse() {
        flywheel1.setVelocity(maxRPMFlywheel);
    }

    // Turn the flywheels off
    public void flywheelOff() {
        flywheel1.setVelocity(0.0);
    }

    public void flywheelFeedOn() {
        flywheel3.setSpeed(.50);
    }

    public void flywheelFeedReverse() {
        flywheel3.setSpeed(-.25);
    }

    public void flywheelFeedOff() {
        flywheel3.setSpeed(0.0);
    }
}