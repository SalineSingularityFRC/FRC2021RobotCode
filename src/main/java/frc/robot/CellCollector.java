package frc.robot;

import frc.controller.MotorController;
import frc.controller.motorControllers.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid;

// Same as pretty much every other mechanism class written, see comments there

/**
 * TODO: Add a pneumatic piston to this 
 */
public class CellCollector {

    Spark collectorMotor1, collectorMotor2;
    DoubleSolenoid collectorSolenoid; 

    private final double forwardSpeed = 1.00;
    private final double reverseSpeed = -1.00;

    double kP = 6e-5; 
    double kI = 0;
    double kD = 0; 
    double kIz = 0; 
    double kFF = 0.000015; 
    double kMaxOutput = 1; 
    double kMinOutput = -1;
    double maxRPM = 11000;

    /**
     * Class to control a power cell collector using a single 550 motor and Spark controller
     * @param motorPort CAN ID of the Spark motor controller the collected is plugged into
     */
    public CellCollector(int motorPort1, int solenoidForward, int solenoidReverse) {
        collectorMotor1 = new Spark(motorPort1, true, 0.0, "Cell Collector Motor1", false, false, kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput);
        collectorSolenoid = new DoubleSolenoid(solenoidForward, solenoidReverse);
    }

    public void collectorForward() {
        collectorMotor1.setVelocity(-maxRPM);
    }

    public void collectorReverse() {
        collectorMotor1.setVelocity(maxRPM);
    }

    public void collectorOff() {
        collectorMotor1.setVelocity(0.0);
    }

    public void collectorUp() {
        collectorSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void collectorDown() {
        collectorSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
}