package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

// Class for the drive pneumatics for pneumatic two-speed shifting gearboxes to drive the robot.

public class DrivePneumatics{

    DoubleSolenoid doubleSolenoid;

	// Defines the solenoids, takes a channel for moving the solenoid in and one for out
    public DrivePneumatics(int forwardChannel, int reverseChannel){
        doubleSolenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
    }
	
	// Set high gear for the gearbox
	public void setHigh() {
		doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
	}
	
	// Set low gear for the gearbox
	public void setLow() {
		doubleSolenoid.set(DoubleSolenoid.Value.kForward);
	}
	
	// Set solenoid to off so the solenoid can move realativly freely
	public void setOff() {
		doubleSolenoid.set(DoubleSolenoid.Value.kOff);
	}

}