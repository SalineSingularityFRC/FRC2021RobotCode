package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.controller.motorControllers.Spark;

public class Climber{

    Spark upMotor, downMotor;
    DoubleSolenoid solenoid; 

    //speed only used when resettings
    private final double upSpeed = -0.4;

   double kP = 6e-5; 
   double kI = 0;
   double kD = 0; 
   double kIz = 0; 
   double kFF = 0.000015; 
   double kMaxOutput = 1; 
   double kMinOutput = -1;
   double maxRPM = 6000;

    public Climber(int downMotorPort, int outPort, int inPort) {
        downMotor = new Spark(downMotorPort, true, 0.0, "Climber", false, false, kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput);
        downMotor.setCoastMode(false);
        solenoid = new DoubleSolenoid(outPort, inPort);

    }

    public void setHigh() {
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void setLow() {
		solenoid.set(DoubleSolenoid.Value.kForward);
	}

	public void setOff() {
		solenoid.set(DoubleSolenoid.Value.kOff);
	}




    /*
        Old code based on using two motors, were only using one now
    public void climberToPosition(double joystickControl) {
        //11.5 volts, probably need to change that
        upMotor.setToPosition(joystickControl, upPosition, upVoltage);
    }

    public void climberUp() {
        upMotor.setSpeed(upSpeed);
    }

    public void climberHoldPosition() {
        upMotor.setSpeed(constantSpeed);
    }

    public void climberDown() {
        upMotor.setSpeed(0);
        upMotor.setCoastMode(true);
        downMotor.setSpeed(downSpeed);
    }

    public void downStop() {
        if(downMotor.isLowerLimitPressed(true)) {
            downMotor.setSpeed(0);
        }

        else {
            downMotor.setSpeed(downSpeed);
        }
    }
    */

    public void rachetDown() {
        downMotor.setVelocity(maxRPM);
    }

    public void rachetWind() {
        downMotor.setPower(0.4);
    }

    public void rachetReset() {
        downMotor.setSpeed(upSpeed);
    }

    public void rachetOffVel() {
        downMotor.setVelocity(0);
    }

    public void rachetOffSpeed() {
        downMotor.setSpeed(0);
    }
}