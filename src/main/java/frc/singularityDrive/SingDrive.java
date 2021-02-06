package frc.singularityDrive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.controller.MotorController;
import frc.controller.motorControllers.Spark;
import frc.controller.controlSchemes.*;
import frc.robot.LimeLight;
import frc.controller.ControlScheme;

/**
 * SingDrive (short for Singularity Drive) is the base class for all drive trains for 5066. It is
 * abstract, meaning that subclasses are forced to extend the following two methods:
 * 
 * public abstract void arcadeDrive(double vertical, double horizontal, double rotation, boolean squaredInputs, SpeedMode speedMode)
 * public abstract void tankDrive(double left, double right, boolean squaredInputs, SpeedMode speedMode)
 */
public abstract class SingDrive {


	/**
	 * Declare motor controllers on the drive train for the year here, not including those meant to strafe.
	 * 
	 * Depending on the hardware placed on the robot, use:
	 * 
	 * CANSparkMax (imported from com.revrobotics.CANSparkMax)
	 * TalonSRX (imported from com.ctre.phoenix.motorcontrol.can.TalonSRX)
	 * VictorSPX (imported from com.ctre.phoenix.motorcontrol.can.VictorSPX)
	 * 
	 * Or, add any classes that the team ends up adding.
	 * 
	 * WARNING: These objects will need to be changed if the number, type, or orientation of motor controllers changes!
	 */
	protected Spark m_leftMotor1, m_leftMotor2, m_leftMotor3, m_rightMotor1, m_rightMotor2, m_rightMotor3;

	/**
	 * When using CANSparkMax motor controllers, change DEFAULT_TO_BRUSHLESS based on the drivetrain
	 * motors, depending on hardware. For example, the neo spark motors are brushless, so set DEFAULT_TO_BRUSHLESS
	 * to true. 775Pros, however, require that DEFAULT_TO_BRUSHLESS is set to false.
	 */
	protected final static boolean DEFAULT_TO_BRUSHLESS = true;


	// The following fields change speed relative to input, and are to be set in the constructor.
	private double slowSpeedConstant, normalSpeedConstant, fastSpeedConstant;

	// Default speed constants to be used when not specified by the constructor:
	protected final static double DEFAULT_SLOW_SPEED_CONSTANT = 0.4;
	protected final static double DEFAULT_NORMAL_SPEED_CONSTANT = 0.8;
	protected final static double DEFAULT_FAST_SPEED_CONSTANT = 1.0;
	protected final static double smartMotionMaxRPM = 5700;

	//the shuit i need for limelight

	// All motor control inputs are multiplied by velocityMultiplier. Often, velocityMultiplier will be set to a speed
	// constant (listed above), or it can be set manually.
	protected double velocityMultiplier;

	/**
	 * enums can be created from SpeedMode to choose different speed controls, which will be used to determine
	 * which speedConstant velocityMultiplier should be set equal to.
	 * 
	 * To declare an enum of type SpeedMode and initialize/modify its value:
	 * 		SpeedMode exampleSpeedMode;
	 * 		exampleSpeedMode = SpeedMode.NORMAL;
	 * To access the value of exampleSpeedMode, for example, in an 'if' statement:
	 * 		if (exampleSpeedMode == SpeedMode.SLOW) {...}
	 */
	public enum SpeedMode {
        FAST,
        NORMAL,
        SLOW
	}


	// RAMP_RATE is used to limit jerks in motor output. Drive Motors starting at 0 output can ramp up to full power
	// in a time denoted by RAMP_RATE (measured in seconds). Suggested value: 0.4 (still needs testing)
	public final static double DEFAULT_RAMP_RATE = 0.5;

	// MINIMUM_THRESHOLD limits unintended drift from joystick axes. Any joystick input less than MINIMUM_THRESHOLD
	// will be set to 0 using this.threshold(double velocity). Suggested value: 0.07 (still needs testing)
	public final static double MINIMUM_THRESHOLD = 0.05;

	// DEFAULT_INPUT_POWER is the default for what a joystick input will be raised to. For example, when the value
	// is 2.0, joystick inputs will be squared when DEFAULT_INPUT_POWER is passed to this.setInputToPower().
	// Suggested values: 1.0 to 3.0 (still needs testing)
	public final static double DEFAULT_INPUT_POWER = 1.5;


	SpeedMode speedMode;

    boolean lowGear;
    boolean climberExtended;
    boolean climberDown;

    double tx, tv;
    
    //final double driveSpeedConstant = 0.3;
    final double txkP = 0.0022;
    final double angleDifferencekP = 0.011;
    final double endDistance = 2.0;
    
    
	// All subclasses must implement the following drive methods:
	
	/**
	 * Standard method for driving based on arcade, which means that one joystick axis controls translational speed and
	 * another controls rotational velocity. A third joystick axis can control strafing, if allowed by driving configuration.
	 * 
	 * @param vertical the forward/reverse constraint for robot movement
	 * @param rotation the turning constraint for robot movement
	 * @param horizontal the side-to-side constraint for robot movement
	 * @param poweredInputs pass true if inputs should be raised to the default power, thus improving sensitivity during slower driving
	 * @param speedMode controls the velocityMultiplier in order to scale motor velocity
	 */
	public abstract void arcadeDrive(double vertical, double rotation, double horizontal, boolean poweredInputs, SpeedMode speedMode);

	/**
	 * Standard method for driving based on tank, which means that one joystick controls the left drivetrain and another controls
	 * the right drivetrain. A third joystick axis can control strafing, if allowed by driving configuration.
	 * 
	 * @param left the constraint for the left drivetrain
	 * @param right the constraint for the right drivetrain
	 * @param horizontal the side-to-side constraint for robot movement: pass in 0.0, as BasicDrive cannot handle strafing movement
	 * @param poweredInputs pass true if inputs should be raised to the default power, thus improving sensitivity during slower driving
	 * @param speedMode controls the velocityMultiplier in order to scale motor velocity
	 */
	public abstract void tankDrive(double left, double right, double horizontal, boolean poweredInputs, SpeedMode speedMode);
                                                                                                                                                                                                 

	/**
	 * This is the essential constructor for SingDrive. Its parameters are motor controller ports and the
	 * driving speed constants.
	 * 
	 * The number and position of motor controllers will likely change from year to year, and likely from season to season,
	 * the the first several parameters and corresponding code will need to be edited accordingly.
	 *  
	 * @param leftMotor1 motor controller port # for one motor controller on the left drive train
	 * @param leftMotor2 motor controller port # for one motor controller on the left drive train
	 * @param rightMotor1 motor controller port # for one motor controller on the right drive train
	 * @param rightMotor2 motor controller port # for one motor controller on the right drive train
	 * 
	 * @param slowSpeedConstant suggested values: 0.2 - 0.5
	 * @param normalSpeedConstant suggested values: 0.6 - 1.0
	 * @param fastSpeedConstant suggest value: 1.0
	 * 
	 * WARNING: This method will need to be changed if the number, type, or orientation of motor controllers changes!
	 */
	public SingDrive(int leftMotor1, int leftMotor2, int leftMotor3, int rightMotor1, int rightMotor2, int rightMotor3,
	double slowSpeedConstant, double normalSpeedConstant, double fastSpeedConstant) {

		SmartDashboard.putNumber("flipper", leftMotor1);
		this.m_leftMotor1 = new Spark(leftMotor1, DEFAULT_TO_BRUSHLESS, DEFAULT_RAMP_RATE);
		//this.m_leftMotor2 = new Spark(leftMotor2, DEFAULT_TO_BRUSHLESS, DEFAULT_RAMP_RATE);
		//this.m_leftMotor3 = new Spark(leftMotor3, DEFAULT_TO_BRUSHLESS, DEFAULT_RAMP_RATE);

		// Setting one motor controller to follow another means that it will automatically set output voltage of the follower
		// controller to the value of the followee motor controller. Setting the boolean value to true inverts the s0ignal
		// in case that the motor controllers are naturally reversed.
		//this.m_leftMotor2.follow(this.m_leftMotor1, false);
		//this.m_leftMotor3.follow(this.m_leftMotor1, false);

		this.m_rightMotor1 = new Spark(rightMotor1, DEFAULT_TO_BRUSHLESS, DEFAULT_RAMP_RATE);
		this.m_rightMotor2 = new Spark(rightMotor2, DEFAULT_TO_BRUSHLESS, DEFAULT_RAMP_RATE);
		this.m_rightMotor3 = new Spark(rightMotor3, DEFAULT_TO_BRUSHLESS, DEFAULT_RAMP_RATE);
		this.m_rightMotor2.follow(this.m_rightMotor1, false);
		this.m_rightMotor3.follow(this.m_rightMotor1, false);

		// Set speed constants.
		this.slowSpeedConstant = slowSpeedConstant;
		this.normalSpeedConstant = normalSpeedConstant;
		this.fastSpeedConstant = fastSpeedConstant;
		// Set velocity multiplier to the normalSpeedConstant to begin the match.
		this.velocityMultiplier = this.normalSpeedConstant;

		// Ramp the voltage of the motor output before normal driving (can be changed for auton, or special circumstances).
		this.rampDefaultVoltage();

		// Set all drive motor controllers to coast to limit wear and tear.
		this.setDriveToCoast(true);
	}



	/**
	 * @return the velocityMultiplier, used to scale motor speed
	 */
	public double getVelocityMultiplier() {
		return this.velocityMultiplier;
	}

	/**
	 * @return the velocityMultiplier, used to scale motor speed
	 */

	 /** LEFT IS REVERSED. GOING FORWARD PUSHES THE LEFT ENCODER VALUES IN THE NEGATIVE DIRECTION */
	public double getCurrentPosition() {
		SmartDashboard.putNumber("left position1", ((Spark) this.m_leftMotor1).getCurrentPosition());
		//SmartDashboard.putNumber("left position2", ((Spark) this.m_leftMotor2).getCurrentPosition());
		//SmartDashboard.putNumber("left position3", ((Spark) this.m_leftMotor3).getCurrentPosition());
		SmartDashboard.putNumber("right position1", ((Spark) this.m_rightMotor1).getCurrentPosition());
		//SmartDashboard.putNumber("right position2", ((Spark) this.m_rightMotor2).getCurrentPosition());
		//SmartDashboard.putNumber("right position3", ((Spark) this.m_rightMotor3).getCurrentPosition());
		return  ((Spark) this.m_leftMotor1).getCurrentPosition()/-6.0 + ((Spark)this.m_rightMotor1).getCurrentPosition()/6.0 +
				((Spark) this.m_leftMotor2).getCurrentPosition()/-6.0 + ((Spark)this.m_rightMotor2).getCurrentPosition()/6.0 +
				((Spark) this.m_leftMotor3).getCurrentPosition()/-6.0 + ((Spark)this.m_rightMotor3).getCurrentPosition()/6.0;//that negative is suposed to be there trust me
		//return this.m_leftMotor1.getCurrentPosition();
	}

	/**
	 * sets the initial position of both encoders
	 */
	public void setInitialPosition(){
		((Spark) this.m_leftMotor1).setInitialPosition();
		((Spark) this.m_rightMotor1).setInitialPosition();
		((Spark) this.m_leftMotor2).setInitialPosition();
		((Spark) this.m_rightMotor2).setInitialPosition();
		((Spark) this.m_leftMotor3).setInitialPosition();
		((Spark) this.m_rightMotor3).setInitialPosition();
	}	

	/**
	 * This method sets velocityMultiplier, which is used to scale the motor speed, based on speed constants set with the constructor.
	 * @param speedMode of an enum of type SpeedMode, set to either SLOW, MEDIUM, or FAST
	 */
	protected void setVelocityMultiplierBasedOnSpeedMode(SpeedMode speedMode) {
		
		switch(speedMode) {
		case SLOW:
			this.velocityMultiplier = this.slowSpeedConstant;
			SmartDashboard.putString("DB/String 8", "Using slow speed constant");
			break;
		case NORMAL:
			this.velocityMultiplier = this.normalSpeedConstant;
			SmartDashboard.putString("DB/String 8", "Using normal speed constant");
			break;
		case FAST:
			this.velocityMultiplier = this.fastSpeedConstant;
			SmartDashboard.putString("DB/String 8", "Using fast speed constant");
			break;
		}
	}

	/**
	 * Allows other classes to set the velocityMultiplier manually (instead of through the speed constants).
	 * this.clamp (seen just below) is used to ensure a valid multiiplier.
	 * 
	 * @param velocityMultiplier motor speed is always multiplied by velocityMultiplier
	 */
	public void setVelocityMultiplier(double velocityMultiplier) {
		this.velocityMultiplier = this.clamp(velocityMultiplier);
	}
	private double clamp(double velocityMultiplier) {
		if (velocityMultiplier > 1.0) {
			return 1.0;
		} else if (velocityMultiplier < -1.0) {
			return -1.0;
		} else {
			return velocityMultiplier;
		}
	}


	/**
	 * Used to manually control the rampRate. For example, if you are preparing to stop the robot
	 * in autonomous mode, it is recommended you set rampRate to 0.0 to avoid sliding through the intended position.
	 * 
	 * @param rampRate describes how fast drive motors can ramp from 0 to full power, in seconds
	 * 
	 * WARNING: This method will need to be changed if the number, type, or orientation of motor controllers changes!
	 */
	public void rampVoltage(double rampRate) {
		this.m_leftMotor1.setRampRate(rampRate);
		this.m_leftMotor2.setRampRate(rampRate);
		this.m_leftMotor3.setRampRate(rampRate);

		this.m_rightMotor1.setRampRate(rampRate);
		this.m_rightMotor2.setRampRate(rampRate);
		this.m_rightMotor3.setRampRate(rampRate);
	}
	/**
	 * Used to return rampRate of motors to the default to avoid wear on motors (recommended for any normal driving).
	 * 
	 * WARNING: This method will need to be changed if the number, type, or orientation of motor controllers changes!
	 */
	public void rampDefaultVoltage() {
		this.m_leftMotor1.setRampRate(DEFAULT_RAMP_RATE);
		this.m_leftMotor2.setRampRate(DEFAULT_RAMP_RATE);
		this.m_leftMotor3.setRampRate(DEFAULT_RAMP_RATE);

		this.m_rightMotor1.setRampRate(DEFAULT_RAMP_RATE);
		this.m_rightMotor2.setRampRate(DEFAULT_RAMP_RATE);
		this.m_rightMotor3.setRampRate(DEFAULT_RAMP_RATE);
	}

	/**
	 * Setting motors to brake mode runs power to the motors to decellerate, but wears on the motors.
	 * 
	 * @param coast pass true to turn on coast mode (highly recommended)
	 * 
	 * WARNING: This method will need to be changed if the number, type, or orientation of motor controllers changes!
	 */
	public void setDriveToCoast(boolean coast) {
		this.m_leftMotor1.setCoastMode(coast);
		this.m_leftMotor2.setCoastMode(coast);
		this.m_leftMotor3.setCoastMode(coast);

		this.m_rightMotor1.setCoastMode(coast);
		this.m_rightMotor2.setCoastMode(coast);
		this.m_rightMotor3.setCoastMode(coast);
	}
	
	
	/**
	 * Threshold is intended to be used by subclasses to limit the drift on joystick axes.
	 * 
	 * @param joystickInput input any joystick value meant to be used as motor output, before squaring
	 * the input or scaling it with velocityMultiplier
	 * @return an adjusted joystick input
	 */
	public static double threshold(double joystickInput) {
		if (Math.abs(joystickInput) <= MINIMUM_THRESHOLD) {
			return 0;
		}
		return joystickInput;
	}

	/**
	 * Similar to squaring inputs, this method allows inputs to be set to a specific power.
	 * 
	 * @param joystickInput pass a joystick value to be set to a power
	 * @param power specify the power (recommended: DEFAULT_INPUT_POWER)
	 * @return the input raised to the power with the original sign
	 */
	public static double setInputToPower(double joystickInput, double power) {

		if (joystickInput >= 0) {
			return Math.pow(joystickInput, power);
		}

		double positiveInput = Math.abs(joystickInput);
		return joystickInput * Math.abs(Math.pow(positiveInput, power - 1));
		
	}


	public boolean limeLightDrive(LimeLight limeLight){
		return limeLight.runLimeLight(this);
    }


	 /**
     * A method to adjust the robot if there is a target
     * @param limeLight the camera
     * @param drive the singDrive
     * @param gyro the gyroscope
     * @return whether or not the robot can/should move because the robot is on angle
     */
    public boolean limeLightDrive(LimeLight limeLight, SingDrive drive, AHRS gyro, boolean squareButton, 
                                boolean offSetButton, boolean aButton ) {
        // Defining tx and tv
        // tx = X coordinate between -27 and 27
        // tv = 0 if no target found, 1 is target found
        tx = limeLight.tx.getDouble(0.0);
		tv = limeLight.tv.getDouble(0.0);
		
		SmartDashboard.putNumber("tx",tx);
		SmartDashboard.putNumber("tv",tv);
		
        if(Math.abs(tx) < 1.0){
            return false;
        }

        //SmartDashboard.putNumber("Inches", ultraIn);

        //TODO put this in teleOp
        // Declaring and instantiating buttons used for enabling LimeLight drive
        /*boolean squareButton = false, offSetButton = false;
        squareButton = driveController.getXButton();
        offSetButton = driveController.getYButton();*/
        
        // Defining and Declaring currentAngle as angle from gyro, between 0 and 360 degrees
        double currentAngle = ControlScheme.smooshGyroAngle(gyro.getAngle());
        SmartDashboard.putNumber("current angle:", currentAngle);

        // Resets gyro value to 0
        if (aButton) {
            gyro.setAngleAdjustment(0);
            gyro.setAngleAdjustment(-ControlScheme.smooshGyroAngle(gyro.getAngle()));
        }

        
        //Starts driving based on LimeLight if the button is pushed and we have a target
        if ( (squareButton || offSetButton ) && tv == 1.0/* && ultraIn > endDistance*/) {
            
            //Declaring the left and right command speeds and setting it equal to the driveSpeedConstant
            double left_command = 0.0;
            double right_command = 0.0;

            //Declares and instaniates steering_adjust, and sets it to txkP * tx
            double steering_adjust = 0.0;
            steering_adjust += txkP *  tx;
            
            //TODO fix this
            // Declare and adjust targetAngle based on currentAngle
            double targetAngle;
            if(squareButton) {
                targetAngle = ControlScheme.getSquareAngleForPort(currentAngle);
            }
            else {
                targetAngle = ControlScheme.getOffsetHatchAngle(currentAngle);
            }

            //Declaring the offset angle for turning
            double angleDifference = currentAngle - targetAngle;
            //This is an alternative angleDifference
            double secondAngleDifference = targetAngle - 360 + currentAngle; 
            //This is where we define which one we want to use. 
            //Takes which ever one is closer to 0
            if (Math.abs(secondAngleDifference) < Math.abs(angleDifference)) {
                angleDifference = secondAngleDifference;
            }
            //To remove gyro control, comment out this line:
            steering_adjust += angleDifferencekP * angleDifference;
            
            // Setting LimeLight drive for tank drive
            left_command += steering_adjust;
            right_command -= steering_adjust;
            drive.tankDrive(left_command, right_command, 0.0, false, SpeedMode.SLOW);
        }

        return true;

    }

	public static double getVelocityOutput(double input, SpeedMode speedMode) {

		//double speedModeMaxRPM;

		/*
		if (speedMode == SpeedMode.SLOW) {
			speedModeMaxRPM = smartMotionMaxRPM * .4;
		}
		else {
			speedModeMaxRPM = smartMotionMaxRPM;
		}
		*/
		
		//SmartDashboard.putNumber("SMMaxRPM", speedModeMaxRPM);

		double output = smartMotionMaxRPM * input;
		return output;
	}

	/*
	For information on driving straight certain distances, or different angles, go to:

	SalineSingularityFRC Github, FRC2018
	Branch: Offseason1
	src...team5066 > controller2018 > AutonControlScheme.java
	*/
}