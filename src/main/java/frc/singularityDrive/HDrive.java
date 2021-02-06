package frc.singularityDrive;


import frc.controller.MotorController;
import frc.controller.motorControllers.*;


/**
 * HDrive is a sibclass of SingDrive that is meant to respresent a drivetrain that uses omni-wheels
 * to strafe. Therefore, there should be motors controlling strafe wheels as well as the standard drivetrain.
 */
public class HDrive extends SingDrive {

    /**
	 * Declare motor controllers meant to strafe here.
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
	private MotorController m_middleMotor1, m_middleMotor2;


    /**
	 * This is the essential constructor for HDrive. Its parameters are motor controller ports and the
	 * driving speed constants.
	 * 
	 * The number and position of motor controllers will likely change from year to year, and possibly in-season.
	 * If so, the first several parameters and corresponding code will need to be edited accordingly.
	 *  
	 * @param leftMotor1 motor controller port # for one motor controller on the left drive train
	 * @param leftMotor2 motor controller port # for one motor controller on the left drive train
	 * @param rightMotor1 motor controller port # for one motor controller on the right drive train
	 * @param rightMotor2 motor controller port # for one motor controller on the right drive train
	 * @param middleMotor1 motor controller port # for one motor controller on the middle drive train
	 * @param middleMotor2 motor controller port # for one motor controller on the middle drive train
	 * 
	 * @param slowSpeedConstant suggested values: 0.2 - 0.5
	 * @param normalSpeedConstant suggested values: 0.6 - 1.0
	 * @param fastSpeedConstant suggest value: 1.0
	 * 
	 * WARNING: This method will need to be changed if the number, type, or orientation of motor controllers changes!
	 */
	public HDrive(int leftMotor1, int leftMotor2, int leftMotor3, int rightMotor1, int rightMotor2, int rightMotor3, int middleMotor1, int middleMotor2,
	double slowSpeedConstant, double normalSpeedConstant, double fastSpeedConstant) {

        super(leftMotor1, leftMotor2, leftMotor3, rightMotor1, rightMotor2, rightMotor3, slowSpeedConstant, normalSpeedConstant, fastSpeedConstant);
        
        this.m_middleMotor1 = new Spark(middleMotor1, DEFAULT_TO_BRUSHLESS, SingDrive.DEFAULT_RAMP_RATE);
        this.m_middleMotor2 = new Spark(middleMotor2, DEFAULT_TO_BRUSHLESS, SingDrive.DEFAULT_RAMP_RATE);
        // Setting one motor controller to follow another means that it will automatically set output voltage of the follower
		// controller to the value of the followee motor controller. Setting the boolean value to true inverts the signal
		// in case that the motor controllers are naturally reversed.
        this.m_middleMotor2.follow(this.m_middleMotor1, false);
    }
    
    /**
	 * This is the more basic constructor for HDrive. Its parameters are only motor controller ports, and they must
	 * correspond to the ports in the above constructor.
	 * 
	 * WARNING: This method will need to be changed if the number, type, or orientation of motor controllers changes!
	 */
    public HDrive(int leftMotor1, int leftMotor2, int leftMotor3, int rightMotor1, int rightMotor2, int rightMotor3, int middleMotor1, int middleMotor2) {

        this(leftMotor1, leftMotor2, leftMotor3, rightMotor1, rightMotor2, rightMotor3, middleMotor1, middleMotor2,
        DEFAULT_SLOW_SPEED_CONSTANT, DEFAULT_NORMAL_SPEED_CONSTANT, DEFAULT_FAST_SPEED_CONSTANT);
	}



    /**
	 * Standard method for driving based on arcade, which means that one joystick axis controls translational speed,
	 * another controls rotational velocity, and one controls strafing.
	 * 
	 * @param vertical the forward/reverse constraint for robot movement
	 * @param rotation the turning constraint for robot movement
	 * @param horizontal the side-to-side constraint for robot movement
	 * @param poweredInputs pass true if inputs should be raised to the default power, thus improving sensitivity during slower driving
	 * @param speedMode controls the velocityMultiplier in order to scale motor velocity
	 */
	public void arcadeDrive(double vertical, double rotation, double horizontal, boolean poweredInputs,	SpeedMode speedMode) {

        double forwardVelocity = vertical, rotationVelocity = rotation, strafeVelocity = horizontal;

        // Account for joystick drift.
        forwardVelocity = threshold(forwardVelocity);
		rotationVelocity = threshold(rotationVelocity);
		strafeVelocity = threshold(strafeVelocity);

		// If prompted, raise inputs to the default power.
		if (poweredInputs) {
			forwardVelocity = super.setInputToPower(forwardVelocity, DEFAULT_INPUT_POWER);
            rotationVelocity = super.setInputToPower(rotationVelocity, DEFAULT_INPUT_POWER);
            strafeVelocity = super.setInputToPower(strafeVelocity, DEFAULT_INPUT_POWER);
        }
        
        // Change velocityMultiplier.
        setVelocityMultiplierBasedOnSpeedMode(speedMode);

		// If a velocity > 1, we will divide by this value, maximum, in order to only set motors to power -1 to 1.
		double mainWheelMaximum = Math.max(1, Math.abs(forwardVelocity) + Math.abs(rotationVelocity));
        double hWheelMaximum = Math.max(1, Math.abs(strafeVelocity));
        
        // Drive the motors, and all subsequent motors through following.
        super.m_leftMotor1.setSpeed(super.velocityMultiplier * (forwardVelocity + rotationVelocity) / mainWheelMaximum);
		super.m_rightMotor1.setSpeed(super.velocityMultiplier * (-forwardVelocity + rotationVelocity) / mainWheelMaximum);
		this.m_middleMotor1.setSpeed(super.velocityMultiplier * strafeVelocity / hWheelMaximum);

    }
    

    /**
	 * Standard method for driving based on tank, which means that one joystick controls the left
	 * drivetrain and another controls the right drivetrain. An extra joystick axis controls strafing.
	 * 
	 * @param left the constraint for the left drivetrain
	 * @param right the constraint for the right drivetrain
	 * @param horizontal the side-to-side constraint for robot movement
	 * @param poweredInputs pass true if inputs should be raised to the default power, thus improving sensitivity during slower driving
	 * @param speedMode controls the velocityMultiplier in order to scale motor velocity
	 */
	public void tankDrive(double left, double right, double horizontal, boolean poweredInputs, SpeedMode speedMode) {
        
        double leftVelocity = left, rightVelocity = right, strafeVelocity = horizontal;
		
		// Account for joystick drift.
		leftVelocity = threshold(leftVelocity);
        rightVelocity = threshold(rightVelocity);
        strafeVelocity = threshold(strafeVelocity);

		// If prompted, raise inputs to the default power.
		if (poweredInputs) {
			leftVelocity = super.setInputToPower(leftVelocity, DEFAULT_INPUT_POWER);
            rightVelocity = super.setInputToPower(rightVelocity, DEFAULT_INPUT_POWER);
            strafeVelocity = super.setInputToPower(strafeVelocity, DEFAULT_INPUT_POWER);
		}
		
		// Change velocityMultiplier.
		setVelocityMultiplierBasedOnSpeedMode(speedMode);
		
		// If a velocity > 1, we will divide by this value, maximum, in order to only set motors to power -1 to 1.
		double leftMaximum = Math.max(1, Math.abs(leftVelocity));
        double rightMaximum = Math.max(1, Math.abs(rightVelocity));
        double strafeMaximum = Math.max(1, Math.abs(strafeVelocity));

		// Drive the motors, and all subsequent motors through following.
		super.m_leftMotor1.setSpeed(super.velocityMultiplier * leftVelocity / leftMaximum);
        super.m_rightMotor1.setSpeed(super.velocityMultiplier * rightVelocity / rightMaximum);
        this.m_middleMotor1.setSpeed(super.velocityMultiplier * strafeVelocity / strafeMaximum);
        
	}

	
}