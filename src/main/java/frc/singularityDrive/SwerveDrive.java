package frc.singularityDrive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.controller.MotorController;
import frc.controller.motorControllers.Falcon;
import frc.controller.motorControllers.Spark;
import frc.controller.controlSchemes.*;
import frc.robot.LimeLight;
import frc.controller.ControlScheme;

//Referance SingDrive.java for comments about blocks of code that are confusing to you
//Most of this is coppied and pasted from that file

public class SwerveDrive  {

    protected Spark m_front_left_Wheel, m_front_right_Wheel, m_back_left_Wheel, m_back_right_Wheel;
    protected Spark m_front_left_Angle, m_front_right_Angle, m_back_left_Angle,  m_back_right_Angle;

    protected final static boolean DEFAULT_TO_BRUSHLESS = true;

    protected final static double DEFAULT_SLOW_SPEED_CONSTANT = 0.4;
	protected final static double DEFAULT_NORMAL_SPEED_CONSTANT = 0.8;
    protected final static double DEFAULT_FAST_SPEED_CONSTANT = 1.0;
    protected final static double DEFAULT_ROTATION_SPEED_CONSTANT = 1.0;
    protected final static double smartMotionMaxRPM = 5700;
    
    protected double velocityMultiplier;

    public enum SpeedMode {
        FAST,
        NORMAL,
        SLOW
    }
    
    public final static double DEFAULT_RAMP_RATE = 0.5;
    public final static double MINIMUM_THRESHOLD = 0.05;
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

    static double initialGyroAngle = -3; // Eventually will be a starting gyro angle

    // public abstract void arcadeDrive(double vertical, double rotation, double
    // horizontal, boolean poweredInputs, SpeedMode speedMode);


    Falcon m_FL_Angle;
    Falcon m_FL_Wheel;

    Falcon m_FR_Angle;
    Falcon m_FR_Wheel;
    
    Falcon m_BL_Angle;
    Falcon m_BL_Wheel;
    
    Falcon m_BR_Angle;
    Falcon m_BR_Wheel;


    private static double distance(double x1, double y1, double x2, double y2) {
        return (Math.sqrt((Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2))));
    }

    

    private static double getAngleIfStill(double horizontal, double vertical, double xNext, double yNext, double xCurr, double yCurr, double height, double width, double angleAdd, double localToGlobal){ //LOCALTOGLOBAL IN DEGREES
        if( (horizontal <= .001 && horizontal >= -.001) && (vertical <= .001 && vertical >= -.001) ){ //check to see if robot is stationary
                //System.out.println(Math.toDegrees(Math.atan(width / height)) + angleAdd);
            return Math.toDegrees(Math.atan(width / height)) + angleAdd;
        }
        else{ //if the robot isn't stationary
            if(xNext < xCurr){
                return 180 + Math.toDegrees(Math.atan((yNext - yCurr) / (xNext - xCurr))) - localToGlobal;
            }
            else if(xNext > xCurr){ 
                return Math.toDegrees(Math.atan((yNext - yCurr) / (xNext - xCurr))) - localToGlobal;
            }
            else{ // xNext == xCurr gives atan divide by zero error
                if(yNext > yCurr){ //going up
                        return 90 - localToGlobal;
                }
                else{
                        return 270 - localToGlobal;
                }
            }
            //return Math.toDegrees(Math.atan((yNext - yCurr) / (xNext - xCurr)));
                
        }
    }
    

    private static double getDistanceIfStill(double horizontal, double vertical, double xNext, double yNext, double xCurr, double yCurr, double rotationSpeed ){
        if( (horizontal <= .001 && horizontal >= -.001) && (vertical <= .001 && vertical >= -.001) ){ //if they're basically zero
                return rotationSpeed;
        }
        else{
                return distance(xNext, yNext, xCurr, yCurr);
                //return 1;
        }
        
    }

    public SwerveDrive( int mFL_Angle_CAN, int mFL_Wheel_CAN, int mFR_Angle_CAN, int mFR_Wheel_CAN, int mBL_Angle_CAN, int mBL_Wheel_CAN, int mBR_Angle_CAN, int mBR_Wheel_CAN, double slowSpeedConstant, double normalSpeedConstant, double fastSpeedConstant, double rotationSpeedConstant){
        m_FL_Angle = new Falcon(mFL_Angle_CAN);
        m_FL_Wheel = new Falcon(mFL_Wheel_CAN);

        m_FR_Angle = new Falcon(mFR_Angle_CAN);
        m_FR_Wheel = new Falcon(mFR_Wheel_CAN);

        m_BL_Angle = new Falcon(mBL_Angle_CAN);
        m_BL_Wheel = new Falcon(mBL_Wheel_CAN);

        m_BR_Angle = new Falcon(mBR_Angle_CAN);
        m_BR_Wheel = new Falcon(mBR_Wheel_CAN);

    }


    /**
     * 
     * @author Travis Crigger
     * 
     * @param vertical              - How much we want to move forward & backward
     * @param horizontal            - How much we want to move Left and Right (Not
     *                              previosly used by Singularity)
     * @param rotation              - How much we want to rotate
     * @param (any)SpeedConstant    - arbitrary speed multiplier constant for the
     *                              speeds defined by which speed mode we are in
     * @param rotationSpeedConstant - arbitrary speed multiplier for how fast we
     *                              rotate the wheels (MAY BE REMOVED)
     * 
     */

    public void swerveDrive(double vertical, double horizontal, double nextRotation, double gyroRotation) {
        //double size = 1.0;

        double rotation = gyroRotation + nextRotation;

        double rotationSpeedConstant = DEFAULT_ROTATION_SPEED_CONSTANT;

        double robotWidth = 1;
        double robotHeight = 1;

        double halfRobotWidth = robotWidth / 2;
        double halfRobotHeight = robotHeight / 2;

        double midToOutSize = distance(halfRobotWidth, halfRobotHeight, 0, 0); // distance between robot center and any
                                          
        double mFR_Offset_Angle = Math.atan(robotHeight / robotWidth); // Angle for offsetting wheel positions along the
                                                                               // circle with radius midToOutSize
        double mBR_Offset_Angle = -mFR_Offset_Angle; // 
        double mFL_Offset_Angle = mBR_Offset_Angle + Math.PI; // 
        double mBL_Offset_Angle = -mFL_Offset_Angle; // same thing, but of the second one// of the wheels

        double mFL_XPos_Curr = (midToOutSize * Math.cos((gyroRotation) + mFL_Offset_Angle));
        double mFL_YPos_Curr = (midToOutSize * Math.sin((gyroRotation) + mFL_Offset_Angle));

        double mFR_XPos_Curr = (midToOutSize * Math.cos((gyroRotation) + mFR_Offset_Angle));
        double mFR_YPos_Curr = (midToOutSize * Math.sin((gyroRotation) + mFR_Offset_Angle));

        double mBL_XPos_Curr = (midToOutSize * Math.cos((gyroRotation) + mBL_Offset_Angle));
        double mBL_YPos_Curr = (midToOutSize * Math.sin((gyroRotation) + mBL_Offset_Angle));

        double mBR_XPos_Curr = (midToOutSize * Math.cos((gyroRotation) + mBR_Offset_Angle));
        double mBR_YPos_Curr = (midToOutSize * Math.sin((gyroRotation) + mBR_Offset_Angle));



        double mFL_XPos_Next = horizontal
                + (midToOutSize * Math.cos((rotation * rotationSpeedConstant) + mFL_Offset_Angle));
        double mFL_YPos_Next = vertical
                + (midToOutSize * Math.sin((rotation * rotationSpeedConstant) + mFL_Offset_Angle));

        double mFR_XPos_Next = horizontal
                + (midToOutSize * Math.cos((rotation * rotationSpeedConstant) + mFR_Offset_Angle));
        double mFR_YPos_Next = vertical
                + (midToOutSize * Math.sin((rotation * rotationSpeedConstant) + mFR_Offset_Angle));

        double mBL_XPos_Next = horizontal
                + (midToOutSize * Math.cos((rotation * rotationSpeedConstant) + mBL_Offset_Angle));
        double mBL_YPos_Next = vertical
                + (midToOutSize * Math.sin((rotation * rotationSpeedConstant) + mBL_Offset_Angle));

        double mBR_XPos_Next = horizontal
                + (midToOutSize * Math.cos((rotation * rotationSpeedConstant) + mBR_Offset_Angle));
        double mBR_YPos_Next = vertical
                + (midToOutSize * Math.sin((rotation * rotationSpeedConstant) + mBR_Offset_Angle));

        



        // Angle adjusting motors will set the wheels to be pointed to the angle of
        // these slopes:
        double FR_Angle = getAngleIfStill(horizontal, vertical, mFR_XPos_Next, mFR_YPos_Next, mFR_XPos_Curr, mFR_YPos_Curr, robotHeight, robotWidth, 90, gyroRotation);
        double FL_Angle = getAngleIfStill(horizontal, vertical, mFL_XPos_Next, mFL_YPos_Next, mFL_XPos_Curr, mFL_YPos_Curr, robotWidth, robotHeight, 180, gyroRotation);
        double BL_Angle = getAngleIfStill(horizontal, vertical, mBL_XPos_Next, mBL_YPos_Next, mBL_XPos_Curr,mBL_YPos_Curr, robotHeight, robotWidth, 270, gyroRotation);
        double BR_Angle = getAngleIfStill(horizontal, vertical, mBR_XPos_Next, mBR_YPos_Next, mBR_XPos_Curr, mBR_YPos_Curr, robotWidth, robotHeight, 0, gyroRotation);

        double FR_Distance = getDistanceIfStill(horizontal, vertical,  mFR_XPos_Next, mFR_YPos_Next, mFR_XPos_Curr, mFR_YPos_Curr, rotation);
        double FL_Distance = getDistanceIfStill(horizontal, vertical,  mFL_XPos_Next, mFL_YPos_Next, mFL_XPos_Curr, mFL_YPos_Curr, rotation);
        double BL_Distance = getDistanceIfStill(horizontal, vertical,  mBL_XPos_Next, mBL_YPos_Next, mBL_XPos_Curr, mBL_YPos_Curr, rotation);
        double BR_Distance = getDistanceIfStill(horizontal, vertical,  mBR_XPos_Next, mBR_YPos_Next, mBR_XPos_Curr, mBR_YPos_Curr, rotation);

        /*System.out.printf(String.format("%.3f %n", mFR_Angle));
        System.out.printf(String.format("%.3f %n", mFL_Angle));
        System.out.printf(String.format("%.3f %n", mBL_Angle));
        System.out.printf(String.format("%.3f %n %n", mBR_Angle));*/

        // System.out.printf(String.format("Value with 3 digits after decimal point %.3f
        // %n", PI)); // OUTPUTS: Value with 3 digits after decimal point 3.142

        /*System.out.printf(String.format("%.3f %n", mFR_Distance));
        System.out.printf(String.format("%.3f %n", mFL_Distance));
        System.out.printf(String.format("%.3f %n", mBL_Distance));
        System.out.printf(String.format("%.3f %n %n", mBR_Distance));*/

        // System.out.printf(String.format("%.3f %n" , mFL_XPos_Next));
        // System.out.printf(String.format("%.3f %n" , mFL_YPos_Next));

        //System.out.println(String.format("(%.3f , %.3f) %n %n", mFL_XPos_Next, mFL_YPos_Next));

        // System.out.printf(String.format("%.3f %n" , mFR_XPos_Next));
        // System.out.printf(String.format("%.3f %n", mFR_YPos_Next));

        //System.out.println(String.format("(%.3f , %.3f) %n %n", mFR_XPos_Next, mFR_YPos_Next));

        // System.out.printf(String.format("%.3f %n" , mBL_XPos_Next));
        // System.out.printf(String.format("%.3f %n" , mBL_YPos_Next));

        //System.out.println(String.format("(%.3f , %.3f) %n %n", mBL_XPos_Next, mBL_YPos_Next));

        // System.out.printf(String.format("%.3f %n" , mBR_XPos_Next));
        // System.out.printf(String.format("%.3f %n" , mBR_YPos_Next));

        //System.out.println(String.format("(%.3f , %.3f) %n %n", mBR_XPos_Next, mBR_YPos_Next));

        m_FL_Angle.setPosition(FL_Angle);
        m_FR_Angle.setPosition(FR_Angle);
        m_BL_Angle.setPosition(BL_Angle);
        m_BR_Angle.setPosition(BR_Angle);

        m_FL_Wheel.setRPMFromStick(FL_Distance / 2.0);
        m_FR_Wheel.setRPMFromStick(FR_Distance / 2.0);
        m_BL_Wheel.setRPMFromStick(BL_Distance / 2.0);
        m_BR_Wheel.setRPMFromStick(BR_Distance / 2.0);

    }

    /*public static void main(String[] args) {
        // SwerveDrive sDrive = new SwerveDrive(1, 1, 0.7853982, 1, 1, 1, 1);
        //SwerveDrive sDrive = new SwerveDrive(1, 1, 0.5235988, 1, 1, 1, 1, 1);


        //System.out.println("/n/n/n/n");
        //System.out.println(gyroWheelCompensate(10, -5));
        //System.out.println(gyroWheelCompensate(-15, 45));

    }*/

    //We think this is handled in getAngleIfStill
    //handles wheel rotation as the robot spins
    private static double gyroWheelCompensate(double gyroAngle, double targetWheelAngle) {
        double gyroDiff = gyroAngle - initialGyroAngle; // Differance between current and initial gyro angles
        return targetWheelAngle - gyroDiff;

    }

    //Figures out which direction to turn based on where we are and where we want our angle to be
    private static double angleCalculator(double currentAngle, double futureAngle) {
        if (currentAngle < 0 || currentAngle > 360 || futureAngle < 0 || futureAngle > 360) {
            return -1;
        }

        double difference = futureAngle - currentAngle;

        if (difference <= 180 && difference >= -180) {
            return difference;
        }

        return (difference > 0) ? difference - 360 : difference + 360;
    }

    //@param voltage: likely be using resistance based encoders that give a voltage between .1 and 5V
    //Shouldn't need this, not using resistance encoders
    private static double voltageToAngle(double voltage){
        if(voltage > 5 || voltage < .1 ) {
            return -1;
        }
        return (voltage - 0.1) * (360/4.9);

    }

}