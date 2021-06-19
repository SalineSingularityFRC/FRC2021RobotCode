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

    //protected Spark m_front_left_Wheel, m_front_right_Wheel, m_back_left_Wheel, m_back_right_Wheel;
    //protected Spark m_front_left_Angle, m_front_right_Angle, m_back_left_Angle,  m_back_right_Angle;

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


    Vector[] fl_vectors = new Vector[2];
    Vector[] fr_vectors = new Vector[2];
    Vector[] bl_vectors = new Vector[2];
    Vector[] br_vectors = new Vector[2];

    

    


    

    
    

    

    public SwerveDrive( int mFL_Angle_CAN, int mFL_Wheel_CAN, int mFR_Angle_CAN, int mFR_Wheel_CAN, int mBL_Angle_CAN, int mBL_Wheel_CAN, int mBR_Angle_CAN, int mBR_Wheel_CAN, double slowSpeedConstant, double normalSpeedConstant, double fastSpeedConstant, double rotationSpeedConstant){
        m_FL_Angle = new Falcon(mFL_Angle_CAN, .25, false);
        m_FL_Wheel = new Falcon(mFL_Wheel_CAN, .25, true);

        m_FR_Angle = new Falcon(mFR_Angle_CAN, .25, false);
        m_FR_Wheel = new Falcon(mFR_Wheel_CAN, .25, true);

        m_BL_Angle = new Falcon(mBL_Angle_CAN, .25, false);
        m_BL_Wheel = new Falcon(mBL_Wheel_CAN, .25, true);

        m_BR_Angle = new Falcon(mBR_Angle_CAN, .25, false);
        m_BR_Wheel = new Falcon(mBR_Wheel_CAN, .25, true);

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

        fl_vectors[0].set(vertical, horizontal);
        fr_vectors[0].set(vertical, horizontal);
        bl_vectors[0].set(vertical, horizontal);
        br_vectors[0].set(vertical, horizontal);

        fl_vectors[1].setAngleDist(Math.toRadians(45), nextRotation);
        fr_vectors[1].setAngleDist(Math.toRadians(45 + 90), nextRotation);
        bl_vectors[1].setAngleDist(Math.toRadians(45 + 180), nextRotation);
        br_vectors[1].setAngleDist(Math.toRadians(45 + 270), nextRotation);

        Vector flTotal = Vector.add(fl_vectors[0], fl_vectors[1]);
        Vector frTotal = Vector.add(fr_vectors[0], fr_vectors[1]);
        Vector blTotal = Vector.add(bl_vectors[0], bl_vectors[1]);
        Vector brTotal = Vector.add(br_vectors[0], br_vectors[1]);

        m_FL_Angle.setPosition( Math.toDegrees(flTotal.getAngle()) + gyroRotation + 258.398 + 0);
        m_FR_Angle.setPosition( Math.toDegrees(frTotal.getAngle()) + gyroRotation + 53.086 + 180);
        m_BL_Angle.setPosition( Math.toDegrees(blTotal.getAngle()) + gyroRotation + 123.486 + 0);
        m_BR_Angle.setPosition( Math.toDegrees(brTotal.getAngle()) + gyroRotation + 218.848 - 180);

        m_FL_Wheel.setRPMFromStick(flTotal.getDistance() * 5000);
        m_FR_Wheel.setRPMFromStick(frTotal.getDistance() * 5000);
        m_BL_Wheel.setRPMFromStick(blTotal.getDistance() * 5000);
        m_BR_Wheel.setRPMFromStick(brTotal.getDistance() * 5000);



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