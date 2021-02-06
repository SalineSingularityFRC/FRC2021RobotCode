package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controller.motorControllers.Spark;

public class ColorSensor{

    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    Spark colorSpinner;
    Canifier canifier; 
    public static final double speed = SmartDashboard.getNumber("Current Color Motor Speed: ", 0.50); //subject to change
    public static final double lowspeed = SmartDashboard.getNumber("Current Color Motor Speed: ", 0.15); //subject to change
    DoubleSolenoid colorSolenoid;
    int pistonExtend;
    int pistonRetract;


    public ColorSensor(int colorSensorPort, int pistionExtend, int pistionRetract){
        colorSpinner = new Spark(colorSensorPort, true, 0.00);
        canifier = new Canifier();
        colorSolenoid = new DoubleSolenoid(7, 0);
    }
    public void setSpeed(double speed) {
        colorSpinner.setSpeed(speed);
    }

    public void spinColorWheelRotations(int numRotations) { // @param numRotations is actually the number of colors we want to see
        boolean colorData[] = canifier.getPinData();
        int color = canifier.binToDecColor(colorData);
        int count = canifier.binToDecCount(colorData);
        String byteString = canifier.byteArrayToString(colorData);
        
        SmartDashboard.putNumber("Current Color: ", color);
        SmartDashboard.putNumber("Current Count: ", count);
        SmartDashboard.putString("Byte Tranfered: ", byteString);

        if(count + 2 < numRotations) {
            colorSpinner.setSpeed(this.speed);
        }
        else if(count < numRotations) {
            colorSpinner.setSpeed(this.lowspeed);
        }
        else {
            colorSpinner.setSpeed(0.0);
            
        }
    }
    public void spinColorWheelColor(/*int targetColor*/) {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        boolean colorData[] = canifier.getPinData();
        int color = canifier.binToDecColor(colorData);
        int count = canifier.binToDecCount(colorData);
        String byteString = canifier.byteArrayToString(colorData);
        int targetColor = 0;
        
        if(gameData.length() > 0) {
            switch(gameData.charAt(0)) {
                case 'B' :
                    targetColor = 2;  // correct 0
                    break;
                case 'G' :
                    targetColor = 3; // correct 1
                    break;
                case 'R' :
                    targetColor = 0; // correct 2
                    break;
                case 'Y' :
                    targetColor = 1;  // correct 3
                    break;
                default :
                    targetColor = 0; //THIS IS ONLY FOR A TEST!!! REMOVE!
                    break;

            }
        }

        SmartDashboard.putNumber("Current Color: ", color);
        SmartDashboard.putNumber("Current Count: ", count);
        SmartDashboard.putString("Byte Tranfered: ", byteString);
        SmartDashboard.putNumber("Target Color: ", targetColor);

        //TargetColor values are shifted to accomedate for the difference between
        //robot color sensor position and game color sensor position
        //ie, they will always see different colors


        if(color != targetColor) {
            colorSpinner.setSpeed(this.lowspeed);
        }
        else {
            colorSpinner.setSpeed(0.0);
        }
    }
    public void stopSpinning() {
        colorSpinner.setSpeed(0);
    }

    public void spinSpeed(double speed) {
        colorSpinner.setSpeed(speed);
    }

    public void resetCount(boolean value) {
        canifier.resetPin(value);
    }

    public void extend() {
        colorSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    public void retract() {
        colorSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

} 