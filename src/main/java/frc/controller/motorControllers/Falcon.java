package frc.controller.motorControllers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.controller.MotorController;

public class Falcon implements MotorController {

    private WPI_TalonFX talon; m

    public Falcon(int canID) {
        talon = new WPI_TalonFX(canID);
        talon.configFactoryDefault();
    }

    public void setSpeed(double speed) { //speed will be from -1.0 to 1.0
        this.talon.set(ControlMode.PercentOutput, speed);
    }

    public void setRampRate(double rampRate) {

    }

    public void setCoastMode(boolean coast) {

    }

    //Not needed for swerve, maybe needed for other systems
    /*public void follow(MotorController baseController, boolean invert) { 
        //this.talon.follow(((Falcon)baseController).getMotorController(), invert);
        this.talon.follow(baseController);
        this.talon.setInverted(InvertType.FollowMaster);

    }*/

    public void setCurrentLimit(boolean limitEnabled, int limit, int triggerThreshold, int triggerTime){
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    }

    public WPI_TalonFX getMotorController() {
        return this.talon;
    }
}
