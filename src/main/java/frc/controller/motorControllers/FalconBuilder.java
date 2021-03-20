package frc.controller.motorControllers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.fasterxml.jackson.databind.util.ArrayBuilders.BooleanBuilder;

import org.json.simple.*;
import org.json.simple.parser.*;



public class FalconBuilder {
    private TalonFXConfiguration config;
    private int canID;
    private double ramp;
    private boolean coast;
    private double kP;
    private double kI;
    private double kD;
    private boolean limitState; ///NC vs NO
    private boolean inverted;
    private FeedbackDevice sensor;
    private int canCoderID;




    public FalconBuilder(JSONObject jsonObject){
        this( (int)jsonObject.get("CanID") );
    }

    private FalconBuilder(int canID){
        this.canID = canID;
    }

    public FalconBuilder canID(int value){
        this.canID = canID;
        return this;
    }

    public FalconBuilder ramp(double value){
        this.ramp = value;
        return this;
    }

    public FalconBuilder coast(Boolean value){
        this.coast = value;
        return this;
    }

    public FalconBuilder kP(double value){
        this.kP = value;
        return this;
    }
    public FalconBuilder kI(double value){
        this.kI = value;
        return this;
    }

    public FalconBuilder kD(double value){
        this.kD = value;
        return this;
    }

    public FalconBuilder inverted(boolean value){
        this.inverted = value;
        return this;
    }

    public FalconBuilder sensor(FeedbackDevice value){
        this.sensor = value;
        return this;
    }

    public FalconBuilder limitNormalState(boolean nc){
        this.limitState = nc;
        return this;
    }

    public FalconBuilder sensorType(boolean nc){
        this.limitState = nc;
        return this;
    }

    public FalconBuilder CanCoderID(int value){
        this.canCoderID = value;
        return this;
    }

    public Falcon build(){
        Falcon talon = new Falcon(this.canID, this.ramp, this.coast);
        config.slot0.kP = this.kP;
        config.slot0.kI = this.kI;
        config.slot0.kD = this.kD;
        if( this.inverted ){
            talon.setInverted(InvertType.InvertMotorOutput);
        }
        config.primaryPID.selectedFeedbackSensor = sensor;
        config.remoteFilter0.remoteSensorDeviceID = this.canCoderID;


        talon.setConfiguration(config);
        return talon;
    }


}
