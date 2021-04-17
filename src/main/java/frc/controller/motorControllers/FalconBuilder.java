package frc.controller.motorControllers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;
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
    private double kF;
    private boolean limitState; ///NC vs NO
    private boolean inverted;
    private int sensor;
    private int canCoderID;
    
    
    




    public FalconBuilder(JSONObject jsonObject){
    
        this( (int) (long) jsonObject.get("CanID"), 
            (double) jsonObject.get("RampRate"), 
            (boolean) jsonObject.get("Coast"), 
            (double) ((JSONObject)jsonObject.get("PID")).get("kP"), 
            (double) ((JSONObject)jsonObject.get("PID")).get("kI"), 
            (double) ((JSONObject)jsonObject.get("PID")).get("kD"),
            (double) ((JSONObject)jsonObject.get("PID")).get("kF"),  
            (boolean) jsonObject.get("Limit"), 
            (boolean) jsonObject.get("IsMotorInverted"), 
            (int) (long) jsonObject.get("SensorType"), 
            (int) (long) jsonObject.get("CanCoderID"));
    }

    private FalconBuilder(int canID, double rampRate, boolean coast, 
        double kp, double ki, double kd, double kF, boolean limitState, boolean inverted, 
        int sensor, int canCoderID){

        this.canID = canID;
        this.ramp = rampRate;
        this.coast = coast;
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.limitState = true;
        this.inverted = inverted;
        this.sensor = sensor;
        this.canCoderID = canCoderID;
        this.kF = kF;



        
    }

    

    public FalconBuilder canID(int value){  
        this.canID = value;
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

    public FalconBuilder kF(double value){
        this.kF = value;
        return this;
    }

    public FalconBuilder inverted(boolean value){
        this.inverted = value;
        return this;
    }

    public FalconBuilder sensor(int value){
        this.sensor = value;
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

    public int cancoderToCanID(){
        CANCoder cancoder = new CANCoder(this.canCoderID);
        cancoder.setPositionToAbsolute();
        cancoder.setPosition(0.0);
        return cancoder.getDeviceID();

    }

    

    public Falcon build(){
        Falcon talon = new Falcon(this.canID, this.ramp, this.coast);
        config = new TalonFXConfiguration(); 
        // config.slot0 = new SlotConfiguration();
        config.slot0.kP = this.kP;
        config.slot0.kI = this.kI;
        config.slot0.kD = this.kD;
        config.slot0.kF = this.kF;
        if( this.inverted ){
            talon.setInverted(InvertType.InvertMotorOutput);
        }
        
        //Decode the sensor number and turn them into FeedbackDevice objects
        if(sensor == 1){
            config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        }
        else if (sensor == 11){
            config.remoteFilter0.remoteSensorDeviceID = cancoderToCanID();
            config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();
            System.out.println("Setting Sensor to CanCoder");
        }
        else if (sensor == 12){
            config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();
            config.remoteFilter1.remoteSensorDeviceID = cancoderToCanID();
        }
        else if(sensor == 14){
            config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.None.toFeedbackDevice();
        }
        else if(sensor == 15){
            config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SoftwareEmulatedSensor.toFeedbackDevice();
        }
        


        talon.setConfiguration(config);
        return talon;
    }


}
