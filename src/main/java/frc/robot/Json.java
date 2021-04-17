package frc.robot;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.rmi.NoSuchObjectException;
import java.util.Iterator;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import org.json.simple.*;
import org.json.simple.parser.*;

import frc.controller.MotorController;
import frc.controller.motorControllers.Falcon;
import frc.controller.motorControllers.FalconBuilder;
import frc.controller.motorControllers.Spark;
import frc.controller.motorControllers.SparkBuilder;
//import jdk.vm.ci.code.CodeUtil.RefMapFormatter;

public class Json {
    JSONParser parser;
    JSONObject jsonObject;

    public Json() {
        parser = new JSONParser();
        Object obj;
        try {
            obj = parser.parse(new FileReader("/home/lvuser/MotorConfig.json"));
            jsonObject = (JSONObject) obj;
        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            System.err.println(e.getMessage());
            //System.exit(1);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

    }

    public boolean isObject(String name){
        return (jsonObject.get(name) != null);
    }

    private MotorController getMotor(JSONObject Motor, JSONObject Default) {
        
        MotorController returnMotor;
        String motorType = (String) Motor.get("Type");

        if (motorType.equals("Falcon")) {
            JSONObject FalconDefault = (JSONObject) Default.get(motorType);
            FalconBuilder fb = new FalconBuilder(FalconDefault);
            if (Motor.get("CanID") != null)
                fb.canID((int) (long) Motor.get("CanID"));
            if (Motor.get("Ramp") != null)
                fb.ramp((double) Motor.get("Ramp"));
            if (Motor.get("Coast") != null)
                fb.coast((boolean) Motor.get("Coast"));
            if(Motor.get("PID") != null){
                JSONObject PID = (JSONObject) Motor.get("PID");
                if (PID.get("kP") != null )
                    fb.kP((double) PID.get("kP"));
                if (PID.get("kI") != null )
                    fb.kI((double) PID.get("kI"));
                if (PID.get("kD") != null )
                    fb.kD((double) PID.get("kD"));
                if (PID.get("kF") != null )
                    fb.kF((double) PID.get("kF"));
                
            }
            if (Motor.get("IsMotorInverted") != null)
                fb.inverted((boolean) Motor.get("IsMotorInverted"));
            if (Motor.get("SensorType") != null)
                fb.sensor((int) (long) Motor.get("SensorType"));
            if (Motor.get("LimitStateNO") != null)
                fb.sensorType((boolean) Motor.get("LimitStateNO"));
            if (Motor.get("CanCoderID") != null)
                fb.CanCoderID((int) (long) Motor.get("CanCoderID"));
            
            returnMotor = fb.build();
        } 
        else {
            JSONObject SparkDefault = (JSONObject) Default.get("Spark");
            // return new Spark((int) (long) Motor.get("CanID"), true, 1, (String)Motor.get("Name"),
            // false, false,
            // (double)PID.get("kP"), (double)PID.get("kI"), (double)PID.get("kD"), 1, 1, 1,
            // 1);
            SparkBuilder sb = new SparkBuilder(SparkDefault);
            if (Motor.get("CanID") != null)
                sb.portNumber((int) (long) Motor.get("CanID"));
            if (Motor.get("BrushlessMotor") != null)
                sb.brushlessMotor((boolean) Motor.get("BrushlessMotor"));
            if (Motor.get("RampRate") != null)
                sb.rampRate((double) Motor.get("RampRate"));
            if (Motor.get("Name") != null)
                sb.name((String) Motor.get("Name"));
            if (Motor.get("LimitSwitch") != null)
                sb.limitSwitch((boolean) Motor.get("LimitSwitch"));
            if (Motor.get("UpperLimitSwitch") != null)
                sb.upperLimitSwitch((boolean) Motor.get("UpperLimit"));
            if(Motor.get("PID") != null){
                JSONObject PID = (JSONObject) Motor.get("PID");
                if (PID.get("kP") != null )
                    sb.kP((double) PID.get("kP"));
                if (PID.get("kI") != null )
                    sb.kI((double) PID.get("kI"));
                if (PID.get("kD") != null )
                    sb.kD((double) PID.get("kD"));
                if (PID.get("kIZ") != null )
                    sb.kIZ((double) PID.get("kIZ"));
                if (PID.get("kFF") != null )
                    sb.kFF((double) PID.get("kFF"));
            }
            if (Motor.get("kMinOut") != null)
                sb.kMinOut((double) Motor.get("kMinOut"));
            if (Motor.get("kMaxOut") != null)
                sb.kMaxOut((double) Motor.get("kMaxOut"));
            returnMotor = sb.build();
        }
        return returnMotor;

    }

    public int getChassisLength() {
        JSONObject Chassis = (JSONObject) jsonObject.get("Chassis");
        return (int) (long) Chassis.get("Length");
    }

    public int getChassisWidth() {
        JSONObject Chassis = (JSONObject) jsonObject.get("Chassis");
        return (int) (long) Chassis.get("Width");
    }

    public MotorController getShooterMotor(String name) throws NoSuchObjectException {
        JSONObject Shooter = (JSONObject) jsonObject.get("Shooter");
        JSONArray Motors = (JSONArray) Shooter.get("Motors");
        JSONObject Default = (JSONObject) jsonObject.get("DefaultConfig");
        Iterator<JSONObject> i = Motors.iterator();
        while (i.hasNext()) {
            JSONObject Motor = (JSONObject) i.next();
            if (((String) Motor.get("Name")).equalsIgnoreCase(name)) {
                return getMotor(Motor, Default);
            }
            
        }

        throw new NoSuchObjectException("Motor Not Found");


    }

    public MotorController getSwerveDriveMotors(String name) throws NoSuchObjectException {
        JSONObject Swerve = (JSONObject) jsonObject.get("SwerveDrive");
        JSONArray Motors = (JSONArray) Swerve.get("Motors");
        JSONObject Default = (JSONObject) jsonObject.get("DefaultConfig");
        Iterator i = Motors.iterator();
        while (i.hasNext()) {
            JSONObject Motor = (JSONObject) i.next();
            if (((String) Motor.get("Name")).equalsIgnoreCase(name)) {
                return getMotor(Motor, Default);
            }
        }

        throw new NoSuchObjectException("Motor Not Found");

    }

    public double getLimelightKP_Heading(){
        JSONObject Limelight = (JSONObject) jsonObject.get("Limelight");
        return (double) Limelight.get("kP_Heading");
    }
    public double getLimelightKP_Distance(){
        JSONObject Limelight = (JSONObject) jsonObject.get("Limelight");
        return (double) Limelight.get("kP_Distance");
    }

}
