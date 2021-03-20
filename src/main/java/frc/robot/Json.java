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
            obj = parser.parse(new FileReader("../motorControllers/MotorConfig.json"));
            jsonObject = (JSONObject) obj;
        } catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

    }

    public int getChassisLength() {
        JSONObject Chassis = (JSONObject) jsonObject.get("Chassis");
        return (int) Chassis.get("Length");
    }

    public int getChassisWidth() {
        JSONObject Chassis = (JSONObject) jsonObject.get("Chassis");
        return (int) Chassis.get("Width");
    }  

    public MotorController getShooterMotor(String name) throws NoSuchObjectException {
        JSONObject Shooter = (JSONObject) jsonObject.get("Shooter");
        JSONArray Motors = (JSONArray) jsonObject.get("Motors");
        Iterator i = Motors.iterator();
        while (i.hasNext()) {
            JSONObject Motor = (JSONObject) i.next();
            if (((String) Motor.get("Name")).equalsIgnoreCase(name)) {
                JSONObject PID = (JSONObject) Motor.get("PID");
                if (((String) Motor.get("Type")).equalsIgnoreCase("Spark")) {

                    return new Spark((int) Motor.get("CanID"), true, 1, (String) Motor.get("Name"), false, false,
                            (double) PID.get("kP"), (double) PID.get("kI"), (double) PID.get("kD"), 1, 1, 1, 1);
                } 
                else if ( ((String)Motor.get("Type")).equalsIgnoreCase("Falcon") ) {
                    return new Falcon((int) Motor.get("CanID")); //TODO: fix alla this
                }

                
            }
        }

        throw new NoSuchObjectException("Motor Not Found");


    }

    public MotorController getSwerveDriveMotors(String name) throws NoSuchObjectException {
        JSONObject Swerve = (JSONObject)jsonObject.get("Swerve");
        JSONArray Motors = (JSONArray) jsonObject.get("Motors");
        JSONObject Default = (JSONObject)jsonObject.get("DefaultConfig");
        Iterator i = Motors.iterator();
        while(i.hasNext()){
            JSONObject Motor = (JSONObject) i.next();
            if( ((String)Motor.get("Name")).equalsIgnoreCase(name) ){
                JSONObject PID = (JSONObject) Motor.get("PID");
                if( ((String) Motor.get("Type")).equalsIgnoreCase("Spark") ){
                    JSONObject SparkDefault = (JSONObject)Default.get("Spark");
                    //return new Spark((int)Motor.get("CanID"), true, 1, (String)Motor.get("Name"), false, false, 
                    //    (double)PID.get("kP"), (double)PID.get("kI"), (double)PID.get("kD"), 1, 1, 1, 1);
                    SparkBuilder sb = new SparkBuilder(SparkDefault);
                    if( Motor.get("CanID") != null ) sb.portNumber( (int)Motor.get("CanID") );
                    if( Motor.get("BrushlessMotor") != null ) sb.brushlessMotor( (boolean)Motor.get("BrushlessMotor") );
                    if( Motor.get("RampRate") != null ) sb.rampRate( (double)Motor.get("RampRate") );
                    if( Motor.get("Name") != null ) sb.name( (String)Motor.get("Name") );
                    if( Motor.get("LimitSwitch") != null ) sb.limitSwitch( (boolean)Motor.get("LimitSwitch") );
                    if( Motor.get("UpperLimitSwitch") != null ) sb.upperLimitSwitch( (boolean)Motor.get("UpperLimit") );
                    if( Motor.get("kP") != null ) sb.kP( (double)Motor.get("kP") );
                    if( Motor.get("kI") != null ) sb.kI( (double)Motor.get("kI") );
                    if( Motor.get("kD") != null ) sb.kD( (double)Motor.get("kD") );
                    if( Motor.get("kIZ") != null ) sb.kIZ( (double)Motor.get("kIZ") );
                    if( Motor.get("kFF") != null ) sb.kFF( (double)Motor.get("kFF") );
                    if( Motor.get("kMinOut") != null ) sb.kMinOut( (double)Motor.get("kMinOut") );
                    if( Motor.get("kMaxOut") != null ) sb.kMaxOut( (double)Motor.get("kMaxOut") );


                }
                else if( ((String)Motor.get("Type")).equalsIgnoreCase("Falcon") ){
                    return new Falcon((int) Motor.get("CanID"));
                }
            }
        }

        throw new NoSuchObjectException("Motor Not Found");

    }




}
