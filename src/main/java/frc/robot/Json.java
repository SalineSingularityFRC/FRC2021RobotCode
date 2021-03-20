package frc.robot;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.rmi.NoSuchObjectException;
import java.util.Iterator;

import org.json.simple.*;
import org.json.simple.parser.*;

import frc.controller.MotorController;
import frc.controller.motorControllers.Falcon;
import frc.controller.motorControllers.Spark;

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
                    return new Falcon((int) Motor.get("CanID"));
                }

                
            }
        }

        throw new NoSuchObjectException("Motor Not Found");


    }




}
