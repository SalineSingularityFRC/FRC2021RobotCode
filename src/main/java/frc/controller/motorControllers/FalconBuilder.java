package frc.controller.motorControllers;

import org.json.simple.*;
import org.json.simple.parser.*;

public class FalconBuilder {
    private int canID;
    public FalconBuilder(JSONObject jsonObject){
        this( (int)jsonObject.get("CanID") );
    }

    private FalconBuilder(int canID){
        this.canID = canID;
    }

    public FalconBuilder canID(int canID){
        this.canID = canID;
        return this;
    }

    public Falcon build(){
        return new Falcon(this.canID);
    }


}
