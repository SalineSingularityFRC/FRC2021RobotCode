package frc.controller.autonomous;

import frc.singularityDrive.SingDrive;
import frc.robot.LimeLight;
import frc.robot.Flywheel;
import frc.robot.Conveyor;
import frc.robot.CellCollector;

public class MoveThroughTrench extends AutonControlScheme{

    public MoveThroughTrench(SingDrive drive, LimeLight limeLight, Flywheel flywheel, Conveyor conveyor) {
        super(drive, limeLight, flywheel, conveyor);
    }

    public void moveAuton(){
        //WHAT IS THIS ANGLE
        super.rotate(17.94, false);
        super.vertical(200, -0.2);
        super.rotate(135, true);
    }
}