package frc.controller.autonomous;

import frc.singularityDrive.SingDrive;
import frc.robot.LimeLight;
import frc.robot.Flywheel;
import frc.robot.Conveyor;
import frc.robot.CellCollector;

public class Trench3 extends AutonControlScheme {

    public Trench3(SingDrive drive, LimeLight limeLight, Flywheel flywheel, Conveyor conveyor) {
        super(drive, limeLight, flywheel, conveyor);
    }

    public void moveAuton() {
        super.vertical(-43.32);
        super.rotate(90, true);
        super.vertical(-282.91);
        super.rotate(90, false);
        super.vertical(-43.32);
        super.rotate(17.94, true);
        super.adjustToTarget();
        super.shoot();
        //move right up to the first ball in the trench//
    }
}