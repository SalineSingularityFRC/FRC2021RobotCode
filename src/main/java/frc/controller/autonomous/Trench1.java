package frc.controller.autonomous;

import frc.singularityDrive.SingDrive;
import frc.robot.LimeLight;
import frc.robot.Flywheel;
import frc.robot.Conveyor;
import frc.robot.CellCollector;

public class Trench1 extends AutonControlScheme {

    public Trench1(SingDrive drive, LimeLight limeLight, Flywheel flywheel, Conveyor conveyor) {
        super(drive, limeLight, flywheel, conveyor);
    }

    //The back of the robot should be facing in the direction of the side of the field with the trench//
    //the right side of the robot is facing the power port
    public void moveAuton() {
        super.vertical(-86.63);
        super.rotate(17.94, true);
        super.adjustToTarget();
        super.shoot();

        //move right up to the first ball in the trench//
    }
}