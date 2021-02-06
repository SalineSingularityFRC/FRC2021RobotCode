package frc.controller.autonomous;

import frc.singularityDrive.SingDrive;
import frc.robot.LimeLight;
import frc.robot.Flywheel;
import frc.robot.Conveyor;
import frc.robot.CellCollector;

public class Lightning3  extends AutonControlScheme{

    public Lightning3 (SingDrive drive, LimeLight limeLight, Flywheel flywheel, Conveyor conveyor) {
        super(drive, limeLight, flywheel, conveyor);
    }

    public void moveAuton(){
        super.vertical(60, 0.5);//move forward
        super.rotate(90, false);//turn clockwise 90 degrees
        super.vertical(156, 0.5);//move forward
        super.rotate(90, true);//turn counterclockwise 90 degrees
        super.vertical( 40, 0.5);//move forward
        super.adjustToTarget();//adjust to the target using limelight
        super.vertical(30,0.5);//move forward
        super.shoot();//shoot power cell
    }

}