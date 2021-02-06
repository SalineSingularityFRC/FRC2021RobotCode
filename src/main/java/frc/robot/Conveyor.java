package frc.robot;

import frc.controller.motorControllers.Spark;

public class Conveyor {

    Spark motor1, motor2;

    private final double forwardSpeed = -0.25;
    private final double reverseSpeed = 0.25;

    double kP = 6e-5; 
    double kI = 0;
    double kD = 0; 
    double kIz = 0; 
    double kFF = 0.000015; 
    double kMaxOutput = 1; 
    double kMinOutput = -1;
    double maxRPM = -4000;

    public Conveyor(int port1) {
        motor1 = new Spark(port1, true, 0.0, "Conveyor", false, false, kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput);
        motor1.setCurrentLimit(20);
        //motor2 = new Spark(port2, true, 0.0);
        //motor2.follow(motor1, false);
    }

    public void conveyorForward() {
        motor1.setVelocity(maxRPM);
    }

    public void conveyorReverse() {
        motor1.setVelocity(-maxRPM);
    }

    public void conveyorOff() {
        motor1.setVelocity(0.0);
    }


}