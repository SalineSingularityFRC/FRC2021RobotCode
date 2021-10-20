package frc.singularityDrive;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.*;

public class Vector {

    private double vectorX = 0;
    private double vectorY = 0;

    public Vector(double x, double y){
        vectorX = x;
        vectorY = y;
    }

    public Vector(){
        vectorX = 0;
        vectorY = 0;
    }

    public double getDistance(){
        return Point2D.distance(vectorX, vectorY, 0, 0);
    }
    public double getAngle(){
        return Math.atan(vectorX/vectorY);
    }
    public double getX(){
        return this.vectorX;
    }
    public double getY(){
        return this.vectorY;
    }
    public void setX(double x){
        this.vectorX = x;
    }
    public void setY(double y){
        this.vectorY = y;
    }
    public void set(double x, double y){
        this.vectorX = x;
        this.vectorY = y;
    }
    public void setAngleDist(double angle, double dist){
        this.vectorX = Math.cos(angle) * dist;
        this.vectorY = Math.sin(angle) * dist;
    }

    public static Vector add(Vector a, Vector b){
        Vector returnVector = new Vector();
        double addedX = a.getX() + b.getX();
        double addedY = a.getY() + b.getY();
        returnVector.set(addedX, addedY);
        return returnVector;

    }

}
