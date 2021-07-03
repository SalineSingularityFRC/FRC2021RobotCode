package frc.singularityDrive;
import frc.singularityDrive.Vector;

public class VectorTest {
    public static void main(String[] args){
        Vector vector1 = new Vector(3, 2);
        Vector vector2 = new Vector(1, 3);
        Vector newVector = Vector.add(vector1, vector2);
        System.out.println(newVector.getX() + ", " + newVector.getY());
        System.out.println(newVector.getAngle() + ", " + newVector.getDistance());
    }
    
}
