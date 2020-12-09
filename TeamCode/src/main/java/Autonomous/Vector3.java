package Autonomous;

/**
 * Author: Ethan Fisher
 * Date: 11/13/2020
 */
public class Vector3 {

    private double x;
    private double y;
    private double z;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3 distanceFromVector(Vector3 other) {
        return new Vector3(other.x - x, other.y - y, other.z - z);
    }

    public double length() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getZ() { return z; }

}
