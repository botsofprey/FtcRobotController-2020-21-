package SensorHandlers;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class UltrasonicIRSensor implements Sensor {
    ModernRoboticsI2cRangeSensor distanceSensor;
    String name;
    int id;
    double lastDistance = 0, currentDistance = 0;
    public static final double GOOD_DIST_TOLERANCE = 10;

    public UltrasonicIRSensor(ModernRoboticsI2cRangeSensor sensor, String name) {
        distanceSensor = sensor;
        this.name = name;
    }

    public UltrasonicIRSensor(ModernRoboticsI2cRangeSensor sensor, int id) {
        distanceSensor = sensor;
        this.id = id;
    }

    public UltrasonicIRSensor(ModernRoboticsI2cRangeSensor sensor, int id, String name) {
        distanceSensor = sensor;
        this.name = name;
        this.id = id;
    }

    public double getDistance() {
        lastDistance = currentDistance;
        currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        return currentDistance;
    }

    public Double getGoodDistance() {
        Double toCheck = getDistance();
        if(Math.abs(toCheck - lastDistance) < GOOD_DIST_TOLERANCE) return toCheck;
        return null;
    }

    @Override
    public void setId(int id) {
        this.id = id;
    }

    @Override
    public void setName(String n) {
        name = n;
    }

    @Override
    public Type getType() {
        return Type.ULTRASONIC_AND_IR_DISTANCE_SENSOR;
    }

    @Override
    public int getId() {
        return id;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void kill() {

    }
}
