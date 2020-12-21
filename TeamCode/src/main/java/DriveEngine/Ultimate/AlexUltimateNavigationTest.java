package DriveEngine.Ultimate;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.InputStream;
import java.util.HashMap;

import Autonomous.HeadingVector;
import Autonomous.Location;
import Autonomous.Rectangle;
import MotorControllers.JsonConfigReader;
import MotorControllers.MotorController;
import MotorControllers.PIDController;
import SensorHandlers.ImuHandler;

/**
 * Created by Alex P on 12/15/2020.
 *
 * The base class for every opmode --- it sets up our drive system and contains all its functions
 */
public class AlexUltimateNavigationTest {
//-------------------- final variables go here --------------------
    private static final int FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR = 0;
    private static final int FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR = 1;
    private static final int BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR = 2;
    private static final int BACK_LEFT_HOLONOMIC_DRIVE_MOTOR = 3;

    private static final boolean PAUSED = false;
    private static final boolean RESUMED = true;

//-------------------- private variables and functions go here --------------------
    private HardwareMap hardwareMap;
    private LinearOpMode mode;
    private Location startingLocation;
    private long startingTime;

    private MotorController[] driveBaseMotors;
    private ImuHandler imu;

    private Location currentLocation;
    private Location[] waypoints;//stores the locations that the robot should be going to
    private double[]currentInchesFromStart;
    private double movementHeading;
    private long currentTime;
    private boolean movement;

    private void initializeUsingConfigFile(String file) {
        InputStream stream = null;
        try {
            stream = hardwareMap.appContext.getAssets().open(file);
        } catch (Exception e) {
            Log.e("Drive Engine Error:", "Config file error:" + e.toString());
            throw new RuntimeException("Drive Engine Config Failed:" + e.toString());
        }
        JsonConfigReader reader = new JsonConfigReader(stream);
        String[] motorNames = { "FRONT_LEFT_MOTOR", "FRONT_RIGHT_MOTOR", "BACK_RIGHT_MOTOR", "BACK_LEFT_MOTOR" };//this allows the use of a for loop to iterate through data in the file
        try {
            for (int i = 0; i < 4; i++) {
                driveBaseMotors[i] = new MotorController(reader.getString(motorNames[i] + "_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);

                driveBaseMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                driveBaseMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if(reader.getString(motorNames[i] + "_DIRECTION").equals("REVERSE")) {
                    driveBaseMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
                }
                else if(reader.getString(motorNames[i] + "_DIRECTION").equals("FORWARD")) {
                    driveBaseMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }

            if(reader.getString("DRIVE_MOTOR_BRAKING_MODE").equals("BRAKE")) {
                for (int i = 0; i < driveBaseMotors.length; i++) {
                    driveBaseMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            else if(reader.getString("DRIVE_MOTOR_BRAKING_MODE").equals("FLOAT")) {
                for (int i = 0; i < driveBaseMotors.length; i++) {
                    driveBaseMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }
        } catch (Exception e) {
            Log.e("Drive Engine Error:", "Config file error:" + e.toString());
            throw new RuntimeException("Drive Engine Config Failed:" + e.toString());
        }
    }

    private void calculateNewLocation(double dt) {//calculates current location
        Location newLocation = new Location(0, 0, 0);
        newLocation.setHeading(imu.getOrientationOffset());
        double[] motorVelocities = new double[4];
        double[] inchesFromStart = new double[4];
        for (int i = 0; i < 4; i++) {
            inchesFromStart[i] = driveBaseMotors[i].getInchesFromStart();
        }
        for (int i = 0; i < 4; i++) {
            motorVelocities[i] = (inchesFromStart[i] - currentInchesFromStart[i]) / dt;
        }
        double frontRightMovementVector = (motorVelocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] + motorVelocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]) * 0.5;
        double frontLeftMovementVector = (motorVelocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] + motorVelocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]) * 0.5;
        double robotSpeed = Math.sqrt(frontLeftMovementVector * frontLeftMovementVector + frontRightMovementVector * frontRightMovementVector);
        double distanceTraveled = robotSpeed * dt;
        double slope = frontLeftMovementVector / frontRightMovementVector;
        double angle = Math.atan(slope) + 0.25 * Math.PI;
        double angleZero = Math.toRadians(movementHeading);
        double angleOne = angle;
        if (angleZero == angleOne) {
            newLocation.setX(currentLocation.getX() + Math.cos(angleZero) * distanceTraveled);
            newLocation.setY(currentLocation.getY() + Math.sin(angleZero) * distanceTraveled);
        }
        else {
            double radius = distanceTraveled / (angleZero - angleOne);
            newLocation.setX(currentLocation.getX() + radius * (-Math.sin(angleZero) - Math.cos(angleOne)));
            newLocation.setY(currentLocation.getY() + radius * (Math.sin(angleZero) + Math.cos(angleOne)));
        }
        movementHeading = Math.toDegrees(angleOne);
        currentInchesFromStart = inchesFromStart;
        currentLocation = newLocation;
    }

//-------------------- public variables and functions go here --------------------
    public AlexUltimateNavigationTest(HardwareMap hw, LinearOpMode op, String configFile, Location startLocation) {
        hardwareMap = hw;
        mode = op;
        startingLocation = currentLocation = startLocation;
        startingTime = currentTime = System.nanoTime();
        movement = PAUSED;
        imu.setOrientationOffset(startLocation.getHeading());
        for (int i = 0; i < 4; i++) {
            currentInchesFromStart[i] = 0;
        }
        initializeUsingConfigFile(configFile);
    }

    public void update() {//call this continuously to keep the robot moving correctly
        long newTime = System.nanoTime();
        long dt = newTime - currentTime;
        calculateNewLocation(dt);
        //todo update movement
    }

    public Location getCurrentLocation() { return currentLocation; }

    public Location getWaypoint(int index) { return waypoints[index]; }
    public void overrideWaypoint(int index, Location newWaypoint) { waypoints[index] = newWaypoint; }
    public void insertWaypoint(int index, Location newWaypoint) {//inserts a waypoint at the index
        Location newWaypoints[] = new Location[waypoints.length + 1];
        for (int i = 0; i < index; i++) {
            newWaypoints[i] = waypoints[i];
        }
        newWaypoints[index] = newWaypoint;
        for (int i = index; i < waypoints.length; i++) {
            newWaypoints[i + 1] = waypoints[i];
        }
        waypoints = newWaypoints;
    }
    public void pushBackWaypoint(Location newWaypoint) { insertWaypoint(waypoints.length, newWaypoint); }//inserts waypoint at the end
    public void deleteWaypoint(int index) {
        Location newWaypoints[] = new Location[waypoints.length + 1];
        for (int i = 0; i < index; i++) {
            newWaypoints[i] = waypoints[i];
        }
        for (int i = index; i < waypoints.length - 1; i++) {
            newWaypoints[i] = waypoints[i + 1];
        }
        waypoints = newWaypoints;
    }
    public int getWaypointLength() { return waypoints.length; }

    public void pauseRobotMovement() { movement = PAUSED; }
    public void resumeRobotMovement() { movement = RESUMED; }
}