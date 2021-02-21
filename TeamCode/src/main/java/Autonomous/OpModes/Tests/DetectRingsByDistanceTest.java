package Autonomous.OpModes.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Autonomous.RingCount;
import DriveEngine.Ultimate.UltimateNavigation2;

import static Autonomous.ConfigVariables.STARTING_ROBOT_LOCATION_RIGHT;
import static DriveEngine.Ultimate.UltimateNavigation2.BACK_SENSOR;
import static DriveEngine.Ultimate.UltimateNavigation2.LEFT_SENSOR;

/**
 * Created by root on 11/20/17.
 */

/*
    An opmode to test saving images using vuforia
 */
@Autonomous(name="Detect Rings by Dist Test", group="Testers")  // @Autonomous(...) is the other common choice
//@Disabled
public class DetectRingsByDistanceTest extends LinearOpMode {

    private LinearOpMode mode;
    protected UltimateNavigation2 robot;

    @Override
    public void runOpMode() throws InterruptedException {

        this.mode = this;

        try {
            robot = new UltimateNavigation2(mode.hardwareMap, STARTING_ROBOT_LOCATION_RIGHT, STARTING_ROBOT_LOCATION_RIGHT.getHeading(), "RobotConfig/UltimateV2.json");
        } catch (Exception e) {
            e.printStackTrace();
        }

        telemetry.addData("Status","Initialized");
        telemetry.addData("Rings", distSensorCountRings());
        telemetry.addData("Bottom Dist Sensor", robot.distanceSensors[BACK_SENSOR].getDistance() + "");
        telemetry.addData("Left Dist Sensor", robot.distanceSensors[LEFT_SENSOR].getDistance() + "");
        telemetry.update();

        while(!opModeIsActive()){
            telemetry.addData("Status","Initialized");
            telemetry.addData("Rings", distSensorCountRings()); // this is how you can determine how many rings are seen
            telemetry.addData("Bottom Dist Sensor", robot.distanceSensors[BACK_SENSOR].getDistance() + "");
            telemetry.addData("Left Dist Sensor", robot.distanceSensors[LEFT_SENSOR].getDistance() + "");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Rings", distSensorCountRings());
            telemetry.addData("Bottom Dist Sensor", robot.distanceSensors[BACK_SENSOR].getDistance() + "");
            telemetry.addData("Left Dist Sensor", robot.distanceSensors[LEFT_SENSOR].getDistance() + "");
            telemetry.update();
        }
    }

    protected RingCount distSensorCountRings() {
        RingCount ringCount = RingCount.NO_RINGS;
        final int[] bottomHits = {0};
        final int[] topHits = {0};

            double bottomDistance = robot.distanceSensors[BACK_SENSOR].getDistance();
            if (bottomDistance <= 16.0) { // lower, because of 2nd wobble goal
                bottomHits[0]++;
            }
            double topDistance = robot.distanceSensors[LEFT_SENSOR].getDistance();
            if (topDistance <= 16.0) {
                topHits[0]++;
            }
            Log.d("Bottom Hits", bottomHits[0] + "");
            Log.d("Top Hits", topHits[0] + "");
        int top = topHits[0];
        int bottom = bottomHits[0];
        if (top > 0) {
            return RingCount.QUAD_STACK;
        }
        return (bottom > 0) ? RingCount.SINGLE_STACK : RingCount.NO_RINGS;
    }
}

