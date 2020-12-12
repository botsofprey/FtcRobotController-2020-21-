package Autonomous.OpModes.Tests;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import Autonomous.VisionHelperUltimateGoal;
import Autonomous.VuforiaHelper;

/**
 * Created by root on 11/20/17.
 */

/*
    An opmode to test saving images using vuforia
 */
@Autonomous(name="Detect Rings Test", group="Testers")  // @Autonomous(...) is the other common choice
//@Disabled
public class DetectRingsTest extends LinearOpMode {

    VisionHelperUltimateGoal ringFinder;
    @Override
    public void runOpMode() throws InterruptedException {

        ringFinder = new VisionHelperUltimateGoal(VisionHelperUltimateGoal.WEBCAM, VisionHelperUltimateGoal.BOTH, hardwareMap);
        ringFinder.startDetection();

        telemetry.addData("Status","Initialized");
        telemetry.addData("Rings", ringFinder.numOfSeenRings());
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Rings", ringFinder.numOfSeenRings());
            telemetry.update();
        }
        ringFinder.kill();
    }







}
