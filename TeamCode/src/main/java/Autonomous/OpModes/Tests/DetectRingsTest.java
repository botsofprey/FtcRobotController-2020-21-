package Autonomous.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Autonomous.VisionHelperUltimateGoal;

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
        ringFinder.startDetection(); // This starts tensor flow and runs a thread that looks for the rings

        telemetry.addData("Status","Initialized");
        telemetry.addData("Rings", ringFinder.numOfSeenRings());
        telemetry.update();

        while(!opModeIsActive()){
            telemetry.addData("Status","Initialized");
            telemetry.addData("Rings", ringFinder.numOfSeenRings()); // this is how you can determine how many rings are seen
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Rings", ringFinder.numOfSeenRings());
            telemetry.update();
        }
        //ringFinder.stopDetection(); // this is how you can stop the ring detection during an op mode

        ringFinder.kill(); // always kill your objects at the end of an op mode, some objects will cause app crashes if they are not killed or stopped
    }
}
