package Autonomous.OpModes.Tests;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Autonomous.ImageProcessing.RingImageProcessor;
import Autonomous.ImageProcessing.SkystoneImageProcessor;
import Autonomous.VuforiaHelper;
import Autonomous.RingCount;

/**
 * Created by root on 11/20/17.
 */

/*
    An opmode to test saving images using vuforia
 */
@Autonomous(name="Detect Rings Picture Test", group="Testers")  // @Autonomous(...) is the other common choice
//@Disabled
public class DetectRingsPictureTest extends LinearOpMode {

    RingImageProcessor ringFinder;
    @Override
    public void runOpMode() throws InterruptedException {
        int imageTaken = 0;
        /*To access the image: you need to iterate through the images of the frame object:*/
        VuforiaHelper vuforia = new VuforiaHelper(hardwareMap);
        //wait for the op mode to start, this is the time to change teams
        //initialize the image processor 
        ringFinder = new RingImageProcessor(SkystoneImageProcessor.DESIRED_HEIGHT, SkystoneImageProcessor.DESIRED_WIDTH);
        telemetry.addData("Status","Initialized");
        telemetry.update();
        waitForStart();
        //storage variables 
        Bitmap bmp;
        RingCount ringCount = RingCount.NO_RINGS;
        long timeStart = System.currentTimeMillis();
        //get an image
        bmp = vuforia.getImage(RingImageProcessor.DESIRED_WIDTH, RingImageProcessor.DESIRED_HEIGHT);
        if(bmp != null && imageTaken <= 0){

            ringCount = ringFinder.getRingCount(bmp,true);

            vuforia.saveBMP(bmp); // save edited image
            imageTaken++;
            telemetry.addData("Ring Count", ringCount.toString());
            telemetry.addData("Image saved", "");
            telemetry.update();
        }
        else{
            Log.d("BMP","NULL!");
        }
    }
}
