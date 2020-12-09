package Autonomous.OpModes.Tests;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import Autonomous.ImageProcessing.SkystoneImageProcessor;
import Autonomous.VuforiaHelper;

/**
 * Created by root on 11/20/17.
 */

/*
    An opmode to test saving images using vuforia
 */
@Autonomous(name="Detect Stones Test", group="Testers")  // @Autonomous(...) is the other common choice
//@Disabled
public class DetectStonesTest extends LinearOpMode {

    SkystoneImageProcessor stoneFinder;
    @Override
    public void runOpMode() throws InterruptedException {
        int imageTaken = 0;
        /*To access the image: you need to iterate through the images of the frame object:*/
        VuforiaHelper vuforia = new VuforiaHelper(hardwareMap);
        //wait for the op mode to start, this is the time to change teams
        //initialize the image processor 
        stoneFinder = new SkystoneImageProcessor(SkystoneImageProcessor.DESIRED_HEIGHT, SkystoneImageProcessor.DESIRED_WIDTH,.1,1, SkystoneImageProcessor.STONE_COLOR.YELLOW);
        telemetry.addData("Status","Initialized");
        telemetry.update();
        waitForStart();
        //storage variables 
        Bitmap bmp;
        ArrayList<Integer> columnLocations = new ArrayList<Integer>();
        long timeStart = System.currentTimeMillis();
        //get an image
        bmp = vuforia.getImage(SkystoneImageProcessor.DESIRED_WIDTH, SkystoneImageProcessor.DESIRED_HEIGHT);
        if(bmp != null && imageTaken <= 0){
            long algorithmStart = System.currentTimeMillis();
            //find the columns
            columnLocations = stoneFinder.findColumns(bmp,true);
            telemetry.addData("Algorithm Time", "" + (System.currentTimeMillis() - algorithmStart));
            //for every column seen, print its location
            if(columnLocations != null){
                for(int i = 0; i < columnLocations.size(); i ++){
                    telemetry.addData("Column " + i, " " + columnLocations.get(i).intValue());
                }
            }
            //save every tenth image
            vuforia.saveBMP(bmp); // save edited image
            imageTaken++;
            telemetry.addData("Loop Time", "" + (System.currentTimeMillis() - timeStart));
            telemetry.addData("Image saved", "");
            telemetry.update();
        }
        else{
            Log.d("BMP","NULL!");
        }

    }







}
