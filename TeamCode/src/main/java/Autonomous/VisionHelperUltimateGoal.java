package Autonomous;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.LABEL_SKY_STONE;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.LABEL_STONE;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.TFOD_MODEL_ASSET;

/**
 * Created by robotics on 12/18/18.
 */

public class VisionHelperUltimateGoal extends Thread {
    public final static int SLEEP_TIME_MILLIS = 200;
    public final static int PHONE_CAMERA = 0;
    public final static int WEBCAM = 1;
    public final static int LOCATION = 0, STONE_DETECTION = 1, BOTH = 2;
    VuforiaLocalizer vuforia;
    VuforiaTrackables targetsSkyStone;
    VuforiaTrackable stoneTarget;
    VuforiaTrackable blueRearBridge;
    VuforiaTrackable redRearBridge;
    VuforiaTrackable redFrontBridge;
    VuforiaTrackable blueFrontBridge;
    VuforiaTrackable red1;
    VuforiaTrackable red2;
    VuforiaTrackable front1;
    VuforiaTrackable front2;
    VuforiaTrackable blue1;
    VuforiaTrackable blue2;
    VuforiaTrackable rear1;
    VuforiaTrackable rear2;
    private volatile TFObjectDetector tfod;
    private volatile boolean running = true, trackingLocation = false;
    private volatile boolean findingSkyStone = false;
    private volatile boolean targetVisible = false;
    private volatile OpenGLMatrix lastLocation = null;
    private volatile OpenGLMatrix lastStoneLocation = null;
    private volatile Location robotLocation = new Location(0, 0);
    private volatile Location skyStoneLocation = new Location(0, 0);
    private volatile Orientation skyStoneOrientation;
    List<VuforiaTrackable> allTrackables;
    Orientation robotOrientation;
    VectorF translation;
    VectorF stoneTranslation;
    private int mode = LOCATION;
    private RevBlinkinLedDriver LEDStripController;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;  // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (5.75f) * mmPerInch;    // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    final int CAMERA_FORWARD_DISPLACEMENT_FROM_CENTER = (int)(1.5*mmPerInch);
    final int CAMERA_VERTICAL_DISPLACEMENT_FROM_CENTER = (int)(14*mmPerInch);
    final int CAMERA_LEFT_DISPLACEMENT_FROM_CENTER = (int)(-mmPerInch);

    public VisionHelperUltimateGoal(int camera, HardwareMap hardwareMap) { this(camera, BOTH, hardwareMap); }

    public VisionHelperUltimateGoal(int camera, int mode, HardwareMap hardwareMap) {
        this.mode = mode;
        switch (mode) {
            case LOCATION:
                vuforia = VuforiaHelper.initVuforia(camera, hardwareMap);
                break;
            case STONE_DETECTION:
            case BOTH:
                initBoth(camera, hardwareMap);
                break;
            default:
                break;
        }

        LEDStripController = hardwareMap.get(RevBlinkinLedDriver.class, "LEDStripController");
        LEDStripController.resetDeviceConfigurationForOpMode();
        LEDStripController.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    private void initBoth(int camera, HardwareMap hardwareMap) {
        try {
            vuforia = VuforiaHelper.initVuforia(camera, hardwareMap);
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKY_STONE);
        } catch (Exception e) {
            Log.e("VisionHelper Error", e.toString());
            throw new RuntimeException(e);
        }
    }

    public void setLEDMode(RevBlinkinLedDriver.BlinkinPattern pattern) {
        LEDStripController.setPattern(pattern);
    }

    @Override
    public void run() {
        while (running) {
            if (trackingLocation) updateRobotLocation();
//                if (findingSkyStone) getStonesInView();
            if (findingSkyStone) getSkystonesInView();
            try {
                sleep(SLEEP_TIME_MILLIS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void startDetection() {
        loadNavigationAssets();
//        resetPositionVotes(); // TODO: update
        robotOrientation = new Orientation(EXTRINSIC, XYZ, DEGREES, 0, 0, 0, 0);
        running = true;
        findingSkyStone = true;
        this.start();
    }

    public void stopDetection() { // TODO: update
//        detectingGold = false;
//        trackingLocation = false;
        findingSkyStone = false;
        running = false;
    }

    public void startSkyStoneDetection() { findingSkyStone = true; }

    public void startTrackingLocation() {
        trackingLocation = true;
    }

    public void stopTrackingLocation() { trackingLocation = false; }

    public Recognition[] getStonesInView() {
        return tfod.getRecognitions().toArray(new Recognition[0]);
    }

    public Orientation getRobotOrientation() {
        return robotOrientation;
    }

    public double getRobotHeading() {
//        double heading = -robotOrientation.thirdAngle;
//        return heading;
        return -robotOrientation.thirdAngle;
    }

    public Location getRobotLocation() {
        if(trackingLocation) return robotLocation;
        else return null;
    }

    public Location getSkystoneLocation() {
        if(findingSkyStone) return skyStoneLocation;
        else return null;
    }
    public Orientation getSkystoneOrientation() { return skyStoneOrientation; }

    private void updateRobotLocation() {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (trackable == stoneTarget)
                continue;
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {
            translation = lastLocation.getTranslation();
            robotLocation = new Location(0, 0);
            robotLocation.update(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch);
            robotOrientation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        } else {
            robotLocation = null;
            robotOrientation = null;
        }
    }

    private void getSkystonesInView() {
//        Log.d("In", "Called laststonelocation here");
        if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
            OpenGLMatrix stoneLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
            if (stoneLocationTransform != null) {
                lastStoneLocation = stoneLocationTransform;
                stoneTranslation = lastStoneLocation.getTranslation();
                skyStoneLocation = new Location(0, 0);
                skyStoneLocation.update(stoneTranslation.get(0) / mmPerInch, stoneTranslation.get(1) / mmPerInch);
                skyStoneOrientation = Orientation.getOrientation(lastStoneLocation, EXTRINSIC, XYZ, DEGREES);
            }
        } else {
            lastStoneLocation = null;
            skyStoneLocation = null;
            skyStoneOrientation = null;
            stoneTranslation = null;
        }
    }

    private Recognition[] filterStonesOnScreen(List<Recognition> stones) {
        Recognition[] stonesArray = stones.toArray(new Recognition[0]);
        Arrays.sort(stonesArray, new Comparator<Recognition>() {
            @Override
            public int compare(Recognition r1, Recognition r2) {
                return (int) (r2.getLeft() - r1.getLeft());
            }
        });
        return stonesArray;
    }

    public void loadNavigationAssets() {
        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone"); // NOTE: the asset is titled Skystone not SkyStone... this is why I told you to copy and paste...
        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT_FROM_CENTER, CAMERA_LEFT_DISPLACEMENT_FROM_CENTER, CAMERA_VERTICAL_DISPLACEMENT_FROM_CENTER)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        90, 0, 90));

        for (VuforiaTrackable trackable : allTrackables)
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setCameraLocationOnRobot(vuforia.getCameraName(), cameraLocationOnRobot);

        targetsSkyStone.activate();
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void kill() {
        stopDetection();
        targetsSkyStone.deactivate();
        if(mode == BOTH || mode == STONE_DETECTION) tfod.shutdown();
//        VuforiaHelper.kill();
    }
}
