package Autonomous.OpModes.AnnieAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Actions.Annie.MiscellaneousActionsV2;
import Actions.Annie.StoneStackingSystemV3;
import Autonomous.AutoAlliance;
import Autonomous.ConfigVariables;
import Autonomous.ImageProcessing.SkystoneImageProcessor;
import Autonomous.Location;
import Autonomous.SkyStonePosition;
import Autonomous.VuforiaHelper;
import DriveEngine.Annie.AnnieNavigation;
import SensorHandlers.UltrasonicIRSensor;

public class AnnieAutonomous {
    private final AutoAlliance alliance;
    private final LinearOpMode mode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private double maxSpeed = 25.0;  // inches/s
    final double STONE_LOCATION_TOLERANCE = 1.5;
    final double LEAVE_STONE_QUARRY_OFFSET = 8.5;

    AnnieNavigation robot;
    StoneStackingSystemV3 sss;
    VuforiaHelper vuforia;
    SkystoneImageProcessor stoneFinder;
    UltrasonicIRSensor front;
    MiscellaneousActionsV2 otherActions;

    public AnnieAutonomous(AutoAlliance alliance, LinearOpMode mode) {
        this.alliance = alliance;
        this.mode = mode;
        this.hardwareMap = mode.hardwareMap;
        this.telemetry = new MultipleTelemetry(mode.telemetry, FtcDashboard.getInstance().getTelemetry());

        initHardwareInterfaces();
        initNavigationSystem();
        initCamera();
    }

    private void initHardwareInterfaces() {
        vuforia = new VuforiaHelper(hardwareMap);
        front = new UltrasonicIRSensor(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front"), "front");
        stoneFinder = new SkystoneImageProcessor(SkystoneImageProcessor.DESIRED_HEIGHT, SkystoneImageProcessor.DESIRED_WIDTH, .1, 1, SkystoneImageProcessor.STONE_COLOR.BLACK, (alliance == AutoAlliance.RED)? SkystoneImageProcessor.RED_TEAM:SkystoneImageProcessor.BLUE_TEAM);
        sss = new StoneStackingSystemV3(hardwareMap);
        otherActions = new MiscellaneousActionsV2(hardwareMap);
    }

    private void initNavigationSystem() {
        try {
            // TODO the y=-32 value doesn't work if we start on the foundation side
            Location startLocation = redToBlue(new Location(62, -26.5, 270.0));
            robot = new AnnieNavigation(hardwareMap, startLocation, startLocation.getHeading(), "RobotConfig/AnnieV1.json");
            disableLocationTracking();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public AnnieAutonomous reportLocation() {
        telemetry.addData("Location", robot.getRobotLocation());
        return this;
    }

    public void updateTelemetry() {
        telemetry.update();
    }

    /**
     * Convert a location defined for the RED alliance to the corresponding location for the BLUE alliance,
     * if alliance is BLUE, otherwise return unchanged.
     *
     * @param location in RED alliance orientation
     * @return location converted to BLUE alliance orientation if alliance is BLUE, else location unchanged
     */
    public Location redToBlue(Location location) {
        if (alliance == AutoAlliance.BLUE) {
            double reflectedHeading = redToBlueHeading(location.getHeading());
            return new Location(-location.getX(), location.getY(), reflectedHeading);
        } else {
            return location;
        }
    }

    public double redToBlueHeading(double heading) {
        if (alliance == AutoAlliance.BLUE) {
            return 360.0 - heading;  // reflect heading angle
        } else {
            return heading;
        }
    }

    public void disableLocationTracking() {
        robot.stopLoggingData();
    }

    public void enableLocationTracking() {
        robot.startLoggingData();
    }

    private void initCamera() {
        vuforia.getImage(SkystoneImageProcessor.DESIRED_WIDTH, SkystoneImageProcessor.DESIRED_HEIGHT);
        sleep(100);
    }

    /**
     * Continuously detect and update the stone position until the Start button is pressed
     * @return SkyStone position where 0 is closest to the wall and 2 is closest to the center
     */
    public SkyStonePosition detectSkyStonePosition() {
        // Continuously detect and update the stone position until the Start button is pressed
        int pos = SkystoneImageProcessor.UNKNOWN;
        while (!opModeIsActive()) {
            pos = stoneFinder.getSkystoneRelativePosition(vuforia.getImage(SkystoneImageProcessor.DESIRED_WIDTH, SkystoneImageProcessor.DESIRED_HEIGHT));
            String posStr = "";
            switch (pos) {
                case SkystoneImageProcessor.LEFT:
                    posStr = "LEFT";
                    break;
                case SkystoneImageProcessor.CENTER:
                    posStr = "CENTER";
                    break;
                case SkystoneImageProcessor.RIGHT:
                    posStr = "RIGHT";
                    break;
                default:
                    posStr = "NOT FOUND";
                    break;
            }
            telemetry.addData("Skystone position", posStr);
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Op Mode", opModeIsActive());
            telemetry.update();
        }

        if (pos >= 0) {
            if (alliance == AutoAlliance.RED) {
                return SkyStonePosition.values()[pos];
            } else {
                return SkyStonePosition.values()[2 - pos];
            }
        } else {
            // if detection fails, grab the stone closest to the center of the field
            return SkyStonePosition.SKY_STONE_2;
        }
    }

    public void startRun() {
        // At the start of the autonomous run, always perform these actions

        enableLocationTracking();

        if (opModeIsActive())
            sss.setCentralGripperDegree(StoneStackingSystemV3.CENTRAL_ARM_GRAB);
        sleep(500);

        if (opModeIsActive())
            sss.setCentralGripperDegree(StoneStackingSystemV3.CENTRAL_ARM_RELEASE);

        sss.liftToPosition(1);
    }

    public void grabSkyStoneFromCenterGroup(SkyStonePosition position) {
        Location targetLocation;

        double yOffsetHack = 0.0;
        switch (position) { // default should be center
            case SKY_STONE_0:
                targetLocation = ConfigVariables.SECOND_STONE_GROUP_LEFT_RED;
                break;
            case SKY_STONE_1:
                targetLocation = ConfigVariables.SECOND_STONE_GROUP_CENTER_RED;
                break;
            default:
                targetLocation = ConfigVariables.SECOND_STONE_GROUP_RIGHT_RED;
                yOffsetHack = -2.0;
                break;
        }

        // HACK to attempt to fix error in Y-position when grabbing stone
//        targetLocation = new Location(targetLocation.getX(), targetLocation.getY() + 4, targetLocation.getHeading());
        robot.driveToLocationPID(redToBlue(targetLocation), 15, mode);

//        robot.driveToLocationPID(redToBlue(new Location(35, -34.5, 270)), 20, mode);
//        robot.driveDistance(26, 0, 20, mode);
//        robot.driveDistance(Math.abs(robot.getRobotLocation().getY() - targetLocation.getY()),redToBlueHeading(-90), 20, mode);
//        Location newTargetLocation = new Location(robot.getRobotLocation().getX(), targetLocation.getY(), targetLocation.getHeading());
//        robot.driveDistanceToLocation(targetLocation, 20, mode);

        approachAndGrabStone();
    }

    public void grabSkyStoneFromWallGroup(SkyStonePosition position) {
        Location targetLocation;

        switch (position) {
            case SKY_STONE_0: // wall stone, do something interesting  -- go after stone 1
                targetLocation = redToBlue(ConfigVariables.FIRST_STONE_GROUP_CENTER_RED);
                break;
            case SKY_STONE_1:
                targetLocation = redToBlue(ConfigVariables.FIRST_STONE_GROUP_CENTER_RED);
                break;
            default:
                targetLocation = redToBlue(ConfigVariables.FIRST_STONE_GROUP_RIGHT_RED);
                break;
        }

        robot.driveToLocationPID(new Location(targetLocation.getX(), targetLocation.getY()+2, 0), maxSpeed, mode);
        robot.turnToHeading(targetLocation.getHeading(), mode);
        approachAndGrabStone();
    }

    private void approachAndGrabStone() {
        sss.liftToPosition(1);
        sleep(300);

        // wait for sensor to provide a good reading (or give up after 2 seconds)
        long startTime = System.currentTimeMillis();
        double measuredDistance = front.getDistance();
        while (opModeIsActive() && measuredDistance > 30.0 && System.currentTimeMillis() - startTime < 2000) measuredDistance = front.getDistance();

        if (measuredDistance < 2.25) {
            robot.driveDistance(1, 180, maxSpeed, mode);
        }
        measuredDistance = front.getDistance();
        while (opModeIsActive() && measuredDistance > 30.0 && System.currentTimeMillis() - startTime < 2000) measuredDistance = front.getDistance();
        if (measuredDistance >= 2.25) {
            sss.liftToPosition(0);
            robot.driveDistance(measuredDistance - 2.0, 0, 15, mode);
        } else {
            sss.liftToPosition(0);
            sleep(250);
        }

        sss.grabStoneCenter();
        sleep(300);

        double allianceFactor = alliance == AutoAlliance.RED ? 1.0 : -1.0;
//        robot.driveToLocationPID(new Location(robot.getRobotLocation().getX() + allianceFactor * LEAVE_STONE_QUARRY_OFFSET, robot.getRobotLocation().getY(), robot.getOrientation()), maxSpeed, mode);
        robot.driveDistance(LEAVE_STONE_QUARRY_OFFSET, 180, 15, mode);
        sleep(150);
    }

    public void traverseSkyBridgeSidewaysNearCenterAndContinueTo(Location continueToLocation) {
//        robot.driveToLocationPID(redToBlue(ConfigVariables.UNDER_RED_BRIDGE), maxSpeed, mode);
        // TODO add on option to driveToLocationPID to eliminate the stop/brake at the end to allow a smooth, at-speed transition to the next way point
        if (continueToLocation != null) {
            robot.driveToLocationPID(redToBlue(continueToLocation), maxSpeed+5, mode);
        }
    }

    public void traverseSkyBridgeForwardNearCenterAndContinueTo(Location continueToLocation) {
//        robot.driveToLocationPID(redToBlue(ConfigVariables.UNDER_RED_BRIDGE_0_HEADING), maxSpeed, mode);
        // TODO add on option to driveToLocationPID to eliminate the stop/brake at the end to allow a smooth, at-speed transition to the next way point
        if (continueToLocation != null) {
            robot.driveToLocationPID(redToBlue(continueToLocation), maxSpeed, mode);
        }
    }

    public void traverseSkyBridgeNearWall(Location continueToLocation) {
        throw new RuntimeException("Not implemented yet");
    }

    public void grabFoundation() {
        // NOTE: This assumes that the robot is already positioned facing the foundation and the lift is above position 0
        robot.turnController.setSp(robot.getOrientation());

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && front.getDistance() > 1.2 && System.currentTimeMillis() - startTime < 2000) {
            robot.driveOnHeadingPID(0, 17, mode);
        }
        robot.brake();

        otherActions.grabFoundation();
        sleep(750);
    }

    public void repositionFoundation() {
        double turnFactor = alliance == AutoAlliance.RED ? 1.0 : -1.0;
        robot.driveOnHeadingWithTurning(redToBlueHeading(200), 0.6, turnFactor * .25);
        sleep(1750);
        robot.brake();

        robot.driveDistance(25, AnnieNavigation.FORWARD, maxSpeed, mode);

        otherActions.releaseFoundation();
        sleep(750);
    }

    public void liftToPosition(int heightPosition) {
        sss.liftToPosition(heightPosition);
        sleep(250); // give lift time to get out of the way so subsequent distance readings can be used
    }

    public void lowerLift() {
        sss.liftToPosition(0);
    }

    public void releaseStone() {
        sss.releaseStoneCenter();
        sleep(250);
    }

    public void turnToFaceQuarry() {
        robot.turnToHeading(redToBlueHeading(270), mode);
    }

    public void turnToZero() {
        robot.turnToHeading(0, mode);
    }

    public void backupAndEstablishLocation() {
        robot.driveDistance(5,180, maxSpeed, mode);
        turnToZero();
        robot.driveToLocationPID(redToBlue(new Location(35, robot.getRobotLocation().getY(), 0)), 35, mode);
    }

    public void driveToLocation(Location location) {
        robot.driveToLocationPID(redToBlue(location), maxSpeed, mode);
    }

    public void driveForwardUntilContact() {
        long startTime = System.currentTimeMillis();
        robot.turnController.setSp(robot.getOrientation());
        while (opModeIsActive() && front.getDistance() > 1.2 && System.currentTimeMillis() - startTime < 2000) {
            robot.driveOnHeadingPID(0, 25, mode);
        }
        robot.brake();
    }

    public void stop() {
        robot.stopNavigation();
        sss.kill();
        otherActions.kill();
        front.kill();
        //VuforiaHelper.kill();     --this crashes the app...
    }

    public boolean opModeIsActive() {
        return mode.opModeIsActive();
    }

    public void sleep(long milliseconds) {
        mode.sleep(milliseconds);
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public void park() {
        robot.turnToHeading(90, 5, mode);
        otherActions.spitTape();
        sleep(800);
        otherActions.pauseTape();
    }
}
