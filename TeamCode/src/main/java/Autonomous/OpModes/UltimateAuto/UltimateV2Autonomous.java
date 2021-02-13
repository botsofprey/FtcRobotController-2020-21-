package Autonomous.OpModes.UltimateAuto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

import Actions.Ultimate.RingIntakeSystemV2;
import Actions.Ultimate.ShooterSystemV2Test;
import Actions.Ultimate.WobbleGrabberV2;
import Autonomous.AutoAlliance;
import Autonomous.Line;
import Autonomous.Location;
import Autonomous.RingCount;
import DriveEngine.Ultimate.Behavior;
import DriveEngine.Ultimate.UltimateNavigation2;

import static Autonomous.ConfigVariables.HIGH_GOAL_LOC;
import static Autonomous.ConfigVariables.LEFT_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.MIDDLE_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.PARKING_LOCATION;
import static Autonomous.ConfigVariables.POWER_SHOT_END_STRAFE;
import static Autonomous.ConfigVariables.RED_WOBBLE_GOAL_LEFT;
import static Autonomous.ConfigVariables.RED_ZONE_ONE;
import static Autonomous.ConfigVariables.RED_ZONE_THREE;
import static Autonomous.ConfigVariables.RED_ZONE_TWO;
import static Autonomous.ConfigVariables.RIGHT_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.RING_STACK_TRUE_LOC;
import static Autonomous.ConfigVariables.SHOOTING_LINE_POINT;
import static Autonomous.ConfigVariables.WOBBLE_OFFSET;
import static DriveEngine.Ultimate.UltimateNavigation2.BACK_SENSOR;
import static DriveEngine.Ultimate.UltimateNavigation2.LEFT;
import static DriveEngine.Ultimate.UltimateNavigation2.LEFT_SENSOR;
import static DriveEngine.Ultimate.UltimateNavigation2.NORTH;
import static DriveEngine.Ultimate.UltimateNavigation2.RIGHT_SENSOR;
import static DriveEngine.Ultimate.UltimateNavigation2.SOUTH;

/**
 * Author: Ethan Fisher
 * Date: 10/29/2020
 *
 * Autonomous for Ultimate Goal
 */
public class UltimateV2Autonomous {

    private final AutoAlliance alliance;
    private final LinearOpMode mode;

    protected UltimateNavigation2 robot;

    protected WobbleGrabberV2 wobbleGrabber;
    protected ShooterSystemV2Test shooter;
    protected RingIntakeSystemV2 intake;

//    protected VisionHelperUltimateGoal vision;


    protected static final double MAX_SPEED = 50;
    protected static final double HIGH_SPEED = 45;
    protected static final double MED_SPEED = 35;
    protected static final double LOW_SPEED = 20;
    protected static final double KIND_OF_SLOW_SPEED = 28;
    protected static final double MIN_SPEED = 5;
    protected static final long SLEEP_TIME = 500;
    protected static final long EMERGENCY_PARK_TIME = 0;

    public UltimateV2Autonomous(AutoAlliance alliance, Location startLocation, final LinearOpMode mode) {

        this.alliance = alliance;
        this.mode = mode;

        wobbleGrabber = new WobbleGrabberV2(mode.hardwareMap);
        shooter = new ShooterSystemV2Test(mode.hardwareMap);
        intake = new RingIntakeSystemV2(mode.hardwareMap);

        startLocation = redToBlue(startLocation);
        try {
            robot = new UltimateNavigation2(mode.hardwareMap, startLocation, startLocation.getHeading(), "RobotConfig/UltimateV2.json");
        } catch (Exception e) {
            e.printStackTrace();
        }

//        vision = new VisionHelperUltimateGoal(VisionHelperUltimateGoal.WEBCAM, mode.hardwareMap);
    }

    // converts red to blue. If it is red, nothing happens
    public Location redToBlue(Location location) {
        if (alliance == AutoAlliance.BLUE)
            return new Location(-location.getX(), location.getY(), 360 - location.getHeading());
        return location;
    }

    public void kill() {
        robot.stopNavigation();
        shooter.kill();
        intake.kill();
        wobbleGrabber.kill();
//        vision.kill();

        mode.telemetry.addData("Robot", "Stopped");
        mode.telemetry.update();
    }

    // AUTONOMOUS FUNCTIONS HERE
    protected void init() {
        shooter.setIndexLeft();
        intake.intakeServoIn();
        // set initial servo positions
    }

    // drives from current location to where power shots must be performed
    // performs power shots from right to left
    protected void performPowerShots(LinearOpMode mode, RingCount ringCount, double runtime) {
        if(mode.opModeIsActive()) { // if the time remaining is more than the required action time, perform it
            shooter.setPowerShotRPM();

            // This is a way of shooting power shots with one location and different headings
            robot.driveToLocationPID(SHOOTING_LINE_POINT, KIND_OF_SLOW_SPEED, mode);
//            robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, 1, mode);
//            powerShotIndex();
//            robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING, 1, mode);
//            powerShotIndex();
//            robot.turnToHeading(LEFT_POWER_SHOT_HEADING, 1, mode);
//            powerShotIndex();

            // This is a way of shooting power shots with strafing
//            robot.driveToLocationPID(new Location(robot.getRobotLocation().getX(), SHOOTING_LINE_POINT.getY()), KIND_OF_SLOW_SPEED, mode);

            // strafe across the field shooting power shots
//            robot.driveToLocationPID(POWER_SHOT_END_STRAFE, MIN_SPEED, mode, new Behavior() {
//                private int shotCount = 0;
//
//                public boolean doBehavior() {
//                    double d = robot.distanceSensors[RIGHT_SENSOR].getDistance();
//                    Log.d("RightDistance", d + "");
//                    if (shotCount == 0 && d >= 27) {
//                        shooter.setIndexRight();
//                        Log.d("PowerShot", "1");
//                        mode.telemetry.addData("Powershot", "1");
//                        shotCount++;
//                    }
//                    else if (shotCount == 1 && d >= 34) {
//                        shooter.setIndexLeft();
//                        Log.d("PowerShot", "2");
//                        mode.telemetry.addData("Powershot", "2");
//                        shotCount++;
//                    }
//                    else if (shotCount == 2 && d >= 40) {
//                        shooter.setIndexRight();
//                        Log.d("PowerShot", "3");
//                        mode.telemetry.addData("Powershot", "3");
//                        shotCount++;
//                    }
//                    mode.telemetry.update();
//
//                    return true; // continue to drive, false would mean stop immediately
//                }
//            });


            // This is a really gross way of shooting power shots with each path using slightly different headings
            switch(ringCount){
                case NO_RINGS:
                    robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, 1,  mode); // turn to heading for first power shot and shoot
                    powerShotIndex();


                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 300);
                    robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING - 0.5, 1, mode);
                    powerShotIndex();

                    shooter.setPowerShotRPM();
                    robot.turnToHeading(LEFT_POWER_SHOT_HEADING, 1, mode); // turn to heading for third power shot and shoot
                    powerShotIndex();
                    break;
                case SINGLE_STACK:
                    robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, 1,  mode); // turn to heading for first power shot and shoot
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 300);
                    robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING, 1, mode);
                    powerShotIndex();

                    shooter.setPowerShotRPM();
                    robot.turnToHeading(LEFT_POWER_SHOT_HEADING, 1, mode); // turn to heading for third power shot and shoot
                    powerShotIndex();
                    break;
                case QUAD_STACK:
                    robot.turnToHeading(RIGHT_POWER_SHOT_HEADING + 3.2, 1,  mode); // turn to heading for first power shot and shoot
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 300);
                    robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING + 3.0, 1, mode);
                    powerShotIndex();

                    shooter.setPowerShotRPM();
                    robot.turnToHeading(LEFT_POWER_SHOT_HEADING + 2.5, 1, mode); // turn to heading for third power shot and shoot
                    powerShotIndex();
                    break;

            }

//            if(ringCount == RingCount.NO_RINGS) {
//                shooter.setShooterMotorPower(MIDDLE_POWER_SHOT_POWER);
//                robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING-0.5, 1, mode);
//            } else {
//                shooter.setShooterMotorPower(MIDDLE_POWER_SHOT_POWER);
//                robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING, 1, mode); // turn to heading for second power shot and shoot
//            }
//            powerShotIndex();


//            shooter.setPowerShotPower();
//            robot.turnToHeading(LEFT_POWER_SHOT_HEADING, 1, mode); // turn to heading for third power shot and shoot
//            powerShotIndex();

            shooter.pauseShooter();
        }
    }

    // drives to the correct wobble goal delivery zone from the current robot location
    protected void deliverWobbleGoal(LinearOpMode mode, RingCount ringCount, double runtime, int wobbleNum) {
        if(mode.opModeIsActive()) {
            wobbleGrabber.setLiftAngleSlow();
            robot.turnToHeading(NORTH, mode);
            Location targetLocation = RED_ZONE_ONE;
            switch(ringCount){
                case SINGLE_STACK:
                    targetLocation = RED_ZONE_TWO;
                    break;
                case QUAD_STACK:
                    targetLocation = RED_ZONE_THREE;
                    break;
            }
            if(wobbleNum == 1) {
                if (ringCount == RingCount.SINGLE_STACK) {
                    robot.driveDistance(RED_ZONE_ONE.getY() - robot.getRobotLocation().getY(), NORTH, MED_SPEED, mode);
                    robot.driveToLocationPID(targetLocation, MED_SPEED, mode);
                } else if(ringCount == RingCount.QUAD_STACK){
//                    robot.driveToLocationPID(targetLocation, MED_SPEED, mode);
                    robot.driveDistance(targetLocation.getY() - robot.getRobotLocation().getY(), NORTH, MED_SPEED, mode);
                } else {
                    robot.driveDistance(targetLocation.getY() - robot.getRobotLocation().getY() - 3, NORTH, MED_SPEED, mode);
                }
            }
            else { // Place second wobble goal slightly off from the location of the first to avoid collision
                Location offsetTarget = new Location(targetLocation.getX(), targetLocation.getY() - WOBBLE_OFFSET);
                if(ringCount == RingCount.NO_RINGS) {
                    offsetTarget = new Location(targetLocation.getX(), targetLocation.getY() - WOBBLE_OFFSET + 4.5);
                }
                robot.driveToLocationPID(offsetTarget, MED_SPEED, mode);
                while(mode.opModeIsActive() && wobbleGrabber.armIsBusy());
            }

            shooter.setPowerShotRPM(); // spin up motor to expected power shot rpm

            wobbleGrabber.releaseWobble();
            wobbleGrabber.setInitAngle();
        }
    }

    // reorients and drives to intake the extra rings but only if there are extra rings
    protected void intakeExtraRings(LinearOpMode mode, RingCount ringCount, double runtime) {
        if(mode.opModeIsActive() && ringCount != RingCount.NO_RINGS) {
            intake.intake();
            shooter.setHighGoalRPM();
//            robot.driveToLocationPID(RING_STACK_START_POINT, HIGH_SPEED, mode);
//            robot.turnToHeading(UltimateNavigation2.EAST, mode);
            turnToRingStack(ringCount);
            robot.driveDistance(22, NORTH, KIND_OF_SLOW_SPEED, mode);
            robot.turnToHeading(NORTH, mode);
        }
    }

    // drives to the second wobble goal and grabs it
    protected void obtainSecondWobbleGoal(LinearOpMode mode, RingCount ringCount, double runtime) {
        if(mode.opModeIsActive()) {
            if(ringCount == RingCount.NO_RINGS) {
                wobbleGrabber.setGrabAndDropAngle();
                wobbleGrabber.releaseWobble(); // Open claw
                robot.turnToHeading(UltimateNavigation2.SOUTH, mode);
                robot.driveToLocationPID(new Location(RED_WOBBLE_GOAL_LEFT.getX(), robot.getRobotLocation().getY(), SOUTH), HIGH_SPEED, mode);
                robot.driveDistance(RED_WOBBLE_GOAL_LEFT.getY() - robot.getRobotLocation().getY(), NORTH, MED_SPEED, mode);
//                robot.driveToLocationPID(RED_WOBBLE_GOAL_LEFT, MED_SPEED, 1, mode);
//            robot.driveOnHeading(UltimateNavigation2.NORTH, LOW_SPEED); // Drive forwards, slowly
//            while(!wobbleGrabber.sensor.isPressed() && mode.opModeIsActive());
//            robot.brake(); // Once sensor is pressed, stop
                wobbleGrabber.setClawGrabAngle(); // Close claw
                mode.sleep(SLEEP_TIME);
                wobbleGrabber.setInitAngle(); // Lift arm back up
//                robot.driveToLocationPID(new Location(RED_WOBBLE_GOAL_LEFT.getX(), RED_WOBBLE_GOAL_LEFT.getY() - 12.5), HIGH_SPEED, mode); // any particular reason we do this move?
//            robot.turnToHeading(UltimateNavigation2.NORTH, mode);
            } else {
                wobbleGrabber.setGrabAndDropAngle();
                wobbleGrabber.releaseWobble(); // Open claw
                robot.turnToHeading(UltimateNavigation2.SOUTH, mode);
                robot.driveToLocationPID(new Location(RED_WOBBLE_GOAL_LEFT.getX(), robot.getRobotLocation().getY(), SOUTH), HIGH_SPEED, mode);
                robot.driveDistance(RED_WOBBLE_GOAL_LEFT.getY() - robot.getRobotLocation().getY(), NORTH, MED_SPEED, mode);
//            robot.driveOnHeading(UltimateNavigation2.NORTH, LOW_SPEED); // Drive forwards, slowly
//            while(!wobbleGrabber.sensor.isPressed() && mode.opModeIsActive());
//            robot.brake(); // Once sensor is pressed, stop
                wobbleGrabber.setClawGrabAngle(); // Close claw
                mode.sleep(SLEEP_TIME);
                wobbleGrabber.setInitAngle(); // Lift arm back up
                // We need to drive forwards a bit because otherwise we are going to hit the ring stack when turning
                robot.driveToLocationPID(new Location(RED_WOBBLE_GOAL_LEFT.getX(), RED_WOBBLE_GOAL_LEFT.getY() - 12.5), HIGH_SPEED, mode); // any particular reason we do this move?
//            robot.turnToHeading(UltimateNavigation2.NORTH, mode);
            }
        }
    }

    // drives to the correct position to shoot the extra rings then shoots them if there are extra rings
    protected void shootExtraRings(LinearOpMode mode, RingCount ringCount, double runtime) {
        if(mode.opModeIsActive() && ringCount != RingCount.NO_RINGS) {
            shooter.setShooterMotorRPM(ShooterSystemV2Test.HIGH_GOAL_RPM + 150);
            turnToHighGoalFromRings(robot.getRobotLocation());

                // shoot rings
                if(ringCount == RingCount.SINGLE_STACK) { // if there is only one extra ring, only index once
                    for(int i = 0; i < 2; i++) {
                        indexShooter();
                    }
                }
                else { // otherwise (there are four rings), index three times
                    for(int i = 0; i < 3; i++) {
                        indexShooter();
                    }
                }
                intake.pauseIntake();
        }
    }

    protected void indexShooter(){
        shooter.setIndexLeft();
        mode.sleep(SLEEP_TIME);
        shooter.setIndexRight();
        mode.sleep(SLEEP_TIME *(2/3));
    }

    protected void powerShotIndex(){
//        mode.sleep(SLEEP_TIME/3);
        shooter.setIndexLeft();
        mode.sleep(SLEEP_TIME);
        shooter.setIndexRight();
        mode.sleep(SLEEP_TIME/3);
    }


    protected void dropIntakeAndWobble(LinearOpMode mode) {
        if(mode.opModeIsActive()){
            wobbleGrabber.setClawGrabAngle();
            mode.sleep(SLEEP_TIME/2);
            intake.intakeServoOut();
            shooter.spinUp();
            wobbleGrabber.setGrabAndDropAngle();
        }
    }

    protected RingCount driveToRingStack(LinearOpMode mode){
        final List<Double> bottomDistances = new ArrayList<>();
        final List<Double> topDistances = new ArrayList<>();

        if(mode.opModeIsActive()) {
//            robot.driveDistance(37.6, NORTH, MED_SPEED, mode);
            intake.intake();
            robot.driveDistance(41.0, NORTH, KIND_OF_SLOW_SPEED, mode, new Behavior() {
                @Override
                public boolean doBehavior() {
                    double bottomDistance = robot.distanceSensors[BACK_SENSOR].getDistance();
                    if (bottomDistance <= 16.0) { // lower, because of 2nd wobble goal
                        bottomDistances.add(bottomDistance);
                    }
                    double topDistance = robot.distanceSensors[LEFT_SENSOR].getDistance();
                    if (topDistance <= 16.0) {
                        topDistances.add(topDistance);
                    }
                    Log.d("Bottom Hits", bottomDistances.size() + "");
                    Log.d("Top Hits", topDistances.size() + "");
                    return true;
                }
            });
            intake.pauseIntake();
        }

        RingCount ringCount = RingCount.NO_RINGS;
        if (bottomDistances.size() >= 1) {
            if (topDistances.size() >= 1) {
                ringCount = RingCount.QUAD_STACK;
            }
            else {
                ringCount = RingCount.SINGLE_STACK;
            }
        }
        return ringCount;
    }

    protected RingCount distSensorCountRings(){
        RingCount ringCount = RingCount.NO_RINGS;
        double bottomCount = 0.0;
        double topCount = 0.0;
        double iterations = 10.0;

        for(int i = 0; i < iterations; i++){
            if(robot.distanceSensors[BACK_SENSOR].getDistance() <= 18.0){
                bottomCount++;
            }
            Log.d("Bottom Dist Sensor", robot.distanceSensors[BACK_SENSOR].getDistance() + "");
            Log.d("Bottom Count", bottomCount + "");

            if (robot.distanceSensors[LEFT_SENSOR].getDistance() <= 18.0){
                topCount++;
            }
            Log.d("Top Dist Sensor", robot.distanceSensors[LEFT_SENSOR].getDistance() + "");
            Log.d("Top Count", topCount + "");
        }

        if (bottomCount / iterations >= 0.6) {
            if (topCount / iterations >= 0.6) {
                ringCount = RingCount.QUAD_STACK;
            }
            else {
                ringCount = RingCount.SINGLE_STACK;
            }
        }


        mode.telemetry.addData("Top Dist Sensor", robot.distanceSensors[LEFT_SENSOR].getDistance());
        mode.telemetry.update();

        return ringCount;
    }

    protected void turnToRingStack(RingCount ringCount){
            double heading = Math.atan2(RING_STACK_TRUE_LOC.getX() - robot.getRobotLocation().getX(), RING_STACK_TRUE_LOC.getY() - robot.getRobotLocation().getY());
            heading = Math.toDegrees(heading);
//            heading = (360.0 - heading) % 360;
        if(ringCount == RingCount.QUAD_STACK){
            heading = heading - 2.5;
        }
            robot.turnToHeading(heading, mode);
    }

    protected void turnToHighGoalFromRings(Location robotLoc){
        double heading = Math.atan2(HIGH_GOAL_LOC.getX() - robotLoc.getX(), HIGH_GOAL_LOC.getY() - robotLoc.getY());
        heading = Math.toDegrees(heading);
//        heading = (360.0 - heading) % 360;
        robot.turnToHeading(heading, 1, mode);

    }

    protected void moveToSecondWobble(){
        robot.driveToLocationPID(new Location(RED_WOBBLE_GOAL_LEFT.getX(), robot.getRobotLocation().getY()), MED_SPEED, mode);
        robot.driveToLocationPID(new Location(robot.getRobotLocation().getX(), RED_WOBBLE_GOAL_LEFT.getY()), MED_SPEED, mode);
    }

    // parks the robot over the launch line
    protected void park(LinearOpMode mode, RingCount ringCount) {
        // turn everything off
        if(mode.opModeIsActive()) {
            shooter.pauseShooter();
            intake.intakeOff();
            wobbleGrabber.setGrabAndDropAngle();

            if(mode.opModeIsActive() && ringCount == RingCount.SINGLE_STACK) {
                // drive to parking line
                robot.driveToLocationPID(new Location(robot.getRobotLocation().getX(), PARKING_LOCATION.getY()), MAX_SPEED, mode);
//                robot.driveToLocationPID(new Location(robot.getRobotLocation().getX(), PARKING_LOCATION.getY()), MED_SPEED, mode);
                wobbleGrabber.pause();
            }
            else if(mode.opModeIsActive() && ringCount == RingCount.NO_RINGS) {
                wobbleGrabber.setLiftAngle();
                robot.driveDistance(8, UltimateNavigation2.SOUTH, HIGH_SPEED, mode);
                robot.driveToLocationPID(PARKING_LOCATION, HIGH_SPEED, mode);
            }
            else if(mode.opModeIsActive() && ringCount == RingCount.QUAD_STACK){
                wobbleGrabber.setInitAngle();
            }
        }
    }
    // These drive functions have no purpose other than testing driveToLocationPID
    public void driveInSquare() {
        robot.driveToLocationPID(new Location(48, -24, 0), HIGH_SPEED, mode);
        delay(500);
        if (!mode.opModeIsActive()) return;
        robot.driveToLocationPID(new Location(0, -24, 0), HIGH_SPEED, mode);
        delay(500);
        if (!mode.opModeIsActive()) return;
        robot.driveToLocationPID(new Location(0, 24, 0), HIGH_SPEED, mode);
        delay(500);
        if (!mode.opModeIsActive()) return;
        robot.driveToLocationPID(new Location(48, 24, 0), HIGH_SPEED, mode);
        delay(500);
        if (!mode.opModeIsActive()) return;
    }


    public void driveToCorners() {
        robot.driveDistanceToLocation(new Location(-20, -61.6, 0), HIGH_SPEED, mode);
        delay(500);
        if (!mode.opModeIsActive()) return;
        robot.driveDistanceToLocation(new Location(48.8, -61.6, 0), HIGH_SPEED, mode);
        delay(500);
        if (!mode.opModeIsActive()) return;
        robot.driveDistanceToLocation(new Location(48.8, 61.6, 0), HIGH_SPEED, mode);
        delay(500);
        if (!mode.opModeIsActive()) return;
        robot.driveDistanceToLocation(new Location(-20, 61.6, 0), HIGH_SPEED, mode);
        delay(500);
        if (!mode.opModeIsActive()) return;
    }

    private void delay(int milliseconds) {
        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() - time < milliseconds);
    }
}