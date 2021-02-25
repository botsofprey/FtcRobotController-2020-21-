package Autonomous.OpModes.UltimateAuto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Actions.Ultimate.RingIntakeSystemV2;
import Actions.Ultimate.ShooterSystemV2Test;
import Actions.Ultimate.WobbleGrabberV2;
import Autonomous.AutoAlliance;
import Autonomous.Location;
import Autonomous.RingCount;
import Autonomous.VisionHelperUltimateGoal;
import DriveEngine.Ultimate.UltimateNavigation2;

import static Actions.Ultimate.WobbleGrabberV2.ARM_POWER;
import static Autonomous.ConfigVariables.HIGH_GOAL_LOC;
import static Autonomous.ConfigVariables.LEFT_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.MIDDLE_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.PARKING_LOCATION;
import static Autonomous.ConfigVariables.RED_WOBBLE_GOAL_LEFT;
import static Autonomous.ConfigVariables.RED_ZONE_ONE;
import static Autonomous.ConfigVariables.RED_ZONE_THREE;
import static Autonomous.ConfigVariables.RED_ZONE_TWO;
import static Autonomous.ConfigVariables.RIGHT_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.RING_STACK_TRUE_LOC;
import static Autonomous.ConfigVariables.SHOOTING_LINE_POINT;
import static Autonomous.ConfigVariables.WOBBLE_OFFSET;
import static DriveEngine.Ultimate.UltimateNavigation2.BACK_SENSOR;
import static DriveEngine.Ultimate.UltimateNavigation2.EAST;
import static DriveEngine.Ultimate.UltimateNavigation2.EAST_ON_45;
import static DriveEngine.Ultimate.UltimateNavigation2.LEFT_SENSOR;
import static DriveEngine.Ultimate.UltimateNavigation2.NORTH;
import static DriveEngine.Ultimate.UltimateNavigation2.NORTH_ON_45;
import static DriveEngine.Ultimate.UltimateNavigation2.SOUTH;
import static DriveEngine.Ultimate.UltimateNavigation2.SOUTH_ON_45;
import static DriveEngine.Ultimate.UltimateNavigation2.WEST;
import static DriveEngine.Ultimate.UltimateNavigation2.WEST_ON_45;

/**
 * Author: Ethan Fisher
 * Date: 10/29/2020
 *
 * Autonomous for Ultimate Goal
 */
public class UltimateV2Autonomous extends Thread {

    private final AutoAlliance alliance;
    private final LinearOpMode mode;

    protected UltimateNavigation2 robot;

    protected WobbleGrabberV2 wobbleGrabber;
    protected ShooterSystemV2Test shooter;
    protected RingIntakeSystemV2 intake;

    protected long prevShotTime;

    protected static final double MAX_SPEED = 50;
    protected static final double HIGH_SPEED = 45;
    protected static final double MED_SPEED = 35;
    protected static final double LOW_SPEED = 20;
    protected static final double KIND_OF_SLOW_SPEED = 28;
    protected static final double MIN_SPEED = 5;
    protected static final long SLEEP_TIME = 550;
    public volatile boolean shouldRun = true;
    private static final double INTAKE_OFFSET_FROM_CENTER = 2.0;

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

        prevShotTime = System.currentTimeMillis();
    }

    // converts red to blue. If it is red, nothing happens
    public Location redToBlue(Location location) {
        if (alliance == AutoAlliance.BLUE)
            return new Location(-location.getX(), location.getY(), 360 - location.getHeading());
        return location;
    }

    public void kill() {
        shouldRun = false;
        robot.brake();
        robot.stopNavigation();
        shooter.kill();
        intake.kill();
        wobbleGrabber.kill();

        mode.telemetry.addData("Robot", "Stopped");
        mode.telemetry.update();
    }

    // AUTONOMOUS FUNCTIONS HERE
    protected void init() {
        intake.intakeServoIn();
        // set initial servo positions
    }

    private void printRPM() {
        mode.telemetry.addData("rpm", shooter.getRPM());
        mode.telemetry.update();
        Log.d("rpm", shooter.getRPM() + "");
    }

    // drives from current location to where power shots must be performed
    // performs power shots from right to left
    protected void performPowerShots(RingCount ringCount) {
        if(mode.opModeIsActive()) { // if the time remaining is more than the required action time, perform it
            shooter.setPowerShotRPM();

            // This is a way of shooting power shots with one location and different headings
            robot.driveToLocationPID(SHOOTING_LINE_POINT, KIND_OF_SLOW_SPEED, mode);

            // This is a really gross way of shooting power shots with each path using slightly different headings
            switch(ringCount){
                case NO_RINGS:
                    robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, 1,  mode); // turn to heading for first power shot and shoot
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 300);
                    robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING + 1, 1, mode);
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 100);
                    robot.turnToHeading(LEFT_POWER_SHOT_HEADING + 1, 1, mode); // turn to heading for third power shot and shoot
                    printRPM();
                    powerShotIndex();
                    break;

                case SINGLE_STACK:
                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 300);
                    robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, 1,  mode); // turn to heading for first power shot and shoot
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 100);
                    robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING + 1, 1, mode);
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 200);
                    robot.turnToHeading(LEFT_POWER_SHOT_HEADING + 1.0, 1, mode); // turn to heading for third power shot and shoot
                    printRPM();
                    powerShotIndex();
                    break;

                case QUAD_STACK:
                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 400);
                    robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, 1,  mode); // turn to heading for first power shot and shoot
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 300);
                    robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING + 2.5, 1, mode);
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 350); // Duplicate powers are there just in case we want to tune some more in the future
                    robot.turnToHeading(LEFT_POWER_SHOT_HEADING + 1.0, 1, mode); // turn to heading for third power shot and shoot
                    printRPM();
                    powerShotIndex();
                    break;
            }
            shooter.setRPM(ShooterSystemV2Test.HIGH_GOAL_RPM + 100);
        }
    }

    // drives to the correct wobble goal delivery zone from the current robot location
    protected void deliverWobbleGoal(RingCount ringCount, int wobbleNum) {
        if(mode.opModeIsActive()) {
            wobbleGrabber.setLiftAngle();
            robot.turnToHeading(NORTH, mode);
            Location targetLocation = RED_ZONE_ONE;
            switch(ringCount) {
                case SINGLE_STACK:
                    targetLocation = RED_ZONE_TWO;
                    break;
                case QUAD_STACK:
                    targetLocation = RED_ZONE_THREE;
                    break;
            }
            if(wobbleNum == 1) {
                if (ringCount == RingCount.SINGLE_STACK) {
                    robot.driveDistance(RED_ZONE_ONE.getY() - robot.getRobotLocation().getY(), NORTH, MAX_SPEED, mode);
                    robot.driveToLocationPID(targetLocation, MAX_SPEED, 1.5, mode);
                } else if(ringCount == RingCount.QUAD_STACK) {
                    robot.driveDistance(targetLocation.getY() - robot.getRobotLocation().getY(), NORTH, MAX_SPEED, mode);
                } else {
                    robot.driveDistance(targetLocation.getY() - robot.getRobotLocation().getY() - 3, NORTH, MED_SPEED, mode);
                }
            }
            else { // Place second wobble goal slightly off from the location of the first to avoid collision
                Location offsetTarget = new Location(targetLocation.getX(), targetLocation.getY() - WOBBLE_OFFSET);
                if(ringCount == RingCount.NO_RINGS) {
                    offsetTarget = new Location(targetLocation.getX(), targetLocation.getY() - WOBBLE_OFFSET + 4.5);
                }
                robot.driveToLocationPID(offsetTarget, MAX_SPEED, 1.5, mode);
            }

            wobbleGrabber.releaseWobble();
            if (wobbleNum == 1)
                wobbleGrabber.setInitAngle();
        }
    }

    // reorients and drives to intake the extra rings but only if there are extra rings
    protected void intakeExtraRings(RingCount ringCount) {
        if(mode.opModeIsActive() && ringCount != RingCount.NO_RINGS) {
            intake.intake();
            double distToStack = turnToRingStack();
            robot.driveDistance(distToStack + 3/*drive 3 inches past the stack point*/, NORTH, MED_SPEED, mode, () -> {
                wobbleGrabber.setArmPower(-ARM_POWER);
                wobbleGrabber.armSensorCheck(mode);
                return true;
            });
            robot.turnToHeading(NORTH, mode);
        }
    }

    // drives to the second wobble goal and grabs it
    protected void obtainSecondWobbleGoal(RingCount ringCount) {
        if(mode.opModeIsActive()) {
            wobbleGrabber.setGrabAndDropAngle();
            wobbleGrabber.releaseWobble(); // Open claw
            robot.turnToHeading(UltimateNavigation2.SOUTH,.75, mode);
            if(ringCount == RingCount.NO_RINGS) {
                robot.driveDistance(RED_WOBBLE_GOAL_LEFT.getY() - robot.getRobotLocation().getY(), NORTH, MED_SPEED, mode);
                wobbleGrabber.setClawGrabAngle(); // Close claw
                mode.sleep(SLEEP_TIME);
                wobbleGrabber.setInitAngleSlow(); // Lift arm back up
            } else {
                robot.driveDistance(RED_WOBBLE_GOAL_LEFT.getX() - robot.getRobotLocation().getX(), WEST, LOW_SPEED, mode);
                mode.sleep(100); // This is just to reduce momentum before driving to the wobble goal so the rings aren't knocked over
                robot.driveDistance(RED_WOBBLE_GOAL_LEFT.getY() - robot.getRobotLocation().getY(), NORTH, MED_SPEED, mode);
                wobbleGrabber.setClawGrabAngle(); // Close claw
                mode.sleep(SLEEP_TIME);
                wobbleGrabber.setInitAngleSlow(); // Lift arm back up
                // We need to drive forwards a bit because otherwise we are going to hit the ring stack when turning
                robot.driveToLocationPID(new Location(RED_WOBBLE_GOAL_LEFT.getX(), RED_WOBBLE_GOAL_LEFT.getY() - 12.5), HIGH_SPEED, mode);
            }
        }
    }

    // drives to the correct position to shoot the extra rings then shoots them if there are extra rings
    protected void shootExtraRings(RingCount ringCount) {
        if(mode.opModeIsActive() && ringCount != RingCount.NO_RINGS) {
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
                shooter.setIndexLeft();
            }
        }
        shooter.setRPM(0);
    }

    protected void indexShooter() {
        regulateShotTime();
        Log.d("Shooter rpm", shooter.getRPM()+"");
        shooter.setIndexLeft();
        regulateShotTime();
        Log.d("Shooter rpm", shooter.getRPM()+"");
        shooter.setIndexRight();
    }

    protected void regulateShotTime() {
        while (mode.opModeIsActive() && prevShotTime + UltimateV2Autonomous.SLEEP_TIME > System.currentTimeMillis());
        prevShotTime = System.currentTimeMillis();
    }

    protected void powerShotIndex() {
        regulateShotTime();
        shooter.setIndexLeft();
        regulateShotTime();
        shooter.setIndexRight();
        mode.sleep(100);
    }

    protected void dropIntakeAndWobble() {
        if(mode.opModeIsActive()){
            wobbleGrabber.setClawGrabAngle();
            mode.sleep(SLEEP_TIME/2); // Pause so that the wobble goal can be held
            shooter.setIndexRight();
            intake.intakeServoOut();
            shooter.setPowerShotRPM();
            wobbleGrabber.setGrabAndDropAngle();
        }
    }

    protected RingCount driveToRingStack() {
        final int[] bottomHits = {0};
        final int[] topHits = {0};

        if(mode.opModeIsActive()) {
            intake.intake();
            robot.driveDistance(44.0, NORTH, MED_SPEED, mode, () -> {
                if (Math.abs(robot.getRobotLocation().getY() + 20) < 3) {
                    double bottomDistance = robot.distanceSensors[BACK_SENSOR].getDistance();
                    if (bottomDistance <= 13.0) { // lower, because of 2nd wobble goal
                        bottomHits[0]++;
                    }
                    double topDistance = robot.distanceSensors[LEFT_SENSOR].getDistance();
                    if (topDistance <= 14.0) {
                        topHits[0]++;
                    }
                    Log.d("Bottom Hits", bottomHits[0] + "");
                    Log.d("Top Hits", topHits[0] + "");
                    Log.d("BottomRingSensor", "\t" + bottomDistance + "\t" + topDistance + "\t" + robot.getRobotLocation().getY());
                }
                return true;
            }, false);
            intake.pauseIntake();
        }

//        mode.telemetry.addData("top", topHits[0]);
//        mode.telemetry.addData("bottom", bottomHits[0]);
        int top = topHits[0];
        int bottom = bottomHits[0];
        if (top > 1)
            return RingCount.QUAD_STACK;
        return (bottom > 1) ? RingCount.SINGLE_STACK : RingCount.NO_RINGS;
    }

    protected double turnToRingStack(){
        double dx = RING_STACK_TRUE_LOC.getX() - robot.getRobotLocation().getX();
        double dy = RING_STACK_TRUE_LOC.getY() - robot.getRobotLocation().getY();
        Log.d("ringStack::dx", dx + "");
        Log.d("ringStack::dy", dy + "");
        double distanceToStack = Math.sqrt(dx * dx + dy * dy);
        Log.d("ringStack::distToStack", distanceToStack + "");

        double stackHeading = Math.atan2(dy, dx);//compute angle difference to ring stack
        Log.d("ringStack::stackHeading", stackHeading + "");
        double headingOffset = 2 * Math.asin(INTAKE_OFFSET_FROM_CENTER / (2.0 * distanceToStack));
        Log.d("ringStack:headingOffset", headingOffset + "");
        robot.turnToHeading(90 - Math.toDegrees(stackHeading - headingOffset), 1, mode);
        return distanceToStack;
    }

    protected void turnToHighGoalFromRings(Location robotLoc){
        double heading = Math.atan2(HIGH_GOAL_LOC.getX() - robotLoc.getX(), HIGH_GOAL_LOC.getY() - robotLoc.getY());
        heading = Math.toDegrees(heading);
        robot.turnToHeading(heading, 1, mode);
    }

    // parks the robot over the launch line
    protected void park(RingCount ringCount) {
        // turn everything off
        if(mode.opModeIsActive()) {
            shooter.pauseShooter();
            wobbleGrabber.setGrabAndDropAngle();

            if(mode.opModeIsActive() && ringCount == RingCount.SINGLE_STACK) {
                // drive to parking line
                robot.driveToLocationPID(new Location(robot.getRobotLocation().getX(), PARKING_LOCATION.getY()), MAX_SPEED, 6, mode);
                wobbleGrabber.pause();
            }
            else if(mode.opModeIsActive() && ringCount == RingCount.NO_RINGS) {
                wobbleGrabber.setLiftAngle();
                robot.driveDistance(8, UltimateNavigation2.SOUTH, HIGH_SPEED, mode);
                robot.driveToLocationPID(PARKING_LOCATION, HIGH_SPEED, mode);
            }
            else if(mode.opModeIsActive() && ringCount == RingCount.QUAD_STACK)
                robot.driveToLocationPID(new Location(robot.getRobotLocation().getX(), PARKING_LOCATION.getY()), MAX_SPEED, 6, mode);
            intake.pauseIntake();
        }
    }

    protected void deliverWobbleGoalOn45(RingCount ringCount, int wobbleNum){
        if(mode.opModeIsActive()){
            wobbleGrabber.setLiftAngle();
            robot.turnToHeading(45, mode);
            Location targetLocation = RED_ZONE_ONE;
            switch(ringCount) {
                case SINGLE_STACK:
                    targetLocation = RED_ZONE_TWO;
                    break;
                case QUAD_STACK:
                    targetLocation = RED_ZONE_THREE;
                    break;
            }
            if(wobbleNum == 1) {
                if (ringCount == RingCount.SINGLE_STACK) {
                    robot.driveDistance(RED_ZONE_TWO.getY() - robot.getRobotLocation().getY(), NORTH_ON_45, MAX_SPEED, mode);
                    robot.driveDistance(RED_ZONE_TWO.getX() - robot.getRobotLocation().getX(), WEST_ON_45, MED_SPEED, mode);
                } else if(ringCount == RingCount.QUAD_STACK) {
                    robot.driveDistance(targetLocation.getY() - robot.getRobotLocation().getY(), NORTH_ON_45, MAX_SPEED, mode);
                } else {
                    robot.driveDistance(targetLocation.getY() - robot.getRobotLocation().getY() - 3, NORTH_ON_45, MED_SPEED, mode);
                }
            }
            else { // Place second wobble goal slightly off from the location of the first to avoid collision
                Location offsetTarget = new Location(targetLocation.getX(), targetLocation.getY() - WOBBLE_OFFSET);
                if(ringCount == RingCount.NO_RINGS) {
                    offsetTarget = new Location(targetLocation.getX(), targetLocation.getY() - WOBBLE_OFFSET + 4.5);
                }
                robot.driveDistance(offsetTarget.getY() - robot.getRobotLocation().getY(), NORTH_ON_45, MED_SPEED, mode);
                robot.driveDistance(offsetTarget.getX() - robot.getRobotLocation().getX(), EAST_ON_45, MED_SPEED, mode);
            }
            wobbleGrabber.releaseWobble();
            if (wobbleNum == 1)
                wobbleGrabber.setInitAngle();
        }
    }

    protected RingCount driveToRingStackOn45() {
        final int[] bottomHits = {0};
        final int[] topHits = {0};

        robot.driveDistance(44.0, NORTH_ON_45, MED_SPEED, mode, () -> {
            double topDistance = robot.distanceSensors[LEFT_SENSOR].getDistance();
            if (topDistance <= 10.0) {
                topHits[0]++;
            }
            if (Math.abs(robot.getRobotLocation().getY() + 22) < 6) {
                double bottomDistance = robot.distanceSensors[BACK_SENSOR].getDistance();
                if (bottomDistance <= 9.0) { // lower, because of 2nd wobble goal
                    bottomHits[0]++;
                }

                Log.d("Bottom Hits", bottomHits[0] + "");
                Log.d("Top Hits", topHits[0] + "");
                Log.d("BottomRingSensor", "\t" + bottomDistance + "\t" + topDistance + "\t" + robot.getRobotLocation().getY());
            }
            return true;
        }, false);

        int top = topHits[0];
        int bottom = bottomHits[0];
        if(top > 1){ return RingCount.QUAD_STACK; }
        return (bottom > 1) ? RingCount.SINGLE_STACK : RingCount.NO_RINGS;
    }


    protected void performPowerShotsOn45(RingCount ringCount){
        if(mode.opModeIsActive()) { // if the time remaining is more than the required action time, perform it
            shooter.setPowerShotRPM();

            // This is a way of shooting power shots with one location and different headings
            if(ringCount == RingCount.NO_RINGS){ // For no rings we are lower than the shooting point so we need to drive north rather than south
                robot.driveDistance(SHOOTING_LINE_POINT.getY() - robot.getRobotLocation().getY(), NORTH_ON_45, KIND_OF_SLOW_SPEED, mode);
            }
            else {
                robot.driveDistance(SHOOTING_LINE_POINT.getY() - robot.getRobotLocation().getY(), SOUTH_ON_45, KIND_OF_SLOW_SPEED, mode);
            }
            robot.driveDistance(SHOOTING_LINE_POINT.getX() - robot.getRobotLocation().getX(), WEST_ON_45, KIND_OF_SLOW_SPEED, mode);

//            robot.driveToLocationPID(SHOOTING_LINE_POINT, KIND_OF_SLOW_SPEED, mode);

            // This is a really gross way of shooting power shots with each path using slightly different headings
            switch(ringCount){
                case NO_RINGS:
                    robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, 1,  mode); // turn to heading for first power shot and shoot
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 300);
                    robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING + 1, 1, mode);
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 100);
                    robot.turnToHeading(LEFT_POWER_SHOT_HEADING + 1, 1, mode); // turn to heading for third power shot and shoot
                    printRPM();
                    powerShotIndex();
                    break;

                case SINGLE_STACK:
                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 300);
                    robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, 1,  mode); // turn to heading for first power shot and shoot
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 100);
                    robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING + 1, 1, mode);
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 200);
                    robot.turnToHeading(LEFT_POWER_SHOT_HEADING + 1.0, 1, mode); // turn to heading for third power shot and shoot
                    printRPM();
                    powerShotIndex();
                    break;

                case QUAD_STACK:
                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 400);
                    robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, 1,  mode); // turn to heading for first power shot and shoot
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 300);
                    robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING + 2.5, 1, mode);
                    printRPM();
                    powerShotIndex();

                    shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 350); // Duplicate powers are there just in case we want to tune some more in the future
                    robot.turnToHeading(LEFT_POWER_SHOT_HEADING + 1.0, 1, mode); // turn to heading for third power shot and shoot
                    printRPM();
                    powerShotIndex();
                    break;
            }
            shooter.setRPM(ShooterSystemV2Test.HIGH_GOAL_RPM + 100);
        }

    }

    protected void intakeExtraRingsOn45(RingCount ringCount) {
        if(mode.opModeIsActive() && ringCount != RingCount.NO_RINGS) {
            intake.intake();
            double distToStack = turnToRingStack();
            robot.driveDistance(distToStack + 3/*drive 3 inches past the stack point*/, NORTH_ON_45, MED_SPEED, mode, () -> {
                wobbleGrabber.setArmPower(-ARM_POWER);
                wobbleGrabber.armSensorCheck(mode);
                return true;
            });
            robot.turnToHeading(NORTH, mode);
        }
    }

    protected void obtainSecondWobbleGoalOn45(RingCount ringCount){
        if(mode.opModeIsActive()) {
            wobbleGrabber.setGrabAndDropAngle();
            wobbleGrabber.releaseWobble(); // Open claw
            robot.turnToHeading(SOUTH_ON_45,.75, mode);
            if(ringCount == RingCount.NO_RINGS) {
                robot.driveDistance(RED_WOBBLE_GOAL_LEFT.getY() - robot.getRobotLocation().getY(), EAST_ON_45, MED_SPEED, mode);
//                wobbleGrabber.setClawGrabAngle(); // Close claw
//                mode.sleep(SLEEP_TIME);
//                wobbleGrabber.setInitAngleSlow(); // Lift arm back up
            } else {
                robot.driveDistance(RED_WOBBLE_GOAL_LEFT.getX() - robot.getRobotLocation().getX(), NORTH_ON_45, LOW_SPEED, mode);
                mode.sleep(100); // This is just to reduce momentum before driving to the wobble goal so the rings aren't knocked over
                robot.driveDistance(RED_WOBBLE_GOAL_LEFT.getY() - robot.getRobotLocation().getY(), EAST_ON_45, MED_SPEED, mode);
//                wobbleGrabber.setClawGrabAngle(); // Close claw
//                mode.sleep(SLEEP_TIME);
//                wobbleGrabber.setInitAngleSlow(); // Lift arm back up
                // We need to drive forwards a bit because otherwise we are going to hit the ring stack when turning
                robot.driveDistance(12, NORTH_ON_45, HIGH_SPEED, mode);
            }
            robot.turnToHeading(SOUTH, mode);
            wobbleGrabber.setClawGrabAngle(); // Close claw
            mode.sleep(SLEEP_TIME);
            wobbleGrabber.setInitAngleSlow(); // Lift arm back up
        }
    }

    protected void parkOn45(RingCount ringCount){
        // turn everything off
        if(mode.opModeIsActive()) {
            shooter.pauseShooter();
            wobbleGrabber.setGrabAndDropAngle();

            if(mode.opModeIsActive() && ringCount == RingCount.SINGLE_STACK) {
                // drive to parking line
                robot.driveDistance(PARKING_LOCATION.getY() - robot.getRobotLocation().getY(), SOUTH_ON_45, HIGH_SPEED, mode);
                wobbleGrabber.pause();
            }
            else if(mode.opModeIsActive() && ringCount == RingCount.NO_RINGS) {
                wobbleGrabber.setLiftAngle();
                robot.driveDistance(8, SOUTH_ON_45, HIGH_SPEED, mode);
                robot.driveDistance(12, WEST_ON_45, HIGH_SPEED, mode);
                robot.driveDistance(PARKING_LOCATION.getY() - robot.getRobotLocation().getY(), NORTH_ON_45, HIGH_SPEED, mode);
            }
            else if(mode.opModeIsActive() && ringCount == RingCount.QUAD_STACK)
                robot.driveDistance(PARKING_LOCATION.getY() - robot.getRobotLocation().getY(), SOUTH_ON_45, HIGH_SPEED, mode);
            intake.pauseIntake();
        }
    }
}