package Autonomous.OpModes.UltimateAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Actions.Ultimate.RingIntakeSystemV2;
import Actions.Ultimate.ShooterSystemV2;
import Actions.Ultimate.WobbleGrabberV2;
import Autonomous.AutoAlliance;
import Autonomous.Location;
import Autonomous.RingCount;
import Autonomous.VisionHelperUltimateGoal;
import DriveEngine.Ultimate.UltimateNavigation2;

import static Autonomous.ConfigVariables.ALT_HIGH_GOAL_HEADING;
import static Autonomous.ConfigVariables.ALT_HIGH_GOAL_POINT;
import static Autonomous.ConfigVariables.HIGH_GOAL_HEADING;
import static Autonomous.ConfigVariables.LEFT_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.MIDDLE_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.PARKING_LOCATION;
import static Autonomous.ConfigVariables.QUAD_STACK_END_POINT;
import static Autonomous.ConfigVariables.RED_WOBBLE_GOAL_LEFT;
import static Autonomous.ConfigVariables.RED_ZONE_ONE;
import static Autonomous.ConfigVariables.RED_ZONE_ONE_EAST;
import static Autonomous.ConfigVariables.RED_ZONE_THREE;
import static Autonomous.ConfigVariables.RED_ZONE_TWO;
import static Autonomous.ConfigVariables.RIGHT_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.RING_STACK_TRUE_LOC;
import static Autonomous.ConfigVariables.SHOOTING_LINE_POINT;
import static Autonomous.ConfigVariables.RING_STACK_START_POINT;
import static Autonomous.ConfigVariables.SINGLE_STACK_END_POINT;

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
    protected ShooterSystemV2 shooter;
    protected RingIntakeSystemV2 intake;

    protected VisionHelperUltimateGoal vision;

    protected static final int WOBBLE_OFFSET = 6;
    protected static final double MAX_SPEED = 50;
    protected static final double HIGH_SPEED = 45;
    protected static final double MED_SPEED = 25;
    protected static final double LOW_SPEED = 15;
    protected static final double MIN_SPEED = 5;
    protected static final long SLEEP_TIME = 500;
    protected static final long EMERGENCY_PARK_TIME = 0;

    public UltimateV2Autonomous(AutoAlliance alliance, Location startLocation, final LinearOpMode mode) {

        this.alliance = alliance;
        this.mode = mode;

        wobbleGrabber = new WobbleGrabberV2(mode.hardwareMap);
        shooter = new ShooterSystemV2(mode.hardwareMap);
        intake = new RingIntakeSystemV2(mode.hardwareMap);

        startLocation = redToBlue(startLocation);
        try {
            robot = new UltimateNavigation2(mode.hardwareMap, startLocation, startLocation.getHeading(), "RobotConfig/UltimateV2.json");
        } catch (Exception e) {
            e.printStackTrace();
        }

        vision = new VisionHelperUltimateGoal(VisionHelperUltimateGoal.WEBCAM, mode.hardwareMap);
    }

    // converts red to blue. If it is red, nothing happens
    public Location redToBlue(Location location) {
        if (alliance == AutoAlliance.BLUE)
            return new Location(-location.getX(), location.getY(), 360 - location.getHeading());
        return location;
    }

    public void kill() {
        vision.kill();
        wobbleGrabber.kill();
        shooter.kill();
        intake.kill();
        robot.stopNavigation();

        mode.telemetry.addData("Robot", "Stopped");
        mode.telemetry.update();
    }

    // AUTONOMOUS FUNCTIONS HERE
    protected void init() {
        wobbleGrabber.setClawGrabAngle();
        shooter.setIndexLeft();
        intake.intakeServoIn();
        // set initial servo positions
    }

    // drives from current location to where power shots must be performed
    // performs power shots from right to left
    protected void performPowerShots(LinearOpMode mode, double runtime, RingCount ringCount) {
        if(mode.opModeIsActive()) { // if the time remaining is more than the required action time, perform it
            shooter.spinUp();
//            if(ringCount != RingCount.NO_RINGS) { // if there is a present ring stack, drive to a checkpoint first
//                robot.driveToLocationPID(RED_ZONE_ONE, HIGH_SPEED, mode);
//            }
            shooter.setPowerShotPower(); // spin up motor to expected power shot rpm
            robot.driveToLocationPID(SHOOTING_LINE_POINT, HIGH_SPEED, mode);

            // perform shots
            robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, mode); // turn to heading for first power shot and shoot
            powerShotIndex();

            robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING, mode); // turn to heading for second power shot and shoot
            powerShotIndex();

            robot.turnToHeading(LEFT_POWER_SHOT_HEADING, mode); // turn to heading for third power shot and shoot
            powerShotIndex();

            shooter.pauseShooter();
        }
    }

    // drives to the correct wobble goal delivery zone from the current robot location
    protected void deliverWobbleGoal(LinearOpMode mode, RingCount ringCount, double runtime, int wobbleNum) {
        if(mode.opModeIsActive()) {
            wobbleGrabber.setLiftAngleSlow();
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
//                if(targetLocation == RED_ZONE_ONE){ // we only want to turn to the right on the first wobble and if its zone one
//                    robot.turnToHeading(UltimateNavigation2.EAST, mode);
//                    targetLocation = RED_ZONE_ONE_EAST; // Because we're facing right, we need a separate location for the zone
//                }
                    robot.driveToLocationPID(targetLocation, HIGH_SPEED, mode);
            }
            else { // Place second wobble goal slightly off from the location of the first to avoid collision
                Location offsetTarget = new Location(targetLocation.getX(), targetLocation.getY() - WOBBLE_OFFSET);
                robot.driveToLocationPID(offsetTarget, HIGH_SPEED, mode);
            }
            wobbleGrabber.releaseWobble();
            wobbleGrabber.setInitAngle();
        }
    }

    // reorients and drives to intake the extra rings but only if there are extra rings
    protected void intakeExtraRings(LinearOpMode mode, RingCount ringCount, double runtime) {
        if(mode.opModeIsActive() && ringCount != RingCount.NO_RINGS) {
            intake.intake();
//            robot.driveToLocationPID(RING_STACK_START_POINT, HIGH_SPEED, mode);
//            robot.turnToHeading(UltimateNavigation2.EAST, mode);
            turnToRingStack();
            if(ringCount == ringCount.SINGLE_STACK) {
//                robot.driveToLocationPID(SINGLE_STACK_END_POINT, MED_SPEED, mode);
                robot.driveDistance(24, UltimateNavigation2.NORTH, LOW_SPEED, mode);
            }
            else {
//                robot.driveToLocationPID(QUAD_STACK_END_POINT, MED_SPEED, mode);
                robot.driveDistance(36, UltimateNavigation2.NORTH, LOW_SPEED, mode);
            }
            robot.turnToHeading(UltimateNavigation2.NORTH, mode);
        }
    }

    // drives to the second wobble goal and grabs it
    protected void obtainSecondWobbleGoal(LinearOpMode mode, double runtime) {
        if(mode.opModeIsActive()) {
            wobbleGrabber.setGrabAndDropAngle();
            wobbleGrabber.releaseWobble(); // Open claw
            robot.turnToHeading(UltimateNavigation2.SOUTH, mode);
            robot.driveToLocationPID(new Location(RED_WOBBLE_GOAL_LEFT.getX(), robot.getRobotLocation().getY()), MED_SPEED, mode);
            robot.driveToLocationPID(new Location(RED_WOBBLE_GOAL_LEFT.getX(), RED_WOBBLE_GOAL_LEFT.getY() + 7), HIGH_SPEED, mode);
//            robot.driveOnHeading(UltimateNavigation2.NORTH, LOW_SPEED); // Drive forwards, slowly
//            while(!wobbleGrabber.sensor.isPressed() && mode.opModeIsActive());
//            robot.brake(); // Once sensor is pressed, stop
            wobbleGrabber.setClawGrabAngle(); // Close claw
            mode.sleep(SLEEP_TIME);
            wobbleGrabber.setInitAngle(); // Lift arm back up
            robot.driveToLocationPID(RED_WOBBLE_GOAL_LEFT, MED_SPEED, mode);
//            robot.turnToHeading(UltimateNavigation2.NORTH, mode);
        }
    }

    // drives to the correct position to shoot the extra rings then shoots them if there are extra rings
    protected void shootExtraRings(LinearOpMode mode, RingCount ringCount, double runtime) {
        if(mode.opModeIsActive() && ringCount != RingCount.NO_RINGS) {
                robot.turnToHeading(UltimateNavigation2.NORTH, mode);
                shooter.setHighGoalPower();
                robot.driveToLocationPID(ALT_HIGH_GOAL_POINT, MED_SPEED, mode);
                robot.turnToHeading(ALT_HIGH_GOAL_HEADING, mode);

                // shoot rings
                if(ringCount == RingCount.SINGLE_STACK){ // if there is only one extra ring, only index once
                    indexShooter();
                }
                else { // otherwise (there are four rings), index three times
                    for(int i = 0; i < 3; i++){
                        indexShooter();
                    }
            }
        }
    }

    protected void indexShooter(){
        shooter.setIndexLeft();
        mode.sleep(SLEEP_TIME);
        shooter.setIndexRight();
        mode.sleep(SLEEP_TIME);
    }

    protected void powerShotIndex(){
        shooter.setIndexLeft();
        mode.sleep(SLEEP_TIME);
        shooter.setIndexRight();
        mode.sleep(SLEEP_TIME/3);
    }


    protected void dropIntakeAndWobble(LinearOpMode mode) {
        if(mode.opModeIsActive()){
            intake.intakeServoOut();
            shooter.spinUp();
        }
    }

    protected void turnToRingStack(){
            double heading = Math.atan2(RING_STACK_TRUE_LOC.getY() - robot.getRobotLocation().getY(), RING_STACK_TRUE_LOC.getX() - robot.getRobotLocation().getX());
            heading = heading * (180.0 / Math.PI);
            heading = (360.0 - heading)% 360;
            robot.turnToHeading(heading, mode);
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

            if(ringCount != RingCount.NO_RINGS) {
                // drive to parking line
                robot.driveToLocationPID(PARKING_LOCATION, MAX_SPEED, mode);
//                robot.driveToLocationPID(new Location(robot.getRobotLocation().getX(), PARKING_LOCATION.getY()), MED_SPEED, mode);
                wobbleGrabber.pause();
            }
            else {
                wobbleGrabber.setLiftAngle();
                robot.driveDistance(8, UltimateNavigation2.SOUTH, HIGH_SPEED, mode);
                robot.driveToLocationPID(PARKING_LOCATION, HIGH_SPEED, mode);
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