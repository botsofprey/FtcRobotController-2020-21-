package Autonomous.OpModes.UltimateAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Actions.Ultimate.RingIntakeSystemV2Test;
import Actions.Ultimate.ShooterSystemV2Test;
import Actions.Ultimate.WobbleGrabberV2Test;
import Autonomous.AutoAlliance;
import Autonomous.Location;
import Autonomous.RingCount;
import Autonomous.VisionHelperUltimateGoal;
import DriveEngine.Ultimate.UltimateNavigation2;

import static Autonomous.ConfigVariables.LEFT_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.MIDDLE_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.PARKING_LOCATION;
import static Autonomous.ConfigVariables.POWER_SHOT_POINT;
import static Autonomous.ConfigVariables.QUAD_STACK_END_POINT;
import static Autonomous.ConfigVariables.RED_WOBBLE_GOAL_LEFT;
import static Autonomous.ConfigVariables.RED_ZONE_ONE;
import static Autonomous.ConfigVariables.RED_ZONE_THREE;
import static Autonomous.ConfigVariables.RED_ZONE_TWO;
import static Autonomous.ConfigVariables.RIGHT_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.SHOOTING_LINE_POINT;
import static Autonomous.ConfigVariables.RING_STACK_START_POINT;

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

    protected WobbleGrabberV2Test wobbleGrabber;
    protected ShooterSystemV2Test shooter;
    protected RingIntakeSystemV2Test intake;

    protected VisionHelperUltimateGoal vision;

    protected static final int WOBBLE_OFFSET = 4;
    protected static final double MAX_SPEED = 50;
    protected static final double HIGH_SPEED = 35;
    protected static final double MED_SPEED = 25;
    protected static final double LOW_SPEED = 15;
    protected static final double MIN_SPEED = 5;
    protected static final long SLEEP_TIME = 500;
    protected static final long EMERGENCY_PARK_TIME = 0;

    public UltimateV2Autonomous(AutoAlliance alliance, Location startLocation, final LinearOpMode mode) {

        this.alliance = alliance;
        this.mode = mode;

        wobbleGrabber = new WobbleGrabberV2Test(mode.hardwareMap);
        shooter = new ShooterSystemV2Test(mode.hardwareMap);
        intake = new RingIntakeSystemV2Test(mode.hardwareMap);

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
    protected void performPowerShots(LinearOpMode mode, double runtime) {
        if(mode.opModeIsActive()) { // if the time remaining is more than the required action time, perform it
            wobbleGrabber.setLiftAngle();
            shooter.spinUp();
            robot.driveToLocationPID(POWER_SHOT_POINT, LOW_SPEED, mode);

            // perform shots
            shooter.setPowerShotPower(); // spin up motor to expected power shot rpm
            robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, mode); // turn to heading for first power shot and shoot
            indexShooter();

            robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING, mode); // turn to heading for second power shot and shoot
            indexShooter();

            robot.turnToHeading(LEFT_POWER_SHOT_HEADING, mode); // turn to heading for third power shot and shoot
            indexShooter();

            shooter.pauseShooter();
        }
    }

    // drives to the correct wobble goal delivery zone from the current robot location
    protected void deliverWobbleGoal(LinearOpMode mode, RingCount ringCount, double runtime, int wobbleNum) {
        if(mode.opModeIsActive()) {
            Location targetLocation = RED_ZONE_ONE;
            if (ringCount == RingCount.SINGLE_STACK)
                targetLocation = RED_ZONE_TWO;
            else if (ringCount == RingCount.QUAD_STACK)
                targetLocation = RED_ZONE_THREE;
            if(wobbleNum == 1) {
                robot.driveToLocationPID(targetLocation, HIGH_SPEED, mode);
            }
            else { // Place second wobble goal slightly off from the location of the first to avoid collision
                Location offsetTarget = new Location(targetLocation.getX() + WOBBLE_OFFSET, targetLocation.getY() - WOBBLE_OFFSET);
                robot.driveToLocationPID(offsetTarget, MED_SPEED, mode);
            }
            wobbleGrabber.releaseWobble();
            wobbleGrabber.setInitAngle();
        }
    }

    // reorients and drives to intake the extra rings but only if there are extra rings
    protected void intakeExtraRings(LinearOpMode mode, RingCount ringCount, double runtime) {
        if(mode.opModeIsActive() && ringCount != RingCount.NO_RINGS) {
            mode.telemetry.addData("made it inside ring function", "");
            mode.telemetry.update();
            intake.intake();
            robot.driveToLocationPID(new Location(robot.getRobotLocation().getX(), RING_STACK_START_POINT.getY(), UltimateNavigation2.NORTH), MED_SPEED, mode);
            robot.turnToHeading(UltimateNavigation2.EAST, mode);
            if(ringCount == ringCount.SINGLE_STACK) {
                robot.driveToLocationPID(QUAD_STACK_END_POINT, HIGH_SPEED, mode);
            }
            else {
                robot.driveToLocationPID(QUAD_STACK_END_POINT, LOW_SPEED, mode);
//                robot.driveDistance(24, UltimateNavigation2.NORTH, LOW_SPEED, mode);
            }
        }
    }

    // drives to the second wobble goal and grabs it
    protected void obtainSecondWobbleGoal(LinearOpMode mode, double runtime) {
        if(mode.opModeIsActive()) {
            mode.telemetry.addData("made it inside secondWobbleGoal function", "");
            mode.telemetry.update();
            robot.turnToHeading(UltimateNavigation2.SOUTH, mode);
            moveToSecondWobble();
            wobbleGrabber.setGrabAndDropAngle(); // Bring arm down to grab angle
            wobbleGrabber.releaseWobble(); // Open claw
            mode.sleep(SLEEP_TIME);
//            robot.driveOnHeading(UltimateNavigation2.NORTH, LOW_SPEED); // Drive forwards, slowly
//            while(!wobbleGrabber.sensor.isPressed() && mode.opModeIsActive());
//            robot.brake(); // Once sensor is pressed, stop
            wobbleGrabber.setClawGrabAngle(); // Close claw
            mode.sleep(SLEEP_TIME);
            wobbleGrabber.setLiftAngle(); // Lift arm back up
            robot.turnToHeading(UltimateNavigation2.NORTH, mode);
        }
    }

    // drives to the correct position to shoot the extra rings then shoots them if there are extra rings
    protected void shootExtraRings(LinearOpMode mode, RingCount ringCount, double runtime) {
        if(mode.opModeIsActive() && ringCount != RingCount.NO_RINGS) {
            mode.telemetry.addData("made it inside extra rings function", "");
            mode.telemetry.update();
                robot.turnToHeading(UltimateNavigation2.NORTH, 1, mode);
                robot.driveDistanceToLocation(SHOOTING_LINE_POINT, MED_SPEED, mode);

                // shoot rings
                shooter.setHighGoalPower(); // spin up motor to proper high goal rpm
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


    protected void dropIntakeAndWobble(LinearOpMode mode) {
        if(mode.opModeIsActive()){
            intake.intakeServoOut();
            wobbleGrabber.setLiftAngle();
        }
    }

    protected void moveToSecondWobble(){
        robot.driveDistance(RED_WOBBLE_GOAL_LEFT.getX() - robot.getRobotLocation().getX(), UltimateNavigation2.EAST, LOW_SPEED, mode);
        robot.driveDistance(RED_WOBBLE_GOAL_LEFT.getY() - robot.getRobotLocation().getY(), UltimateNavigation2.NORTH, LOW_SPEED, mode);
    }

    // parks the robot over the launch line
    protected void park(LinearOpMode mode) {
        // turn everything off
        if(mode.opModeIsActive()){
            wobbleGrabber.pause();
            shooter.pauseShooter();
            intake.intakeOff();

            // drive to parking line
            robot.driveDistance(PARKING_LOCATION.getY() - robot.getRobotLocation().getY(), UltimateNavigation2.NORTH, MED_SPEED, mode);
        }
    }

    public void driveInSquare() {
        robot.driveToLocationPID(new Location(-48, -24, 0), HIGH_SPEED, mode);
        if (!mode.opModeIsActive()) return;
        robot.driveToLocationPID(new Location(0, -24, 0), HIGH_SPEED, mode);
        if (!mode.opModeIsActive()) return;
        robot.driveToLocationPID(new Location(0, 24, 0), HIGH_SPEED, mode);
        if (!mode.opModeIsActive()) return;
        robot.driveToLocationPID(new Location(-48, 24, 0), HIGH_SPEED, mode);
        if (!mode.opModeIsActive()) return;
    }

    public void driveInSquare2() {
        robot.driveDistanceToLocation(new Location(48, -24, 0), HIGH_SPEED, mode);
        if (!mode.opModeIsActive()) return;
        robot.driveDistanceToLocation(new Location(0, -24, 0), HIGH_SPEED, mode);
        if (!mode.opModeIsActive()) return;
        robot.driveDistanceToLocation(new Location(0, 24, 0), HIGH_SPEED, mode);
        if (!mode.opModeIsActive()) return;
        robot.driveDistanceToLocation(new Location(48, 24, 0), HIGH_SPEED, mode);
        if (!mode.opModeIsActive()) return;
    }
}