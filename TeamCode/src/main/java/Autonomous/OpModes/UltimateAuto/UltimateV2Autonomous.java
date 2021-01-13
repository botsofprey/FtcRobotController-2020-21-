package Autonomous.OpModes.UltimateAuto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Actions.Ultimate.RingIntakeSystemV2Test;
import Actions.Ultimate.ShooterSystemV2Test;
import Actions.Ultimate.WobbleGrabberV2Test;
import Autonomous.AutoAlliance;
import Autonomous.Location;
import Autonomous.RingCount;
import Autonomous.VisionHelperUltimateGoal;
import DriveEngine.Ultimate.UltimateNavigation2;

import static Autonomous.ConfigVariables.PARKING_LOCATION;
import static Autonomous.ConfigVariables.POWER_SHOT_LEFT_ON_LINE;
import static Autonomous.ConfigVariables.POWER_SHOT_MIDDLE_ON_LINE;
import static Autonomous.ConfigVariables.POWER_SHOT_RIGHT_ON_LINE;
import static Autonomous.ConfigVariables.RED_WOBBLE_GOAL_RIGHT;
import static Autonomous.ConfigVariables.RED_ZONE_ONE;
import static Autonomous.ConfigVariables.RED_ZONE_THREE;
import static Autonomous.ConfigVariables.RED_ZONE_TWO;
import static Autonomous.ConfigVariables.SHOOTING_LINE_POINT;
import static Autonomous.ConfigVariables.STARTING_RING_PILE;

/**
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

    protected static final double MAX_SPEED = 50;
    protected static final double MED_SPEED = 25;
    protected static final double LOW_SPEED = 15;
    protected static final double MIN_SPEED = 5;

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
            Log.e("Robot error", e.toString());
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
        // set initial servo positions
    }

    // drives from current location to where power shots must be performed
    // performs power shots from right to left
    protected void performPowerShots(double runtime) {
        Log.d("runtime", ""+runtime);
        if(30 - runtime > -200) { // if the time remaining is more than the required action time, perform it
//            robot.driveDistanceToLocation(POWER_SHOT_RIGHT_ON_LINE, LOW_SPEED, mode);
//            robot.driveToLocation(POWER_SHOT_RIGHT_ON_LINE, LOW_SPEED, mode);
            robot.driveToLocationPID(POWER_SHOT_RIGHT_ON_LINE, LOW_SPEED, mode);
            mode.sleep(500);
            robot.driveToLocationPID(POWER_SHOT_MIDDLE_ON_LINE, LOW_SPEED, mode);
            mode.sleep(500);
            robot.driveToLocationPID(POWER_SHOT_LEFT_ON_LINE, LOW_SPEED, mode);
            // perform shots
        }
    }

    // drives to the correct wobble goal delivery zone from the current robot location
    protected void deliverWobbleGoal(RingCount ringCount, double runtime) {
        if(30 - runtime > -200) {
//            robot.turnToHeading(UltimateNavigation2.NORTH, 5, mode);
            switch (ringCount) {
                case NO_RINGS:
                    robot.driveToLocationPID(RED_ZONE_ONE, LOW_SPEED, mode);
                    break;
                case SINGLE_STACK:
                    robot.driveToLocationPID(RED_ZONE_TWO, LOW_SPEED, mode);
                    break;
                case QUAD_STACK:
                    robot.driveToLocationPID(RED_ZONE_THREE, LOW_SPEED, mode);
                    break;
            }
        }
    }

    // reorients and drives to intake the extra rings but only if there are extra rings
    protected void intakeExtraRings(RingCount ringCount, double runtime) {
        if(30 - runtime > -200 && ringCount != RingCount.NO_RINGS) {
            robot.turnToHeading(UltimateNavigation2.SOUTH, 5, mode);
            // turn on intake
            robot.driveToLocationPID(STARTING_RING_PILE, MED_SPEED, mode);
            // do a funky intake thingy here
        }
    }

    // drives to the second wobble goal and grabs it
    protected void obtainSecondWobbleGoal(double runtime) {
        if(30 - runtime > -200) {
//            robot.turnToHeading(UltimateNavigation2.SOUTH, mode);
            robot.driveToLocationPID(RED_WOBBLE_GOAL_RIGHT, MED_SPEED, mode);
            // grab wobble goal
            // lift wobble goal
        }
    }

    // drives to the correct position to shoot the extra rings then shoots them if there are extra rings
    protected void shootExtraRings(RingCount ringCount, double runtime) {
        if(30 - runtime > -200 && ringCount != RingCount.NO_RINGS) {
            robot.turnToHeading(UltimateNavigation2.NORTH, 1, mode);
            robot.driveToLocationPID(SHOOTING_LINE_POINT, MED_SPEED, mode);
            // shoot rings
        }
    }

    // parks the robot over the launch line
    protected void park() {
        // turn everything off
        robot.driveToLocationPID(new Location(robot.getRobotLocation().getX(), PARKING_LOCATION.getY()), MAX_SPEED, mode);
    }
}