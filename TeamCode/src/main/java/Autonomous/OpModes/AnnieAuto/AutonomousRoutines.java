package Autonomous.OpModes.AnnieAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Autonomous.AutoAlliance;
import Autonomous.ConfigVariables;
import Autonomous.Location;
import Autonomous.SkyStonePosition;

public class AutonomousRoutines {
    public static void runMainRoutine(AutoAlliance alliance, LinearOpMode mode) {
        AnnieAutonomous annie = new AnnieAutonomous(alliance, mode);

        SkyStonePosition skyStonePosition = annie.detectSkyStonePosition();

        mode.waitForStart();

        annie.startRun();

        annie.grabSkyStoneFromCenterGroup(skyStonePosition);

        annie.traverseSkyBridgeSidewaysNearCenterAndContinueTo(ConfigVariables.RED_FOUNDATION_CENTER);

        annie.liftToPosition(1);

        annie.grabFoundation();
        annie.repositionFoundation();
        annie.releaseStone();

        //TODO: go past skybridge and to next stone properly
//        annie.backupAndEstablishLocation();
        //TODO: backup and turn to zero

        annie.lowerLift();

//        annie.traverseSkyBridgeForwardNearCenterAndContinueTo(new Location(Math.abs(annie.robot.getRobotLocation().getX()), -35, 0)); // abs of x pos to make sure it is a red position, this location runs through red to blue

//        annie.grabSkyStoneFromWallGroup(skyStonePosition);

//        annie.traverseSkyBridgeSidewaysNearCenterAndContinueTo(new Location(ConfigVariables.RED_FOUNDATION_CENTER.getX(), ConfigVariables.RED_FOUNDATION_CENTER.getY()-19.25, ConfigVariables.RED_FOUNDATION_CENTER.getHeading()));

//        annie.driveToLocation(new Location(40, 29, 0));
//        annie.driveToLocation(new Location(50, 39, 0));

//        annie.liftToPosition(2);

        //annie.driveToLocation(new Location(35, 35, 0));  // TODO can we make the intent of this more clear?

//        annie.driveForwardUntilContact();
//        annie.releaseStone();

        //annie.driveToLocation(new Location(35, 35, 0));  // TODO can we make the intent of this more clear?

//        annie.lowerLift();

        annie.park();

        while (annie.opModeIsActive());

        annie.stop();
    }

    public static void runGrabTest(AutoAlliance alliance, LinearOpMode mode) {

        AnnieAutonomous annie = new AnnieAutonomous(alliance, mode);

        annie.reportLocation().updateTelemetry();

        mode.waitForStart();

        annie.startRun();

        annie.grabSkyStoneFromCenterGroup(SkyStonePosition.SKY_STONE_0);
        annie.reportLocation().updateTelemetry();
        annie.sleep(2000);
        annie.driveToLocation(new Location(55, -55, 270));
        annie.reportLocation().updateTelemetry();
        annie.sleep(2000);
        annie.releaseStone();
        annie.driveToLocation(new Location(55, -34.5, 270));
        annie.reportLocation().updateTelemetry();
        annie.sleep(2000);
        annie.grabSkyStoneFromCenterGroup(SkyStonePosition.SKY_STONE_2);
        annie.reportLocation().updateTelemetry();
        annie.sleep(2000);
        annie.driveToLocation(new Location(55, -18, 270));
        annie.reportLocation().updateTelemetry();
        annie.sleep(2000);
        annie.releaseStone();
        annie.driveToLocation(new Location(55, -34.5, 270));
        annie.reportLocation().updateTelemetry();
        annie.sleep(2000);
        annie.grabSkyStoneFromCenterGroup(SkyStonePosition.SKY_STONE_1);
        annie.reportLocation().updateTelemetry();
        annie.sleep(2000);
        annie.driveToLocation(new Location(55, -34.5, 270));
        annie.reportLocation().updateTelemetry();

        while (annie.opModeIsActive());

        annie.stop();
    }
}
