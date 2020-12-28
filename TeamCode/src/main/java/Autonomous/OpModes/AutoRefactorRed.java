package Autonomous.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Autonomous.AutoAlliance;
import Autonomous.OpModes.AnnieAuto.AutonomousRoutines;


/**
 * This class provides a high level interface to the robot's functionality suitable for
 * constructing arbitrary autonomous programs which can easily be reflected between the
 * red and blue sides of the field.
 */


@Autonomous(name = "Auto Refactor Red Main", group = "Testers")
//@Disabled
public class AutoRefactorRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutonomousRoutines.runMainRoutine(AutoAlliance.RED, this);
    }

}
