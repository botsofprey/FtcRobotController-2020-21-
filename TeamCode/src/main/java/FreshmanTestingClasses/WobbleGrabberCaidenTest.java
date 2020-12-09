package FreshmanTestingClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Caiden Lineman on 11/04/2020.
 *
 * functions to run the wobble grabber; picks up wobble grabber as well as putting it down
 *
 */

public class WobbleGrabberCaidenTest {

    private HardwareMap hardwareMap;

    private Servo claw;
    private DcMotor arm;

    private static final int ARM_HOLD_ANGLE = 45;
    private static final int ARM_RELEASE_ANGLE = 0;
    private static final int CLAW_OPEN_ANGLE = 75; //105
    private static final int CLAW_CLOSE_ANGLE = 105; //75

    private static final int UP_POSITION = ARM_HOLD_ANGLE / 180; // this is integer division. These four numbers evaluate to 0
    private static final int DOWN_POSITION = ARM_RELEASE_ANGLE / 180;
    private static final int OPEN_POSITION = CLAW_OPEN_ANGLE / 180;
    private static final int CLOSE_POSITION = CLAW_CLOSE_ANGLE / 180;

    private static final double ARM_POWER = .05;

    public WobbleGrabberCaidenTest(HardwareMap hwm) {
        hardwareMap = hwm;
        claw = hardwareMap.servo.get("wobbleGrabberClaw");
        arm = hardwareMap.dcMotor.get("wobbleGrabberArm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void grabWobbleGoal() {
        // move arm to down position
        // move claw to closed position

        /** test code that worked to base updates on
        arm.setTargetPosition(100);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        while(arm.isBusy());
        arm.setPower(0);
        */

        claw.setPosition(OPEN_POSITION);
        arm.setTargetPosition(DOWN_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);
        while(!arm.isBusy()) {
            claw.setPosition(CLOSE_POSITION);
            arm.setTargetPosition(UP_POSITION);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(-ARM_POWER);
    }

    public void releaseWobbleGoal() {
        // move arm to down position
        // move claw to open position
        arm.setTargetPosition(DOWN_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);
        while(!arm.isBusy()) {
            claw.setPosition(OPEN_POSITION);
            arm.setTargetPosition(UP_POSITION);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(-ARM_POWER);
        }
        while(!arm.isBusy()) {
            claw.setPosition(CLOSE_POSITION);
        }
    }
}
