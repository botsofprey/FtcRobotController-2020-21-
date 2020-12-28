package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RingIntakeSystemV1 {

    // TODO this class should be working rn

    private DcMotor intakeMotor;
    private boolean intakeOn;
    private boolean intakeReversed;

    private static final int MOTOR_POWER = 1;

    public RingIntakeSystemV1(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeOn = false;
        intakeReversed = true;
    }

    public void toggleIntakePower() {
        // Turn intake motor on or off
        intakeOn = !intakeOn;
        intakeMotor.setPower((intakeOn ? MOTOR_POWER : 0) * (intakeReversed ? -1 : MOTOR_POWER));
    }

    public void toggleIntake() {
        if(intakeOn) {
            toggleIntakePower();
        }
        else {
            intakeOn = true;
            intakeMotor.setPower(MOTOR_POWER);
        }
    }

    public void toggleOuttake() {
        if(intakeOn) {
            toggleIntakePower();
        }
        else {
            intakeOn = true;
            intakeMotor.setPower(-(MOTOR_POWER));
        }
    }

    public void toggleIntakeDirection() {
        // Reverse direction of intake motor
        if (intakeOn) {
            intakeReversed = !intakeReversed;
            intakeMotor.setPower(-intakeMotor.getPower());
        }
    }

    // the following are used in auto
    public void forwardIntake() {
        intakeReversed = false;
        intakeMotor.setPower(intakeOn ? MOTOR_POWER : 0);
    }

    public void reverseIntake() {
        intakeReversed = true;
        intakeMotor.setPower(-(intakeOn ? MOTOR_POWER : 0));
    }

    public void turnOn() {
        intakeOn = true;
        intakeMotor.setPower(MOTOR_POWER * (intakeReversed ? -1 : 1));
    }

    public void turnOff() {
        intakeOn = false;
        intakeMotor.setPower(0);
    }


}
