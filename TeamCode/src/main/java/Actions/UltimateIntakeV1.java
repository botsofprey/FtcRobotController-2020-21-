package Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;

import MotorControllers.MotorController;

public class UltimateIntakeV1 implements ActionHandler {

    private MotorController intake;
    private HardwareMap hardwareMap;

    public UltimateIntakeV1(HardwareMap hw) {
    hardwareMap = hw;

    try {
        intake = new MotorController("intake", "ActionConfig/LiftMotor.json", hardwareMap);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    catch(IOException e) {
        e.printStackTrace();
    }
    }

    public void intake() { intake.setMotorPower(1); }
    public void intakeBackwards() { intake.setMotorPower(-1); }
    public void pauseIntake() { intake.brake(); }

    @Override
    public boolean doAction(String action, long maxTimeAllowed) {
        return false;
    }

    @Override
    public boolean stopAction(String action) {
        return false;
    }

    @Override
    public boolean startDoingAction(String action) {
        return false;
    }

    @Override
    public void kill() {
        intake.brake();
    }
}
