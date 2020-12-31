package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;

import Actions.ActionHandler;
import MotorControllers.MotorController;

public class RingIntakeSystemV2Test implements ActionHandler {

    private MotorController intakeMotor;

    public RingIntakeSystemV2Test(HardwareMap hardwareMap) {
        try {
            intakeMotor = new MotorController("intakeMotor", "MotorConfig/NoLoad40.json", hardwareMap);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void intake() {
        intakeMotor.setMotorPower(1);
    }

    public void spit() {
        intakeMotor.setMotorPower(-1);
    }

    public void pause() {
        intakeMotor.brake();
    }

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
        intakeMotor.killMotorController();
    }
}
