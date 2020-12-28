package Actions.Ultimate;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import MotorControllers.PIDController;

public class WheelMotor {

    public DcMotor motor;
    private LinearOpMode mode;
    public double curRPM;
    public double targetRPM;
    private long prevTicks;
    private long prevTime;

    private static final int MAX_RPM = 6000;
    private static final double MINIMUM_TIME_DIFFERENCE = 100000000;// 1/10 of a second
    private static final long NANOS_PER_MINUTE = 60000000000L;
    private static final double TICKS_PER_ROTATION = 28;
    private static final double ADJUSTMENT_RATE = 5;

    private PIDController rpmController;

    public WheelMotor(String name, HardwareMap hardwareMap, final LinearOpMode mode) {
        motor = hardwareMap.dcMotor.get(name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetRPM = 0;
        prevTime = System.nanoTime();
        prevTicks = motor.getCurrentPosition();

        this.mode = mode;
    }

    public void setRPM(int RPM) {
        targetRPM = RPM;
        motor.setPower(targetRPM / MAX_RPM);
    }

    public void updateShooterRPM() {
        int currentTicks = motor.getCurrentPosition();
        long currentTime = System.nanoTime();

        long tickDiff = currentTicks - prevTicks;
        long timeDiff = currentTime - prevTime;
        if (timeDiff > MINIMUM_TIME_DIFFERENCE) {
            curRPM = (tickDiff / (double) timeDiff) * (NANOS_PER_MINUTE / TICKS_PER_ROTATION);
            prevTicks = currentTicks;
            prevTime = currentTime;
            Log.d("RPM", "" + curRPM);
            adjustRPM();
        }
    }

    private void adjustRPM() {
        double rpmDif = targetRPM - curRPM;
        double powerDif = rpmDif / (MAX_RPM * ADJUSTMENT_RATE);
        double newPower = motor.getPower() + powerDif;

        if (targetRPM == 0) newPower = 0;

//        if (targetRPM == 0)
//            motor.setPower(0);
//        else
            motor.setPower(newPower);

//        double curRPM = (dTicks / (double) dt) * (NANOS_PER_MINUTE / TICKS_PER_ROTATION);
//        double rpmCorrection = rpmController.calculatePID(curRPM);

//        motor.setPower((curRPM + rpmCorrection) / MAX_RPM);

//        mode.telemetry.addData("Motor Power", "" + targetPower);
//        mode.telemetry.update();
    }
}