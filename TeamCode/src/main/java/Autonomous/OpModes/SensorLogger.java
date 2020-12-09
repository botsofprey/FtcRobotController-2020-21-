package Autonomous.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.io.PrintStream;

import SensorHandlers.LIDARSensor;
import SensorHandlers.LimitSwitch;
import SensorHandlers.SensorPackage;

@Autonomous(name = "SensorLogger", group = "Competition")
public class SensorLogger extends LinearOpMode {
    SensorPackage sensors;
    PrintStream log;

    @Override
    public void runOpMode() {
        log = System.out;
        log.println("DATA,left,back,right");

        sensors = new SensorPackage(new LIDARSensor(hardwareMap.get(DistanceSensor.class, "left"), "left"),
                new LIDARSensor(hardwareMap.get(DistanceSensor.class, "back"), "back"),
                new LIDARSensor(hardwareMap.get(DistanceSensor.class, "right"), "right"),
                new LimitSwitch(hardwareMap.get(TouchSensor.class, "liftReset"), "liftReset"));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double left = sensors.getSensor(LIDARSensor.class, "left").getDistance();
            double back = sensors.getSensor(LIDARSensor.class, "back").getDistance();
            double right = sensors.getSensor(LIDARSensor.class, "right").getDistance();
            telemetry.addData("Left Dist", left);
            telemetry.addData("Back Dist", back);
            telemetry.addData("Right Dist", right);
            telemetry.addData("Lift Reset", sensors.getSensor(LimitSwitch.class, "liftReset").isPressed());
            telemetry.update();

            log.println(String.format("DATA,%g,%g,%g", left, back, right)); // Couldn't help yourself but to use the % formatting, Jordan? -- this was actually Mr. McDonald...
        }

        sensors.kill();
    }
}
