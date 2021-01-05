package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;

public class BlinkinLEDController {
	private HardwareMap hardwareMap;
	private PWMOutput LEDDriver;
	
	public BlinkinLEDController(String controllerName, HardwareMap hw) {
		hardwareMap = hw;
		LEDDriver = hardwareMap.pwmOutput.get(controllerName);
	}
	
	public void setOutput(int pulseWidthMicros) {
		if (pulseWidthMicros < 1000 || pulseWidthMicros > 2000) return;
		LEDDriver.setPulseWidthPeriod(pulseWidthMicros);
	}
}