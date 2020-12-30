package DriveEngine.Ultimate;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Autonomous.Location;
import drive.SampleMecanumDrive;

public class UltimateSplineNavigation {
	private HardwareMap hardwareMap;
	private SampleMecanumDrive drive;

	public UltimateSplineNavigation(HardwareMap hw) {
		hardwareMap = hw;
		drive = new SampleMecanumDrive(hardwareMap);
	}

	public RobotTrajectory robotTrajectory(Location location) {
		RobotTrajectory trajectory = new RobotTrajectory();
		trajectory.setTrajectory(drive.trajectoryBuilder(location.convertToPose2d()));
		return trajectory;
	}
	
	public void followTrajectory(RobotTrajectory trajectory) {
		drive.followTrajectory(trajectory.getTrajectory());
	}
}