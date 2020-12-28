//package DriveEngine.Ultimate;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import Autonomous.Location;
//
//public class UltimateSplineNavigation {
//	private HardwareMap hardwareMap;
//	private SampleMecanumDrive drive;
//
//	public UltimateSplineNavigation(HardwareMap hw) {
//		hardwareMap = hw;
//		drive = new SampleMecanumDrive(hardwareMap);
//	}
//
//	public RobotTrajectory robotTrajectory(Location location) {
//		RobotTrajectory trajectory = new RobotTrajectory();
//		trajectory.setTrajectory(drive.trajectoryBuilder(trajectory.convertLocation(location)));
//		return trajectory;
//	}
//}