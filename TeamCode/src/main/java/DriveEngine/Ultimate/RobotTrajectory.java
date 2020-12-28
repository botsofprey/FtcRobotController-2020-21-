package DriveEngine.Ultimate;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import Autonomous.Location;

public class RobotTrajectory {
	private boolean isBuilt;
	private TrajectoryBuilder trajectoryBuilder;
	private Trajectory trajectory;
	
	public Pose2d convertLocation(Location location) {
		double x = location.getY();
		double y = -location.getX();
		double heading = Math.toRadians(-location.getHeading());
		return new Pose2d(x, y, heading);
	}
	
	public RobotTrajectory() {
		isBuilt = false;
	}
	
	public void setTrajectory(TrajectoryBuilder builder) {
		trajectoryBuilder = builder;
	}//todo finish this class
}