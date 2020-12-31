package DriveEngine.Ultimate;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import Autonomous.Location;

public class RobotTrajectory {
	private boolean isBuilt;
	private TrajectoryBuilder trajectoryBuilder;
	private Trajectory trajectory;
	
	public RobotTrajectory() {
		isBuilt = false;
	}
	
	public void setTrajectory(TrajectoryBuilder builder) {
		if (isBuilt) return;
		trajectoryBuilder = builder;
	}
	
	public RobotTrajectory moveToLocation(Location endLocation) {
		if (isBuilt) return this;
		Pose2d endPose = endLocation.convertToPose2d();
		trajectoryBuilder = trajectoryBuilder.lineToLinearHeading(endPose);
		return this;
	}
	
	public RobotTrajectory splineToLocation(Location endLocation, double endMovementHeading) {
		if (isBuilt) return this;
		Pose2d endPose = endLocation.convertToPose2d();
		double endMovement = Math.toRadians(-endMovementHeading);
		trajectoryBuilder = trajectoryBuilder.splineToLinearHeading(endPose, endMovement);
		return this;
	}
	
	public RobotTrajectory build() {
		if (isBuilt) return this;
		trajectory = trajectoryBuilder.build();
		isBuilt = true;
		return this;
	}
	
	public Trajectory getTrajectory() { return trajectory; }
}