package frc.robot.vision;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.vision.LimelightHelpers.LimelightResults;

public class Limelight {
	private final NetworkTable nt;
	private final String limelightName;

	public Limelight(final String limelightName) {
		this.nt = NetworkTableInstance.getDefault().getTable(limelightName);
		this.limelightName = limelightName;
		this.setStream(0);
	}

	public void setStream(final int stream) { this.nt.getEntry("stream").setNumber(stream); }

	public void setPipeline(final int pipeline) { this.nt.getEntry("pipeline").setNumber(pipeline); }

	// Whether the limelight has any valid targets (0 or 1)
	public boolean hasValidTargets() {
		if(RobotBase.isReal()) {
			return this.nt.getEntry("tv").getDouble(0) == 1;
		} else {
			// return this value in simulation
			return true;
		}
	}

	public LimelightResults getResults() { return LimelightHelpers.getLatestResults(this.limelightName); }

	public int getTargetAprilTagID() { return (int) LimelightHelpers.getFiducialID(this.limelightName); }

	// Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
	//@AutoLogOutput(key = "Limelight/Horizontal Offset")
	public Measure<Angle> getTargetHorizontalOffset() { return Units.Degrees.of(this.nt.getEntry("tx").getDouble(0)); }

	// Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
	//@AutoLogOutput(key = "Limelight/Vertical Offset")
	public Measure<Angle> getTargetVerticalOffset() { return Units.Degrees.of(this.nt.getEntry("ty").getDouble(0)); }

	public int getNumberOfAprilTags() {
		final LimelightResults reultsOfJson = LimelightHelpers.getLatestResults(this.limelightName);
		return reultsOfJson.targetingResults.targets_Fiducials.length;
	}

	// Target Area (0% of image to 100% of image)
	public double getTargetArea() { return this.nt.getEntry("ta").getDouble(0); }

	public double getTargetSkew() { return this.nt.getEntry("ts").getDouble(0); }

	// Robot transform in 3D field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
	public Pose3d getPose3d() { return LimelightHelpers.getBotPose3d(this.limelightName); }

	// Robot transform in 2D field-space. Translation (X,Y) Rotation(Z)
	//@AutoLogOutput(key = "Odometry/Limelight")
	public Pose2d getPose2d() {
		// Pose2d botPose = getBotPose2d().relativeTo(new Pose2d(-8.27, -4.105, new Rotation2d()));
		// This should do the same thing as the commented out line above, without need for manual coordinate transformation
		if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			return LimelightHelpers.getBotPose2d_wpiRed(this.limelightName);
		} else {
			return LimelightHelpers.getBotPose2d_wpiBlue(this.limelightName);
		}
	}

	//@AutoLogOutput(key = "Odometry/BotPose")
	public Pose2d getBotPose2d() { return LimelightHelpers.getBotPose2d(this.limelightName); }

	// Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
	public Pose3d getBluePose3d() { return LimelightHelpers.getBotPose3d_wpiBlue(this.limelightName); }

	public Pose2d getBluePose2d() { return LimelightHelpers.getBotPose2d_wpiBlue(this.limelightName); }

	public Pose2d getRedPose2d() { return LimelightHelpers.getBotPose2d_wpiRed(this.limelightName); }

	// Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
	public Pose3d getRedPose3d() { return LimelightHelpers.getBotPose3d_wpiRed(this.limelightName); }

	// 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
	//@AutoLogOutput(key = "limelight/Pose3d")
	public Pose3d getRobotTagPose3d() { return LimelightHelpers.getTargetPose3d_RobotSpace(this.limelightName); }

	// 3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
	public Pose3d getCameraTagPose3d() { return LimelightHelpers.getTargetPose3d_CameraSpace(this.limelightName); }
}
