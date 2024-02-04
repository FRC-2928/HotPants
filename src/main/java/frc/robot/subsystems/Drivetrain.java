package frc.robot.subsystems;

import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule.Place;
import frc.robot.vision.Limelight;

public class Drivetrain extends SubsystemBase {
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
	public final SwerveDriveKinematics kinematics = Constants.Drivetrain.kinematics;

	// Used to track odometry
	public final SwerveDrivePoseEstimator poseEstimator;
	private final Limelight limelight = new Limelight("limelight");
	private double offset;
	private MedianFilter filterVertical = new MedianFilter(10);

	// ----------------------------------------------------------
    // Initialization
    // ----------------------------------------------------------
	public Drivetrain(
		GyroIO gyroIO,
		ModuleIO flModuleIO,
		ModuleIO frModuleIO,
		ModuleIO blModuleIO,
		ModuleIO brModuleIO) 
  	{	
		if (gyroIO == null) gyroIO = new GyroIOSim(this);
		this.gyroIO = gyroIO;
		modules[0] = new SwerveModule(flModuleIO, Place.FrontLeft);
		modules[1] = new SwerveModule(frModuleIO, Place.FrontRight);
		modules[2] = new SwerveModule(blModuleIO, Place.BackLeft);
		modules[3] = new SwerveModule(brModuleIO, Place.BackRight);

		poseEstimator = new SwerveDrivePoseEstimator( this.kinematics,
													this.gyroInputs.yawPosition,
													this.getModulePositions(),
													new Pose2d() 	
													);
	}

	// ----------------------------------------------------------
    // Control Input
    // ----------------------------------------------------------

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed Speed of the robot in the x direction (forward).
	 * @param ySpeed Speed of the robot in the y direction (sideways).
	 * @param rot Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 */
	public void drive(double vxMetersPerSecond, double vyMetersPerSecond, 
					  double omegaRadiansPerSecond, boolean fieldRelative) {
		
		SmartDashboard.putNumber("xSpeed", vxMetersPerSecond);
		SmartDashboard.putNumber("ySpeed", vyMetersPerSecond);
		SmartDashboard.putNumber("rotation", omegaRadiansPerSecond);

		var swerveModuleStates =
			kinematics.toSwerveModuleStates(
				fieldRelative
					? ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, getHeading())
					: new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond));

		setModuleStates(swerveModuleStates);
	}

	/**
	 * Set the required speed and angle of each wheel.
	 * 
	 * @param states - required speed in meters per/sec
	 * 				 - angle in radians per/sec
	*/
	public void setModuleStates(final SwerveModuleState[] states) {
		// 5. DESATURATE WHEEL SPEEDS
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.maxVelocityMetersPerSec);

		// 6. SET SPEED AND ANGLE FOR EACH WHEEL
		for(int i = 0; i < this.modules.length; i++)
			this.modules[i].applyState(states[i]);
	}

	public void setModuleStates(final SwerveModule.State state) { this.setModuleStates(state.states); }

	// Convert field-relative ChassisSpeeds to robot-relative ChassisSpeeds.
	public ChassisSpeeds fieldOrientedDrive(final ChassisSpeeds field) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(field, getHeading().unaryMinus());
	}

	// Compensate for wheel rotation while driving and rotating
	public ChassisSpeeds compensate(final ChassisSpeeds original) {
		// using this function because its just an easy way to rotate the axial/lateral speeds
		return ChassisSpeeds
			.fromFieldRelativeSpeeds(
				original,
				Rotation2d.fromRadians(original.omegaRadiansPerSecond * Constants.Drivetrain.thetaCompensationFactor)
			);
	}
  
	public void resetGyro() {
		gyroIO.resetGyro();
	}
	
	// ----------------------------------------------------------
    // Control Input
    // ----------------------------------------------------------
	/**
	 * The angle is continuous; that is, it will continue from 360 to
     * 361 degrees. 
	 * 
	 * @return heading of the robot as a Rotation2d.
	 */
	@AutoLogOutput(key = "Gyro/Heading")
	public Rotation2d getHeading() {
		return this.gyroInputs.heading;
	}

	/**
	 * Current reported yaw of the Pigeon2 in degrees
	 * Constructs and returns a Rotation2d
	 * 
	 * @return current angle of the robot
	 */
	@AutoLogOutput(key = "Gyro/YawPosition")
	public Rotation2d getRobotAngle() {
		return this.gyroInputs.yawPosition;
	}

	/**
	 * Takes the negative of the current angular value
	 * and converts given radians to rotations.
	 * 
	 * @return the continuous rotations and partial rotations
	 */
	@AutoLogOutput(key = "Gyro/Rotations")
	public double getGyroRotations() {
		return getHeading().unaryMinus().getRotations();
	}

	public SwerveModule[] getSwerveModules() {return this.modules;}

	public SwerveDriveKinematics getKinematics() {return this.kinematics;}

	// ----------------------------------------------------------
    // Vision
    // ----------------------------------------------------------

	// Robot transform in 3D field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
	// using "botpose"
	public Pose3d getLimelightPose3d() {
		return this.limelight.getPose3d();
	}

	// Robot transform in 2D field-space. Translation (X,Y) Rotation(Z)
	// using "botpose"
	public Pose2d getLimelightPose2d() {
		return this.limelight.getPose2d();
	}

	// Robot transform in field-space with the alliance driverstation at the origin
	// using botpose_wpired and botpose_wpiblue
	public Pose2d getLimelightPoseRelative() {
		if(RobotBase.isReal()) {
			// if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
			// 	return this.limelight.getRedPose2d();
			// } else {
				return this.limelight.getBluePose2d();
			// }
		} else {
			// In simulation we just return the encoder pose.
			return this.getPoseEstimation();
		}
	}

	public Pose2d getLimelightPoseBlue(){
		return this.limelight.getBluePose2d();
	}

	/**
	 * Gets the angle of the target relative to the robot
	 * @return offset angle between target and the robot
	 */
	public double getTargetHorizontalOffset() {
		return this.limelight.getTargetHorizontalOffset();
	}

	public double getTargetVerticalOffset() {
		if(this.limelight.getTargetVerticalOffset() != 0) {
			this.offset = this.limelight.getTargetVerticalOffset();
		}
		return this.filterVertical.calculate(this.offset);
	}

	public boolean hasValidLimelightTarget() {
		if(RobotBase.isReal()) {
			return this.limelight.hasValidTargets();
		} else {
			return this.getHasValidTargetsSim();
		}
	}

	@AutoLogOutput(key = "AprilTag/id")
	public int getAprilTagID() {
		if(RobotBase.isReal()) {
			return this.limelight.getTargetAprilTagID();
		} else {
			return 8;
		}
	}

	// ----------------------------------------------------------
    // Odometry
    // ----------------------------------------------------------
	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		this.poseEstimator.resetPosition(
			this.gyroInputs.yawPosition,
			this.getModulePositions(),
			pose);
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Estimation")
	public Pose2d getPoseEstimation() {
		return this.poseEstimator.getEstimatedPosition();
	}

	public SwerveModulePosition[] getModulePositions() {
		return Arrays.stream(this.modules).map(SwerveModule::updateModulePosition).toArray(SwerveModulePosition[]::new);
	}

	// ----------------------------------------------------------
    // Process Logic
    // ----------------------------------------------------------
	@Override
	public void periodic() {
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);

		for(final SwerveModule module : this.modules)
			module.update();

		// Stop moving when disabled
		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
		}

		// Log empty setpoint states when disabled
		if (DriverStation.isDisabled()) {
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
			
		}
		
		this.poseEstimator.update(this.gyroInputs.yawPosition, this.getModulePositions());

		if(this.limelight.hasValidTargets()) {
			this.poseEstimator.addVisionMeasurement(this.getLimelightPose2d(), Timer.getFPGATimestamp() - 0.3);
		}

		Pose2d botPose = this.limelight.getPose2d();
	}

	// ----------------------------------------------------------
    // Simulation
    // ----------------------------------------------------------
	public boolean getHasValidTargetsSim() {
		double heading = this.getPoseEstimation().getRotation().getDegrees();

		// if(DriverStation.getAlliance() == DriverStation.Alliance.Red) return heading < 75 || heading > -75;
		// else 
		return heading > 135 || heading < -135;
	}

}
