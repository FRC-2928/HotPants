package frc.robot.subsystems;

import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule.Place;

public class Drivetrain extends SubsystemBase {
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
	public final SwerveDriveKinematics kinematics = Constants.Drivetrain.kinematics;
	public final SwerveDrivePoseEstimator poseEstimator;

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
	 * Set the required speed and angle of each wheel.
	 * 
	 * @param states - required speed in meters per/sec
	 * 				 - angle in degrees
	*/
	public void setModuleStates(final SwerveModuleState[] states) {
		// 6. DESATURATE WHEEL SPEEDS
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.maxWheelSpeed);

		// 7. SET SPEED AND ANGLE FOR EACH WHEEL
		// Also calculates the feedforward for the drive velocity
		for(int i = 0; i < this.modules.length; i++)
			this.modules[i].applyState(states[i]);
	}

	public void setModuleStates(final SwerveModule.State state) { this.setModuleStates(state.states); }

	// Field-oriented drive
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
	 * 
	 * @return the number of times the robot has rotated thru 360 degrees
	 */
	@AutoLogOutput(key = "Gyro/Rotations")
	public double getGyroRotations() {
		return getHeading().unaryMinus().getRotations();
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

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Estimation")
	public Pose2d getPoseEstimation() {
		return this.poseEstimator.getEstimatedPosition();
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
		// this.est.update(this.gyro.getRotation2d(), this.getModulePosition());
		this.poseEstimator.update(this.gyroInputs.yawPosition, this.getModulePositions());
	}

	// ----------------------------------------------------------
    // Odometry
    // ----------------------------------------------------------
	public SwerveModulePosition[] getModulePositions() {
		return Arrays.stream(this.modules).map(SwerveModule::updateModulePosition).toArray(SwerveModulePosition[]::new);
	}

}
