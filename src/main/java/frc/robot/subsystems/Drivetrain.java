package frc.robot.subsystems;

import java.util.Arrays;

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

	public SwerveModulePosition[] getModulePositions() {
		return Arrays.stream(this.modules).map(SwerveModule::updateModulePosition).toArray(SwerveModulePosition[]::new);
	}

	public void swerve(final SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.maxWheelSpeed);
		for(int i = 0; i < this.modules.length; i++)
			this.modules[i].applyState(states[i]);
	}

	public void swerve(final SwerveModule.State state) { this.swerve(state.states); }

	public ChassisSpeeds fod(final ChassisSpeeds field) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(field, this.gyroInputs.yawPosition);
	}

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
	
	public double getGyroRotations() {
		return gyroInputs.rotations;
	}

	@Override
	public void periodic() {
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);
		for(final SwerveModule swerve : this.modules)
			swerve.update();

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
}
