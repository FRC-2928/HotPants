package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix6.hardware.*;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
	public final Pigeon2 gyro = new Pigeon2(Constants.CAN.pigeon, "canivore");

	public final SwerveModule swerveFrontLeft = new SwerveModule(
		SwerveModule.Place.FrontLeft,
		new TalonFX(Constants.CAN.swerveFrontLeftAzimuth, "canivore"),
		new TalonFX(Constants.CAN.swerveFrontLeftDrive, "canivore"),
		new CANcoder(Constants.CAN.swerveFrontLeftEncoder, "canivore"),
		Constants.CAN.swerveFrontLeftOffset
	);
	public final SwerveModule swerveFrontRight = new SwerveModule(
		SwerveModule.Place.FrontRight,
		new TalonFX(Constants.CAN.swerveFrontRightAzimuth, "canivore"),
		new TalonFX(Constants.CAN.swerveFrontRightDrive, "canivore"),
		new CANcoder(Constants.CAN.swerveFrontRightEncoder, "canivore"),
		Constants.CAN.swerveFrontRightOffset
	);
	public final SwerveModule swerveBackLeft = new SwerveModule(
		SwerveModule.Place.BackLeft,
		new TalonFX(Constants.CAN.swerveBackLeftAzimuth, "canivore"),
		new TalonFX(Constants.CAN.swerveBackLeftDrive, "canivore"),
		new CANcoder(Constants.CAN.swerveBackLeftEncoder, "canivore"),
		Constants.CAN.swerveBackLeftOffset
	);
	public final SwerveModule swerveBackRight = new SwerveModule(
		SwerveModule.Place.BackRight,
		new TalonFX(Constants.CAN.swerveBackRightAzimuth, "canivore"),
		new TalonFX(Constants.CAN.swerveBackRightDrive, "canivore"),
		new CANcoder(Constants.CAN.swerveBackRightEncoder, "canivore"),
		Constants.CAN.swerveBackRightOffset
	);
	public final SwerveModule[] modules = {
		this.swerveFrontLeft,
		this.swerveFrontRight,
		this.swerveBackLeft,
		this.swerveBackRight, };

	public final SwerveDriveKinematics kinematics = Constants.Drivetrain.kinematics;
	public final SwerveDrivePoseEstimator est = new SwerveDrivePoseEstimator(
		this.kinematics,
		this.gyro.getRotation2d(),
		this.getModulePositions(),
		new Pose2d()
	);

	public Drivetrain() {
		this.swerveFrontRight.drive.setInverted(true);
		this.swerveBackRight.drive.setInverted(true);
	}

	public SwerveModulePosition[] getModulePositions() {
		return Arrays.stream(this.modules).map(SwerveModule::pos).toArray(SwerveModulePosition[]::new);
	}

	public void swerve(final SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.maxWheelSpeed);
		for(int i = 0; i < this.modules.length; i++)
			this.modules[i].applyState(states[i]);
	}

	public void swerve(final SwerveModule.State state) { this.swerve(state.states); }

	public ChassisSpeeds fod(final ChassisSpeeds field) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(field, this.gyro.getRotation2d().unaryMinus());
	}

	public ChassisSpeeds compensate(final ChassisSpeeds original) {
		// using this function because its just an easy way to rotate the axial/lateral speeds
		return ChassisSpeeds
			.fromFieldRelativeSpeeds(
				original,
				Rotation2d.fromRadians(original.omegaRadiansPerSecond * Constants.Drivetrain.thetaCompensationFactor)
			);
	}

	@Override
	public void periodic() {
		for(final SwerveModule swerve : this.modules)
			swerve.update();

		this.est.update(this.gyro.getRotation2d(), this.getModulePositions());
	}
}
