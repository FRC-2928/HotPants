package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends Command {
	public final Drivetrain drivetrain;
	public final DriverOI oi;

	public Rotation2d absoluteTarget = new Rotation2d();
	public double absoluteTargetMagnitude = 0.5;
	private final PIDController absoluteController = Constants.Drivetrain.absoluteRotationPID.createController();

	private final SlewRateLimiter xLimiter = new SlewRateLimiter(100.0);
	private final SlewRateLimiter yLimiter = new SlewRateLimiter(100.0);
	private final SlewRateLimiter turnLimiter = new SlewRateLimiter(100.0);

	public JoystickDrive(final Drivetrain drivetrain, final DriverOI oi) {
		this.drivetrain = drivetrain;
		this.oi = oi;

		this.addRequirements(drivetrain);

		this.absoluteController.enableContinuousInput(-0.5, 0.5);
	}

	@Override
	public void execute() {
		final double mul = MathUtil.interpolate(1, 0.5, this.oi.slow.get());

		// 1. CONVERT JOYSTICK VALUES
		Translation2d linearVelocity = getLinearVelocity(mul); // Meters per/sec
		SmartDashboard.putNumber("Linear Velocity X", linearVelocity.getX());
		SmartDashboard.putNumber("Linear Velocity Y", linearVelocity.getY());

		double omegaRadPerSec = getTheta(mul); // Radians per/sec
		SmartDashboard.putNumber("Omega", omegaRadPerSec);

		// 2 CONVERT TO CHASSIS SPEEDS	
		ChassisSpeeds desired = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omegaRadPerSec);

		// Compensate for wheel rotation while driving and rotating. Rotate 0.35 radians (20 degrees).
		if(Constants.Drivetrain.Flags.thetaCompensation) desired = this.drivetrain.compensate(desired);

		// 3. CONVERT FROM FIELD RELATIVE SPEED TO ROBOT RELATIVE CHASSIS SPEEDS
		if(Constants.Drivetrain.Flags.fod) desired = this.drivetrain.fieldOrientedDrive(desired);

		// 4. CONVERT CHASSIS SPEEDS TO MODULE SPEEDS
		// ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(desired, 0.02);
		SwerveModuleState[] setpointStates = this.drivetrain.kinematics.toSwerveModuleStates(desired);

		// Set the required speed and angle of each wheel.
		this.drivetrain.setModuleStates(setpointStates);
	}

	/**
	 * 
	 * @param mul
	 * @return the velocity in meters per/sec
	 */
	private Translation2d getLinearVelocity(final double mul) {
		// Left Axis

		// Get joystick inputs and apply deadbands
		final double axial = -MathUtil.applyDeadband(this.oi.moveAxial.get(), 0.1);
		final double lateral = MathUtil.applyDeadband(this.oi.moveLateral.get(), 0.1);

		// Get the angle theta from the conversion of rectangular coordinates to polar coordinates
		final Rotation2d moveDirection = Rotation2d.fromRadians(Math.atan2(lateral, axial));
		// final Rotation2d moveDirection = new Rotation2d(this.oi.moveAxial.get(), this.oi.moveLateral.get());

		// Calculate the move magnitude
		double magnitude = Math.hypot(lateral, axial);
		magnitude = MathUtil.clamp(magnitude, 0, 1); // Make it a unit value?
		final double moveMagnitude = magnitude * magnitude; // Square values
		
		double xSpeed = Math.cos(moveDirection.getRadians()) * moveMagnitude * mul;
		double ySpeed = Math.sin(moveDirection.getRadians()) * moveMagnitude * mul;

		// Convert to meters per/sec
		double vxMetersPerSecond = xSpeed * Constants.Drivetrain.maxVelocityMetersPerSec;
		double vyMetersPerSecond = ySpeed * Constants.Drivetrain.maxVelocityMetersPerSec;

		return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
	}

	private double getTheta(final double mul) {
		// Right Axis
		final double theta;
		if(Constants.Drivetrain.Flags.absoluteRotation) {
			final double rotX = this.oi.moveRotationX.get();
			final double rotY = this.oi.moveRotationY.get();

			this.absoluteTargetMagnitude = Math.sqrt(rotX * rotX + rotY * rotY);

			final boolean command = this.absoluteTargetMagnitude > 0.5;
			if(command) this.absoluteTarget = Rotation2d.fromRadians(Math.atan2(-rotX, rotY));

			this.absoluteTargetMagnitude = this.absoluteTargetMagnitude * 0.5 + 0.5;

			double measurement = Constants.mod(this.drivetrain.getGyroRotations(),1) - 0.5;
			double setpoint = this.absoluteTarget.getRotations();
			theta = MathUtil.applyDeadband(
						MathUtil
							.clamp(this.absoluteController.calculate(measurement, setpoint), -0.5, 0.5),
						command ? 0.075 : 0.25
					);
		} else {
			theta = MathUtil.applyDeadband(this.oi.moveTheta.get(), 0.25);
		}
		
		// theta = this.turnLimiter.calculate(theta);

		double omega = theta * mul
					* (Constants.Drivetrain.Flags.absoluteRotation ? this.absoluteTargetMagnitude : 1);

		return omega * Constants.Drivetrain.maxAngularVelocityRadPerSec;
	}

	// Square values
	// private double curve(final double input) {
	// 	return Math.pow(input, 2); // x^2
	// }

}
