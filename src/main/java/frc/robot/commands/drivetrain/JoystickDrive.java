package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
		Translation2d linearVelocity = getSpeeds(mul); // Meters per/sec
		SmartDashboard.putNumber("Speeds X", linearVelocity.getX());
		SmartDashboard.putNumber("Speeds Y", linearVelocity.getY());

		double omegaRadPerSec = getTheta(mul); // Radians per/sec
		SmartDashboard.putNumber("Omega", omegaRadPerSec);

		// 2 CONVERT TO CHASSIS SPEEDS
		// Convert to meters per second
		double xMetersPerSec = linearVelocity.getX() * Constants.Drivetrain.maxWheelSpeed;
		double yMetersPerSec = linearVelocity.getY() * Constants.Drivetrain.maxWheelSpeed;

		// ChassisSpeeds desired = new ChassisSpeeds(xMetersPerSec, yMetersPerSec,omegaRadPerSec);
		ChassisSpeeds desired = new ChassisSpeeds(
			linearVelocity.getX(),
			linearVelocity.getY(),
			omegaRadPerSec
		);


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
	 * magnitude of the velocity between 0 and 1.414 
	 * 
	 * @param mul
	 * @return magnitude of the velocity
	 */
	private Translation2d getSpeeds(final double mul) {
		// Left Axis

		// Get joystick inputs and apply deadbands
		final double axial = -MathUtil.applyDeadband(this.oi.moveAxial.get(), 0.1);
		final double lateral = MathUtil.applyDeadband(this.oi.moveLateral.get(), 0.1);

		// Calculate the move direction and magnitude
		final Rotation2d moveDirection = Rotation2d.fromRadians(Math.atan2(lateral, axial));
		final double moveMagnitude = this.curve(MathUtil.clamp(Math.sqrt(lateral * lateral + axial * axial), 0, 1));

		double xSpeed = Math.cos(moveDirection.getRadians()) * moveMagnitude * Constants.Drivetrain.axialLateralSpeed * mul;
		double ySpeed = Math.sin(moveDirection.getRadians()) * moveMagnitude * Constants.Drivetrain.axialLateralSpeed * mul;

		return new Translation2d(xSpeed, ySpeed);
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

			// Constants.mod(this.drivetrain.gyro.getRotation2d().unaryMinus().getRotations(), 1)
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

		return theta * Math.toRadians(Constants.Drivetrain.thetaSpeed)
					* mul
					* (Constants.Drivetrain.Flags.absoluteRotation ? this.absoluteTargetMagnitude : 1);
	}

	// Input curve function
	private double curve(final double input) {
		return Math.pow(input, 2); // x^2
	}

	// private Translation2d getLinearVelocity() {
	// 	// Apply deadband
    //       double linearMagnitude =
    //           MathUtil.applyDeadband(
    //               Math.hypot(this.oi.moveAxial.get(), this.oi.moveLateral.get()), 0.1);
    //       Rotation2d linearDirection =
    //           new Rotation2d(-this.oi.moveAxial.get(), this.oi.moveLateral.get());
          
    //       // Square values
    //       linearMagnitude = linearMagnitude * linearMagnitude;
	// 	  SmartDashboard.putNumber("Magnitude", linearMagnitude);

    //       // Calcaulate new linear velocity
    //       Translation2d linearVelocity =
    //           new Pose2d(new Translation2d(), linearDirection)
    //               .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
    //               .getTranslation();

	// 	return linearVelocity;
	// }

	// private double getOmega() {
	// 	double omega = MathUtil.applyDeadband(this.oi.moveTheta.get(), 0.1);

	// 	// Square values
	// 	omega = Math.copySign(omega * omega, omega);
	// 	return omega;
	// }
}
