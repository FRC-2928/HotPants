package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command {
	public final Drivetrain drivetrain;
	public final DriverOI oi;

	public Rotation2d absoluteTarget = new Rotation2d();
	public double absoluteTargetMagnitude = 0.5;
	private final PIDController absoluteController = Constants.Drivetrain.absoluteRotationPID.createController();

	public Drive(final Drivetrain drivetrain, final DriverOI oi) {
		this.drivetrain = drivetrain;
		this.oi = oi;

		this.addRequirements(drivetrain);

		this.absoluteController.enableContinuousInput(-0.5, 0.5);
	}

	@Override
	public void execute() {
		final double mul = MathUtil.interpolate(1, 0.5, this.oi.slow.get());

		final double axial = -MathUtil.applyDeadband(this.oi.moveAxial.get(), 0.1);
		final double lateral = MathUtil.applyDeadband(this.oi.moveLateral.get(), 0.1);
		final Rotation2d moveDirection = Rotation2d.fromRadians(Math.atan2(lateral, axial));
		final double moveMagnitude = this.curve(MathUtil.clamp(Math.sqrt(lateral * lateral + axial * axial), 0, 1));

		final double theta;

		if(Constants.Drivetrain.Flags.absoluteRotation) {
			final double rotX = this.oi.moveRotationX.get();
			final double rotY = this.oi.moveRotationY.get();

			this.absoluteTargetMagnitude = Math.sqrt(rotX * rotX + rotY * rotY);

			final boolean command = this.absoluteTargetMagnitude > 0.5;
			if(command) this.absoluteTarget = Rotation2d.fromRadians(Math.atan2(-rotX, rotY));

			this.absoluteTargetMagnitude = this.absoluteTargetMagnitude * 0.5 + 0.5;
			double measurement = Constants.mod(this.drivetrain.gyro.getRotation2d().unaryMinus().getRotations(), 1) - 0.5;
			double setpoint = this.absoluteTarget.getRotations();

			theta = MathUtil
				.applyDeadband(
					MathUtil
						.clamp(
							this.absoluteController
								.calculate(
									Constants.mod(this.drivetrain.gyro.getRotation2d().unaryMinus().getRotations(), 1)
										- 0.5,
									this.absoluteTarget.getRotations()
								),
							-0.5,
							0.5
						),
					command ? 0.075 : 0.25
				);
		} else theta = MathUtil.applyDeadband(this.oi.moveTheta.get(), 0.25);

		ChassisSpeeds desired = new ChassisSpeeds(
			Math.cos(moveDirection.getRadians()) * moveMagnitude * Constants.Drivetrain.axialLateralSpeed * mul,
			Math.sin(moveDirection.getRadians()) * moveMagnitude * Constants.Drivetrain.axialLateralSpeed * mul,
			theta
				* Math.toRadians(Constants.Drivetrain.thetaSpeed)
				* mul
				* (Constants.Drivetrain.Flags.absoluteRotation ? this.absoluteTargetMagnitude : 1)
		);

		if(Constants.Drivetrain.Flags.thetaCompensation) desired = this.drivetrain.compensate(desired);
		if(Constants.Drivetrain.Flags.fod) desired = this.drivetrain.fod(desired);

		this.drivetrain.swerve(this.drivetrain.kinematics.toSwerveModuleStates(desired));
	}

	// Input curve function
	private double curve(final double input) {
		return Math.pow(input, 2); // x^2
	}
}
