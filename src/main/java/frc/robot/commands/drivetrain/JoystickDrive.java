package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends Command {
	public final Drivetrain drivetrain = Robot.cont.drivetrain;
	public final DriverOI oi = Robot.cont.driverOI;

	public Measure<Angle> absoluteTarget = Units.Radians.zero();
	public double absoluteTargetMagnitude = 0.5;
	private final PIDController absoluteController = Constants.Drivetrain.absoluteRotationPID.createController();

	public JoystickDrive() {
		this.addRequirements(this.drivetrain);

		this.absoluteController.enableContinuousInput(-0.5, 0.5);
	}

	@Override
	public void initialize() {
		this.absoluteTarget = Units.Radians.of(this.drivetrain.est.getEstimatedPosition().getRotation().getRadians());
	}

	@Override
	public void execute() {
		final Translation2d translation = this.translation();

		this.drivetrain
			.control(
				new ChassisSpeeds(translation.getX(), translation.getY(), this.theta().in(Units.RadiansPerSecond))
			);
	}

	private Translation2d translation() {
		// get inputs, apply deadbands
		final double axial = MathUtil.applyDeadband(this.oi.moveAxial.get(), 0.1);
		final double lateral = MathUtil.applyDeadband(this.oi.moveLateral.get(), 0.1);

		// cartesian -> polar
		final Rotation2d direction = Rotation2d
			.fromRadians(Math.atan2(axial, lateral))
			.plus(new Rotation2d(Math.PI / 2)); // why?

		// Calculate the move magnitude
		final double magnitude = Math.pow(MathUtil.clamp(Math.hypot(lateral, axial), 0, 1), 2); // get length and normalize

		final double dx = Math.cos(direction.getRadians()) * magnitude * this.oi.slow.get();
		final double dy = Math.sin(direction.getRadians()) * magnitude * this.oi.slow.get();

		// Convert to m/s
		final Measure<Velocity<Distance>> vx = Constants.Drivetrain.maxVelocity.times(dx);
		final Measure<Velocity<Distance>> vy = Constants.Drivetrain.maxVelocity.times(dy);

		return new Translation2d(vx.in(Units.MetersPerSecond), vy.in(Units.MetersPerSecond));
	}

	private Measure<Velocity<Angle>> theta() {
		final double theta;

		if(Constants.Drivetrain.Flags.absoluteRotation) {
			// Joystick Right Axis
			final double rotX = this.oi.moveRotationX.get();
			final double rotY = this.oi.moveRotationY.get();

			// This will determine the rotation speed based on how far the joystick is moved.
			this.absoluteTargetMagnitude = Math.sqrt(rotX * rotX + rotY * rotY);
			Logger.recordOutput("JoystickDrive/AbsoluteRotation/Magnitude", this.absoluteTargetMagnitude);

			// Get a new rotation target if right joystick values are beyond the deadband.
			// Otherwise, we'll keep the old one.
			final boolean rotateRobot = this.absoluteTargetMagnitude > 0.5;
			if(rotateRobot) this.absoluteTarget = Units.Radians.of(Math.atan2(rotX, -rotY));
			Logger.recordOutput("JoystickDrive/AbsoluteRotation/Target", this.absoluteTarget);

			this.absoluteTargetMagnitude = this.absoluteTargetMagnitude * 0.5 + 0.5;

			final double measurement = Constants
				.mod(this.drivetrain.gyroInputs.yawPosition.minus(this.drivetrain.fodOffset).in(Units.Rotations), 1)
				- 0.5;
			final double setpoint = this.absoluteTarget.in(Units.Rotations);

			theta = MathUtil
				.applyDeadband(
					-MathUtil.clamp(this.absoluteController.calculate(measurement, setpoint), -0.5, 0.5), // todo: determine whether this - is ok
					rotateRobot ? 0.075 : 0.25
				);
		} else {
			theta = MathUtil.applyDeadband(this.oi.moveTheta.get(), 0.25);
		}

		return Constants.Drivetrain.maxAngularVelocity
			.times(
				theta
					* this.oi.slow.get()
					* (Constants.Drivetrain.Flags.absoluteRotation ? this.absoluteTargetMagnitude : 1)
			);
	}
}
