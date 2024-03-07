package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import frc.robot.Constants;

public class SwerveModule {
	public static enum Place {
		FrontLeft(0), FrontRight(1), BackLeft(2), BackRight(3);

		private Place(final int index) { this.index = index; }

		public final int index;
	}

	public SwerveModule(final Place place) {
		this.place = place;

		this.io = switch(Constants.mode) {
		case REAL -> new ModuleIOReal(this);
		case SIM -> new ModuleIOSim();
		case REPLAY -> new ModuleIO() {
		};
		default -> throw new Error();
		};

		this.azimuthPID.enableContinuousInput(-180, 180);
	}

	public final Place place;
	public final ModuleIO io;
	public final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

	private final PIDController azimuthPID = Constants.Drivetrain.swerveAzimuthPID.createController();
	private final SimpleMotorFeedforward driveFFW = Constants.Drivetrain.driveFFW;
	private final PIDController drivePID = Constants.Drivetrain.drivePID.createController();

	public SwerveModulePosition position = new SwerveModulePosition();
	public SwerveModuleState current = new SwerveModuleState();
	public SwerveModuleState desired = new SwerveModuleState();

	public Measure<Distance> drivePosition() {
		return Units.Meters
			.of(
				this.inputs.drivePosition.in(Units.Rotations) * Constants.Drivetrain.wheelCircumference.in(Units.Meters)
			);
	}

	public void halt() {
		this.io.setAzimuthVoltage(0);
		this.io.setDriveVoltage(0);
	}

	private void azimuth(final Measure<Angle> desired) {
		// Calculate azimuth power required to reach the setpoint
		final double azimuth = this.azimuthPID
			.calculate(this.inputs.angle.in(Units.Degrees), desired.in(Units.Degrees));

		// Restrict the azimuth power and reverse the direction
		final double azimuthVolts = MathUtil.clamp(-azimuth, -10, 10);

		// inputs.azimuthAppliedVolts will track the applied voltage
		this.io.setAzimuthVoltage(azimuthVolts);
	}

	private void drive(final Measure<Velocity<Distance>> speed) {
		// Calculate drive power
		final double ffw = this.driveFFW.calculate(speed.in(Units.MetersPerSecond));
		final double output = this.drivePID
			.calculate(this.inputs.driveVelocity.in(Units.MetersPerSecond), speed.in(Units.MetersPerSecond));
		final double driveVolts = MathUtil.clamp(ffw + output, -10, 10);

		// inputs.driveAppliedVolts will track the applied voltage
		this.io.setDriveVoltage(driveVolts);
	}

	public void control(final SwerveModuleState state) {
		state.speedMetersPerSecond = -state.speedMetersPerSecond;

		if(Math.abs(state.angle.minus(Rotation2d.fromDegrees(this.inputs.angle.in(Units.Degrees))).getDegrees()) > 90) {
			state.speedMetersPerSecond = -state.speedMetersPerSecond;
			state.angle = state.angle.rotateBy(Rotation2d.fromDegrees(180.0));
		}

		this.azimuth(Units.Degrees.of(state.angle.getDegrees()));
		this.drive(Units.MetersPerSecond.of(state.speedMetersPerSecond));

		this.desired = state;
	}

	public void runCharacterization(final double volts) {
		this.azimuth(Units.Degrees.of(0));
		this.io.setDriveVoltage(volts);
	}

	public void periodic() {
		this.io.updateInputs(this.inputs);
		Logger.processInputs("Drivetrain/" + this.place.name(), this.inputs);

		this.position = new SwerveModulePosition(
			this.drivePosition().in(Units.Meters),
			new Rotation2d(this.inputs.angle)
		);
		this.current = new SwerveModuleState(
			this.inputs.driveVelocity.in(Units.MetersPerSecond),
			new Rotation2d(this.inputs.angle)
		);
	}
}
