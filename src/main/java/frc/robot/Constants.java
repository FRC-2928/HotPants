package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.*;

public class Constants {
	private static Mode currentMode() {
		if(Robot.isReal()) return Mode.REAL;
		if(!Logger.hasReplaySource()) return Mode.SIM;
		else return Mode.REPLAY;
	}

	private Constants() { throw new IllegalCallerException("Cannot instantiate `Constants`"); }

	public static final Mode mode = Constants.currentMode();
	public static final boolean real = Constants.mode == Constants.Mode.REAL;

	public static final Measure<Distance> fieldWidth = Units.Meters.of(16.55445); // Correlates to Field oriented x coordinate
	public static final Measure<Distance> fieldDepth = Units.Meters.of(8.21); // Correlates to Field oriented y coordinate

	public static enum Mode {
		/** Running on a real robot. */
		REAL,

		/** Running a physics simulator. */
		SIM,

		/** Replaying from a log file. */
		REPLAY
	}

	public static final double mod(final double lhs, final double rhs) { return (lhs % rhs + rhs) % rhs; }

	public static final double angleNorm(final double angle) { return Constants.mod(angle + 180, 360) - 180; }

	public static final double angleDistance(final double a, final double b) {
		return Math
			.abs(Constants.angleNorm(180 - Math.abs(Math.abs(Constants.angleNorm(a) - Constants.angleNorm(b)) - 180)));
	}

	public static record PIDValues(double p, double i, double d, double f) {
		public final PIDController createController() { return new PIDController(this.p, this.i, this.d); }
	}

	public static record Ratio<U extends Unit<U>>(double factor) {
		public Ratio(final Measure<U> from, final Measure<U> to) {
			this(to.baseUnitMagnitude() / from.baseUnitMagnitude());
		}

		public Ratio(final double from, final double to) { this(to / from); }

		public final Measure<U> forward(final Measure<U> value) { return value.times(this.factor); }

		public final Measure<U> inverse(final Measure<U> value) { return value.divide(this.factor); }

		public final Ratio<U> inverse() { return new Ratio<U>(1 / this.factor); }
	}

	public static final class CAN {
		private CAN() { throw new IllegalCallerException("Cannot instantiate `Constants.CAN`"); }

		public static final class Misc {
			public static final int pdh = 0;

			public static final int feederLauncher = 0;
			public static final int intakeRoller = 3;
		}

		public static final class CTRE {
			public static final String bus = "canivore";

			public static final int pigeon = 0;

			public static final int swerveFrontLeftAzimuth = 13;
			public static final int swerveFrontLeftDrive = 14;

			public static final int swerveFrontRightAzimuth = 19;
			public static final int swerveFrontRightDrive = 18;

			public static final int swerveBackLeftAzimuth = 10;
			public static final int swerveBackLeftDrive = 11;

			public static final int swerveBackRightAzimuth = 15;
			public static final int swerveBackRightDrive = 16;

			public static final int shooterPivot = 12;
			public static final int shooterEncoder = 12;
			public static final int shooterFlywheels = 1;

			public static final int climber = 17;
		}
	}

	public static final class PWM {
		private PWM() { throw new IllegalCallerException("Cannot instantiate `Constants.PWM`"); }

		public static final int climberRatchet = 9;
	}

	public static final class DIO {
		private DIO() { throw new IllegalCallerException("Cannot instantiate `Constants.DIO`"); }

		public static final int release = 0;
		public static final int lockout = 1;
	}

	public static final class Drivetrain {
		private Drivetrain() { throw new IllegalCallerException("Cannot instantiate `Constants.Drivetrain`"); }

		public static final class Flags {
			private Flags() { throw new IllegalCallerException("Cannot instantiate `Constants.Drivetrain.Flags`"); }

			/// Field-oriented drive
			public static final boolean fod = true;
			/// Absolute rotation (point right stick in direction to face)
			public static final boolean absoluteRotation = true;

			/// Optimize wheel rotation to only rotate less than 90deg per turn
			public static final boolean wheelOptimization = true;
			/// Compensate for wheel rotation while driving and rotating
			public static final boolean thetaCompensation = false;
		}

		public static final class Choreo {
			public static final PIDValues x = new PIDValues(0.1, 0, 0, 0);
			public static final PIDValues y = new PIDValues(0.1, 0, 0, 0);
			public static final PIDValues theta = new PIDValues(0.5, 0, 0, 0);
		}

		/* TORQUE-based velocity does not require a feed forward, as torque will accelerate the
			rotor up to the desired velocity by itself */
		// kP = 5 An error of 1 rotation per second results in 5 amps output
		// kI = 0.1 An error of 1 rotation per second increases output by 0.1 amps every second
		// kD = 0.001 A change of 1000 rotation per second squared results in 1 amp output
		public static final Slot1Configs driveGainsSlot1 = new Slot1Configs()
			.withKP(5)
			.withKI(0.1)
			.withKD(0.001)
			.withKS(0)
			.withKV(0)
			.withKA(0);

		/* VOLTAGE-based velocity requires a feed forward to account for the back-emf of the motor */
		// kP = 0.11 An error of 1 rotation per second results in 2V output
		// kI = 0.5 An error of 1 rotation per second increases output by 0.5V every second
		// kD = 0.0001 A change of 1 rotation per second squared results in 0.01 volts output
		// 0.12 Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
		public static final Slot0Configs driveGainsSlot0 = new Slot0Configs()
			.withKP(0.11)
			.withKI(0.5)
			.withKD(0.0001)
			.withKS(0)
			.withKV(0.12)
			.withKA(0);
		// .withKP(3).withKI(0).withKD(0)
		// .withKS(0).withKV(0).withKA(0);

		// todo: tune
		public final static PIDValues drivePID = new PIDValues(0.1, 0, 0, 0);
		// todo: tune
		public static final PIDValues swerveAzimuthPID = new PIDValues(0.1, 0.01, 0.003, 0);
		//public static final PIDValues swerveAzimuthPID = new PIDValues(0.01, 0, 0.005, 0);
		public static final PIDValues absoluteRotationPID = new PIDValues(1.8, 0.01, 0, 0);
		// todo: find
		public static final SimpleMotorFeedforward driveFFW = new SimpleMotorFeedforward(0, 4, 0);

		public static final PIDValues targetVerticalControllerPID = new PIDValues(0.2, 0, 0, 0);
		public static final PIDValues targetHorizontalControllerPID = new PIDValues(0.2, 0, 0, 0);
		public static final PIDValues visionAbsoluteRotationErrorPID = new PIDValues(1, 0, 0, 0);

		public static final double thetaCompensationFactor = 0.1;

		public static final Measure<Distance> wheelBase = Units.Inches.of(29 - 2.5 * 2);
		public static final Measure<Distance> trackWidth = Drivetrain.wheelBase; // For a square drivetrain`

		public static final Measure<Angle> swerveFrontLeftOffset = Units.Rotations.of(-0.42138671875);
		public static final Translation2d swerveFrontLeftTranslation = new Translation2d(
			Constants.Drivetrain.wheelBase,
			Constants.Drivetrain.trackWidth
		);
		public static final Measure<Angle> swerveFrontRightOffset = Units.Rotations.of(0.2978515625);
		public static final Translation2d swerveFrontRightTranslation = new Translation2d(
			Constants.Drivetrain.wheelBase,
			Constants.Drivetrain.trackWidth.negate()
		);
		public static final Measure<Angle> swerveBackLeftOffset = Units.Rotations.of(0.027587890625);
		public static final Translation2d swerveBackLeftTranslation = new Translation2d(
			Constants.Drivetrain.wheelBase.negate(),
			Constants.Drivetrain.trackWidth
		);
		public static final Measure<Angle> swerveBackRightOffset = Units.Rotations.of(-0.4169921875);
		public static final Translation2d swerveBackRightTranslation = new Translation2d(
			Constants.Drivetrain.wheelBase.negate(),
			Constants.Drivetrain.trackWidth.negate()
		);

		public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			Constants.Drivetrain.swerveFrontLeftTranslation,
			Constants.Drivetrain.swerveFrontRightTranslation,
			Constants.Drivetrain.swerveBackLeftTranslation,
			Constants.Drivetrain.swerveBackRightTranslation
		);

		// Gear ratios for SDS MK4i L2, adjust as necessary
		public static final double driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // ~= 6.746
		public static final double azimuthGearRatio = 150.0 / 7.0;

		public static final Measure<Distance> wheelRadius = Units.Inches.of(2);
		public static final Measure<Distance> wheelCircumference = Drivetrain.wheelRadius.times(2 * Math.PI);

		public static final Measure<Velocity<Distance>> maxVelocity = Units.Meters.per(Units.Second).of(5);

		// max angular velocity computes to 6.41 radians per second
		public static final Measure<Velocity<Angle>> maxAngularVelocity = Units.RotationsPerSecond
			.of(
				Drivetrain.maxVelocity.in(Units.MetersPerSecond)
					/ (2
						* Math.PI
						* Math
							.hypot(
								Drivetrain.trackWidth.divide(2).in(Units.Meters),
								Drivetrain.wheelBase.divide(2).in(Units.Meters)
							))
			);
	}

}
