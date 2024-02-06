package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
	private Constants() { throw new IllegalCallerException("Cannot instantiate `Constants`"); }

	public static Mode currentMode = Mode.SIM;

	public static enum Mode {
		/** Running on a real robot. */
		REAL,

		/** Running a physics simulator. */
		SIM,

		/** Replaying from a log file. */
		REPLAY
	}
	// public static final Mode mode = RobotBase.isReal();

	public static final double mod(final double lhs, final double rhs) { return (lhs % rhs + rhs) % rhs; }

	public static final double angleNorm(final double angle) { return Constants.mod(angle + 180, 360) - 180; }

	public static final double angleDistance(final double a, final double b) {
		return Math
			.abs(Constants.angleNorm(180 - Math.abs(Math.abs(Constants.angleNorm(a) - Constants.angleNorm(b)) - 180)));
	}

	public static record PIDValues(double p, double i, double d, double f) {
		public final PIDController createController() { return new PIDController(this.p, this.i, this.d); }
	}
	

	public static record Ratio(double factor) {
		public Ratio(final double from, final double to) { this(to / from); }

		public final double forward(final double value) { return value * this.factor; }

		public final double inverse(final double value) { return value / this.factor; }

		public final Ratio inverse() { return new Ratio(1 / this.factor); }
	}

	public static final class CAN {
		private CAN() { throw new IllegalCallerException("Cannot instantiate `Constants.CAN`"); }

		public static final int pdh = 0;

		public static final int pigeon = 0;

		public static final int swerveFrontLeftAzimuth = 16;
		public static final int swerveFrontLeftDrive = 15;
		public static final int swerveFrontLeftEncoder = 16;
		public static final double swerveFrontLeftOffset = -0.385009765625;

		public static final int swerveFrontRightAzimuth = 3;
		public static final int swerveFrontRightDrive = 4;
		public static final int swerveFrontRightEncoder = 3;
		public static final double swerveFrontRightOffset = -0.38671875;

		public static final int swerveBackLeftAzimuth = 17;
		public static final int swerveBackLeftDrive = 18;
		public static final int swerveBackLeftEncoder = 17;
		public static final double swerveBackLeftOffset = -0.19384765625;

		public static final int swerveBackRightAzimuth = 1;
		public static final int swerveBackRightDrive = 2;
		public static final int swerveBackRightEncoder = 1;
		public static final double swerveBackRightOffset = -0.404296875;
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
			public static final boolean thetaCompensation = true;
		}

		/* TORQUE-based velocity does not require a feed forward, as torque will accelerate the 
			rotor up to the desired velocity by itself */
		// kP = 5 An error of 1 rotation per second results in 5 amps output
		// kI = 0.1 An error of 1 rotation per second increases output by 0.1 amps every second
		// kD = 0.001 A change of 1000 rotation per second squared results in 1 amp output
		public static final Slot1Configs driveGainsSlot1 = new Slot1Configs()
			.withKP(5).withKI(0.1).withKD(0.001)
			.withKS(0).withKV(0).withKA(0);
	

		/* VOLTAGE-based velocity requires a feed forward to account for the back-emf of the motor */
		// kP = 0.11 An error of 1 rotation per second results in 2V output
		// kI = 0.5 An error of 1 rotation per second increases output by 0.5V every second
		// kD = 0.0001 A change of 1 rotation per second squared results in 0.01 volts output
		// 0.12 Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 
		public static final Slot0Configs driveGainsSlot0 = new Slot0Configs()
			.withKP(0.11).withKI(0.5).withKD(0.0001)
			.withKS(0).withKV(0.12).withKA(0);
			// .withKP(3).withKI(0).withKD(0)
			// .withKS(0).withKV(0).withKA(0);

		// POSITION Closed-loop control
		// An error of 0.5 rotations results in 1.2 volts output
		// A change of 1 rotation per second results in 0.1 volts output
		public static final Slot0Configs turnGainsSlot0 = new Slot0Configs()
			.withKP(2.4).withKI(0).withKD(0.1)
			.withKS(0).withKV(0).withKA(0);

		// The steer motor uses any SwerveModule.SteerRequestType control request with the
		// output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
			// .withKP(100).withKI(0).withKD(0.05)
			// .withKS(0).withKV(1.5).withKA(0);

		// todo: tune
		public static final PIDValues swerveAzimuthPID = new PIDValues(0.1, 0.01, 0.003, 0);
		public static final PIDValues absoluteRotationPID = new PIDValues(1.8, 0.01, 0, 0);
		// todo: find
		public static final SimpleMotorFeedforward driveFFW = new SimpleMotorFeedforward(0, 4, 0);
		public final static PIDController drivePID = new PIDController(0.1, 0, 0);

		public static final double thetaCompensationFactor = 0.1;

		public static final double wheelPositionRadius = 0.3906711; // radius of the circle that wheels are positioned on
		public static final double wheelBase = (wheelPositionRadius * Math.sqrt(2)); // 0.55154329
		public static final double trackWidth = wheelBase; // For a square drivetrain

		public static final Translation2d swerveFrontLeftTranslation = new Translation2d(
			Constants.Drivetrain.wheelPositionRadius,
			Rotation2d.fromDegrees(-45)
		);
		public static final Translation2d swerveFrontRightTranslation = new Translation2d(
			Constants.Drivetrain.wheelPositionRadius,
			Rotation2d.fromDegrees(45)
		);
		public static final Translation2d swerveBackLeftTranslation = new Translation2d(
			Constants.Drivetrain.wheelPositionRadius,
			Rotation2d.fromDegrees(180 + 45)
		);
		public static final Translation2d swerveBackRightTranslation = new Translation2d(
			Constants.Drivetrain.wheelPositionRadius,
			Rotation2d.fromDegrees(180 - 45)
		);

		public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			Constants.Drivetrain.swerveFrontLeftTranslation,
			Constants.Drivetrain.swerveFrontRightTranslation,
			Constants.Drivetrain.swerveBackLeftTranslation,
			Constants.Drivetrain.swerveBackRightTranslation
		);

		public static final Ratio motorEncoderToRotations = new Ratio(2048, 1);
		public static final Ratio driveGearMotorToWheel = new Ratio(6.75, 1); // 6.75:1 (motor:wheel)
		public static final Ratio azimuthGearMotorToWheel = new Ratio(150.0 / 7.0, 1); // (150 / 7):1 (motor:wheel)
		public static final double driveGearRatio = 6.75;
		public static final double azimuthGearRatio = 150.0 / 7.0;

		public static final double wheelRadius = 2.0 * 0.0254; // m
		public static final double wheelCircumference = (wheelRadius * 2) * Math.PI;
		public static final double rotationsPerMeter = driveGearRatio / wheelCircumference;
	
		public static final double maxVelocityMetersPerSec = 5.0; // m/s

		// max angular velocity computes to 6.41 radians per second
		public static final double maxAngularVelocityRadPerSec = maxVelocityMetersPerSec / Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

		// // todo: choose
		// public static final double axialLateralSpeed = 1; // m/s
		// public static final double thetaSpeed = 180.0; // deg/s
	}
}
