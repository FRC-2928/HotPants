package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.Units;
public class Constants {
	private static Mode currentMode() {
		if(Robot.isReal()) return Mode.REAL;
		if(!Logger.hasReplaySource()) return Mode.SIM;
		else return Mode.REPLAY;
	}

	private Constants() { throw new IllegalCallerException("Cannot instantiate `Constants`"); }

	public static final Mode mode = Constants.currentMode();
	public static final boolean real = Constants.mode == Constants.Mode.REAL;
	public static final Distance fieldWidth = Units.Meters.of(16.541); // Correlates to Field oriented x coordinate
	public static final Distance fieldDepth = Units.Meters.of(8.211); // Correlates to Field oriented y coordinate

	public static final AudioConfigs talonFXAudio = new AudioConfigs()
		.withAllowMusicDurDisable(true)
		.withBeepOnBoot(true)
		.withBeepOnConfig(false);

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

		public final ProfiledPIDController createProfiledController(final TrapezoidProfile.Constraints profile) {
			return new ProfiledPIDController(this.p, this.i, this.d, profile);
		}
	}

	public static PIDConstants fromPIDValues(final PIDValues pid) { return new PIDConstants(pid.p, pid.d, pid.d); }

	public static final class LimelightFX {
		private LimelightFX() { throw new IllegalCallerException("Cannot instantiate `Constants.LimelightFX`"); }

		public static final class Colors {
			private Colors() { throw new IllegalCallerException("Cannot instantiate `Constants.LimelightFX.Colors`"); }
		}

		public static final boolean enabled = true;
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

			public static final int swerveFrontLeftAzimuth = 16;
			public static final int swerveFrontLeftDrive = 15;

			public static final int swerveFrontRightAzimuth = 3;
			public static final int swerveFrontRightDrive = 4;

			public static final int swerveBackLeftAzimuth = 17;
			public static final int swerveBackLeftDrive = 18;

			public static final int swerveBackRightAzimuth = 1; 
			public static final int swerveBackRightDrive = 2;

			public static final int shooterPivot = 12;
			public static final int shooterFlywheelA = 1;
			public static final int shooterFlywheelB = 4;

			public static final int climber = 17;
		}
	}

	public static final class PWM {
		private PWM() { throw new IllegalCallerException("Cannot instantiate `Constants.PWM`"); }

		public static final int climberRatchet = 9;

		public static final int ampBarServoA = 0;
		public static final int ampBarServoB = 1;
	}

	public static final class DIO {
		private DIO() { throw new IllegalCallerException("Cannot instantiate `Constants.DIO`"); }

		public static final int release = 0;
		public static final int lockout = 1;
	}

	public static final class Drivetrain {
		private Drivetrain() { throw new IllegalCallerException("Cannot instantiate `Constants.Drivetrain`"); }

		public static final class Auto {
			public static final PIDValues translationDynamic = new PIDValues(7.5, 0, 0.5, 0);
			public static final PIDValues thetaDynamic = new PIDValues(5, 0, 0.02, 0);
			public static final PIDValues centerLimelight = new PIDValues(1, 0, 0, 0);
			public static final PIDValues centerTheta = new PIDValues(4, 0, 0.1, 0);
		}

		public static final SlotConfigs azimuth = new SlotConfigs()
			.withKP(-60)
			.withKI(0)
			.withKD(0)
			.withKS(0)
			.withKV(0)
			.withKA(0);

		public static final SlotConfigs drive = new SlotConfigs()
			.withKP(4) /* 0.15 */
			.withKI(0.0)
			.withKD(0.15)
			.withKS(0.225)
			.withKV(2.62) 
			.withKA(0);

		// todo: tune
		public final static PIDValues drivePID = new PIDValues(0.1, 0, 0, 0);
		//public static final PIDValues swerveAzimuthPID = new PIDValues(0.01, 0, 0.005, 0);
		public static final PIDValues absoluteRotationPID = new PIDValues(2.3, 0, 0.15, 0);
		public static final TrapezoidProfile.Constraints absoluteRotationConstraints = new TrapezoidProfile.Constraints(
			1,
			17
		);
		public static final SimpleMotorFeedforward absoluteRotationFeedforward = new SimpleMotorFeedforward(2, 1);
		// todo: find
		public static final SimpleMotorFeedforward driveFFW = new SimpleMotorFeedforward(0, 2.5, 0);

		public static final double thetaCompensationFactor = 0.2;

		public static final Distance wheelBase = Units.Inches.of(29 - 2.5 * 2);
		public static final Distance trackWidth = Drivetrain.wheelBase; // For a square drivetrain`

		// public static final Angle swerveFrontLeftOffset = Units.Rotations.of(0.227783);
		// public static final Angle swerveFrontLeftOffset = Units.Rotations.of(0.349609375);
		public static final Angle swerveFrontLeftOffset = Units.Rotations.of(-0.385009765625);
		public static final Translation2d swerveFrontLeftTranslation = new Translation2d(
			Constants.Drivetrain.wheelBase,
			Constants.Drivetrain.trackWidth
		);
		// public static final Angle swerveFrontRightOffset = Units.Rotations.of(-0.150146484375);
		public static final Angle swerveFrontRightOffset = Units.Rotations.of(-0.38671875);
		public static final Translation2d swerveFrontRightTranslation = new Translation2d(
			Constants.Drivetrain.wheelBase,
			Constants.Drivetrain.trackWidth.negate()
		);
		// public static final Angle swerveBackLeftOffset = Units.Rotations.of(-0.136474609375);
		public static final Angle swerveBackLeftOffset = Units.Rotations.of(-0.19384765625);
		public static final Translation2d swerveBackLeftTranslation = new Translation2d(
			Constants.Drivetrain.wheelBase.negate(),
			Constants.Drivetrain.trackWidth
		);
		// public static final Angle swerveBackRightOffset = Units.Rotations.of(-0.4404296875);
		public static final Angle swerveBackRightOffset = Units.Rotations.of(-0.404296875);
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
		// public static final double driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // ~= 6.746
		public static final double driveGearRatio = (50.0 / 14) * (16.0 / 28) * (45.0 / 15); // ~= 6.746
		public static final double azimuthGearRatio = 150.0 / 7.0;

		public static final Distance wheelRadius = Units.Inches.of(2);
		public static final Distance wheelCircumference = Drivetrain.wheelRadius.times(2 * Math.PI);

		public static final LinearVelocity maxVelocity = Units.Meters.per(Units.Second).of(5.0);

		// max angular velocity computes to 6.41 radians per second
		public static final AngularVelocity maxAngularVelocity = Units.RotationsPerSecond
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

	public static class Shooter {
		public static final class FlywheelConfiguration {
			public static final FlywheelConfiguration yellowFairlane = new FlywheelConfiguration(
				Units.RotationsPerSecond.of(30),
				Units.RotationsPerSecond.of(27),
				0.3
			);
			public static final FlywheelConfiguration greenBane = new FlywheelConfiguration(
				Units.RotationsPerSecond.of(40),
				Units.RotationsPerSecond.of(37),
				0.60
			);

			public FlywheelConfiguration(
				final AngularVelocity flywheelVelocity,
				final AngularVelocity flywheelVelocityThreshold,
				final double ampPower
			) {
				this.speakerVelocity = flywheelVelocity;
				this.speakerVelocityThreshold = flywheelVelocityThreshold;
				this.ampPower = ampPower;
			}

			public final AngularVelocity speakerVelocity;
			public final AngularVelocity speakerVelocityThreshold;

			public final double ampPower;
		}

		private Shooter() { throw new IllegalCallerException("Cannot instantiate `Constants.Shooter`"); }

		public static final SlotConfigs pivotConfig = new SlotConfigs()
			.withGravityType(GravityTypeValue.Arm_Cosine)
			.withKS(0.025)
			.withKG(0.028)
			.withKP(10)
			.withKD(0.05);

		public static final double pivotCurrentLimit = 40;
		public static final AngularVelocity pivotMaxVelocityShoot = Units.DegreesPerSecond.of(2);
		public static final Slot0Configs flywheelGainsSlot0 = new Slot0Configs()
			.withKP(0.05)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0)
			.withKV(0.015)
			.withKA(0);

		public static final PIDValues targetRotationController = new PIDValues(0.3, 0, 0, 0);

		public static final FlywheelConfiguration flywheels = FlywheelConfiguration.greenBane;

		// todo: fill angles

		// a little above intake height to avoid hitting floor but to be ready
		public static final Angle readyIntake = Units.Rotations.of(-0.1085);
		// min angle before hitting floor
		public static final Angle intakeGround = Units.Rotations.of(-0.1085);

		public static final Angle readyDrive = Units.Degrees.zero();
		public static final Angle readyShootFront = Units.Rotations.of(0.122);
		public static final Angle readyShootRear = Units.Degrees.of(125);
		// public static final Angle readyShootRearSub = Units.Degrees.of(105);
		public static final Angle shootAmp = Units.Degrees.of(110);

		public static final double ampBarServoAExtend = 1.0;
		public static final double ampBarServoBExtend = 0.0;
		public static final double ampBarServoARetract = 0.0;
		public static final double ampBarServoBRetract = 1.0;

		public static final Angle startingConfiguration = Units.Degrees.of(90);

		public static final Angle shootSub = Units.Degrees.of(115);
		public static final Angle shootFerry = Units.Degrees.of(130);

		// max angle before exiting allowed extension range
		public static final Angle max = Units.Rotations.of(0.39);

		public static final double fireTimeout = 0.3;
	}

	public static class Climber {
		private Climber() { throw new IllegalCallerException("Cannot instantiate `Constants.Climber`"); }

		public static final boolean ratchetEnabled = false;

		public static final SlotConfigs configFast = new SlotConfigs().withKP(0.25);
		public static final SlotConfigs configSlow = new SlotConfigs().withKP(0.01);

		public static final double ratchetLocked = 84;
		public static final double ratchetFree = 93;

		public static final double max = 129;
		public static final double disengageDistance = 0.5;
		public static final double initializeRaiseDistance = 2;
	}
}
