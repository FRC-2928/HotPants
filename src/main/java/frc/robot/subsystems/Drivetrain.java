package frc.robot.subsystems;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.subsystems.SwerveModule.Place;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class Drivetrain extends SubsystemBase {
	public static class State {
		public final SwerveModuleState[] states;

		public State() { this.states = new SwerveModuleState[4]; }

		public State(final SwerveModuleState[] states) { this.states = states; }

		public static State forward() {
			return new State()
				.set(Place.FrontLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
				.set(Place.FrontRight, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
				.set(Place.BackLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
				.set(Place.BackRight, new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
		}

		public static State locked() {
			return new State()
				.set(Place.FrontLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
				.set(Place.FrontRight, new SwerveModuleState(0, Rotation2d.fromDegrees(45)))
				.set(Place.BackLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(-135)))
				.set(Place.BackRight, new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
		}

		public SwerveModuleState get(final Place place) { return this.states[place.index]; }

		public State set(final Place place, final SwerveModuleState state) {
			this.states[place.index] = state;
			return this;
		}
	}

	public final GyroIO gyro;
	public final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

	public final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR

	public final SwerveDriveKinematics kinematics = Constants.Drivetrain.kinematics;
	public final SwerveDrivePoseEstimator est;
	public final Limelight limelightNote = new Limelight("limelight-note");
	public final Limelight limelightShooter = new Limelight("limelight-shooter");
	public final Limelight limelightRear = new Limelight("limelight-rear");
	public final Limelight limelight = new Limelight("limelight");

	public final JoystickDrive joystickDrive = new JoystickDrive(this);
	private Rotation2d joystickFOROffset;
	public ChassisSpeeds joystickSpeeds = new ChassisSpeeds();
	private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();
	public AutoFactory autoFactory;

	// PathPlanner config constants
	private static final double ROBOT_MASS_KG = /*74.088*/ 57;
	private static final double ROBOT_MOI = 6.883;
	private static final double WHEEL_COF = 1.2;
	private static final RobotConfig PP_CONFIG = /* TODO: TUNE THESE FOR OUR ROBOT */
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              Constants.Drivetrain.wheelRadius,
              Constants.Drivetrain.maxVelocity,
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(Constants.Drivetrain.driveGearRatio),
              (Current)Units.Amps.of(120),
              1),
          Constants.Drivetrain.kinematics.getModules());

	public Drivetrain() {
		this.gyro = switch(Constants.mode) {
		case REAL -> new GyroIOReal();
		case REPLAY -> new GyroIO() {
		};
		default -> throw new Error();
		};

		this.modules[0] = new SwerveModule(Place.FrontLeft);
		this.modules[1] = new SwerveModule(Place.FrontRight);
		this.modules[2] = new SwerveModule(Place.BackLeft);
		this.modules[3] = new SwerveModule(Place.BackRight);

		this.est = new SwerveDrivePoseEstimator(
			this.kinematics,
			new Rotation2d(this.gyroInputs.yawPosition),
			this.modulePositions(),
			new Pose2d()
		);

		this.joystickFOROffset = new Rotation2d(this.gyroInputs.yawPosition);

		// PathPlannerLib auto configuration. Refer https://pathplanner.dev/pplib-getting-started.html
		AutoBuilder
			.configure(
				this::getEstimatedPosition,
				null,
				this::getCurrentChassisSpeeds,
				this::controlRobotOriented,
				new PPHolonomicDriveController(
					Constants.fromPIDValues(Constants.Drivetrain.Auto.translationDynamic),
					Constants.fromPIDValues(Constants.Drivetrain.Auto.thetaDynamic)),
				PP_CONFIG,
				() -> false,
				this
			);

		PathPlannerLogging
			.setLogActivePathCallback(
				poses -> Logger.recordOutput("Drivetrain/Auto/PathPoses", poses.toArray(Pose2d[]::new))
			);
		PathPlannerLogging.setLogCurrentPoseCallback(pose -> Logger.recordOutput("Drivetrain/Auto/CurrentPose", pose));
		PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("Drivetrain/Auto/DesiredPose", pose));

		autoFactory = new AutoFactory(
            this.est::getEstimatedPosition, // A function that returns the current robot pose
            this::reset, // A function that resets the current robot pose to the provided Pose2d
            this::controlSwerveSample, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            this // The drive subsystem
        );
	}

	public void control(ChassisSpeeds speeds) {
		Logger.recordOutput("Drivetrain/dx", speeds.vxMetersPerSecond);
		Logger.recordOutput("Drivetrain/dy", speeds.vyMetersPerSecond);
		Logger.recordOutput("Drivetrain/dtheta", speeds.omegaRadiansPerSecond);

		speeds = this.fod(speeds);
		speeds = this.compensate(speeds);
		speeds = ChassisSpeeds.discretize(speeds, 0.02);

		this.robotChassisSpeeds = speeds;

		this.control(this.kinematics.toSwerveModuleStates(speeds));
	}

	public void controlSwerveSample(final SwerveSample sample) {
		controlRobotOriented(sample.getChassisSpeeds());
	}

	public void controlRobotOriented(final ChassisSpeeds speeds) {
		speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond * 0.45;

		Logger.recordOutput("Drivetrain/dx", speeds.vxMetersPerSecond);
		Logger.recordOutput("Drivetrain/dy", speeds.vyMetersPerSecond);
		Logger.recordOutput("Drivetrain/dtheta", speeds.omegaRadiansPerSecond);

		this.robotChassisSpeeds = ChassisSpeeds.discretize(speeds.unaryMinus(), 0.02);

		this.control(this.kinematics.toSwerveModuleStates(this.robotChassisSpeeds));
	}

	public void control(final Drivetrain.State state) { this.control(state.states); }

	public void control(final SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.maxVelocity);

		for(int i = 0; i < this.modules.length; i++) {
			if (this.modules[i] != null) {
				this.modules[i].control(states[i]);
			}
		}
	}

	public void halt() { this.control(State.locked()); }

	public ChassisSpeeds fod(final ChassisSpeeds speeds) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, new Rotation2d(getFieldOrientedAngle()));
	}

	public ChassisSpeeds rod(final ChassisSpeeds speeds) {
		return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, new Rotation2d(getFieldOrientedAngle()));
	}

	public ChassisSpeeds compensate(final ChassisSpeeds original) {
		// using this function because its just an easy way to rotate the axial/lateral speeds
		return ChassisSpeeds
			.fromFieldRelativeSpeeds(
				original,
				Rotation2d.fromRadians(original.omegaRadiansPerSecond * -Constants.Drivetrain.thetaCompensationFactor)
			);
	}

	public void resetAngle() {
		// this.reset(new Pose2d(this.est.getEstimatedPosition().getTranslation(), Rotation2d.fromRadians(0)));
		this.joystickFOROffset = new Rotation2d(this.gyroInputs.yawPosition);
		// ((JoystickDrive) this.getDefaultCommand()).forTarget = Units.Radians.zero();
	}

	public Angle getFieldOrientedAngle() {
		return this.gyroInputs.yawPosition.minus(this.joystickFOROffset.getMeasure());
	}

	public void reset(final Pose2d newPose) {
		this.est.resetPosition(new Rotation2d(this.gyroInputs.yawPosition), this.modulePositions(), newPose);
	}
	public void setAngle(Angle angle){
		this.reset(new Pose2d(this.est.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(angle.in(Units.Degrees))));
	}

	@AutoLogOutput
	public void runCharacterization(final double volts) {
		for(int i = 0; i < this.modules.length; i++) {
			this.modules[i].runCharacterization(volts);
		}
		Logger.recordOutput("Drivetrain/InputVoltage", volts);
	}

	@AutoLogOutput(key = "Drivetrain/CurrentPositions")
	public SwerveModulePosition[] modulePositions() {
		return Arrays.stream(this.modules).map(module -> (module != null) ? module.position : new SwerveModulePosition()).toArray(SwerveModulePosition[]::new);
	}

	@AutoLogOutput(key = "Drivetrain/States/Desired")
	public SwerveModuleState[] desiredModuleStates() {
		return Arrays.stream(this.modules).map(module -> (module != null) ? module.desired : new SwerveModuleState()).toArray(SwerveModuleState[]::new);
	}

	@AutoLogOutput(key = "Drivetrain/States/Current")
	public SwerveModuleState[] currentModuleStates() {
		return Arrays.stream(this.modules).map(module -> (module != null) ? module.current : new SwerveModuleState()).toArray(SwerveModuleState[]::new);
	}

	@AutoLogOutput(key = "Drivetrain/CurrentChassisSpeeds")
	public ChassisSpeeds getCurrentChassisSpeeds() {
		return this.kinematics.toChassisSpeeds(this.currentModuleStates());
	}

	// Returns the current odometry pose, transformed to blue origin coordinates.
	@AutoLogOutput(key = "Odometry/BlueOriginPose")
	public Pose2d blueOriginPose() {
		// if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
		// 	return this.est
		// 		.getEstimatedPosition()
		// 		.relativeTo(new Pose2d(Constants.FIELD_LAYOUT.getFieldLength(), Constants.FIELD_LAYOUT.getFieldWidth(), Rotation2d.fromRadians(Math.PI)));
		// } else {
			return this.est.getEstimatedPosition();
		// }
	}

	public Pose2d getEstimatedPosition(){
		return this.est.getEstimatedPosition();
	}

	@Override
	public void periodic() {
		this.gyro.updateInputs(this.gyroInputs);
		Logger.processInputs("Drivetrain/Gyro", this.gyroInputs);
        Logger.recordOutput("Drivetrain/Botpose",limelightNote.getBluePose3d());
		this.joystickSpeeds = this.joystickDrive.speeds();
		if(this.getCurrentCommand() == this.joystickDrive) this.control(this.joystickSpeeds);

		for(final SwerveModule module : this.modules) {
			if (module != null) {
				module.periodic();
			}
		}

		// Update the odometry pose
		this.est.update(new Rotation2d(this.gyroInputs.yawPosition), this.modulePositions());

		// Add vision measurements to pos est with megatag 2
		PoseEstimate mt2 = this.limelight.getPoseMegatag2();
		boolean doRejectUpdate = false;

		// if our angular velocity is greater than 720 degrees per second, ignore vision updates
		if(Math.abs(this.gyroInputs.yawVelocityRadPerSec.in(Units.DegreesPerSecond)) > 720) {
			doRejectUpdate = true;
		}

		if(mt2.tagCount == 0) {
			doRejectUpdate = true;
		}

		if(!doRejectUpdate) {
			est.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
			est.addVisionMeasurement(
				mt2.pose,
				mt2.timestampSeconds);
		}

		Logger.recordOutput("Drivetrain/LimelightNotePose", this.limelightNote.getPose2d());
		Logger.recordOutput("Drivetrain/Pose", this.est.getEstimatedPosition());
		Logger.recordOutput("Drivetrain/poseMegatag", this.limelight.getPoseMegatag2().pose);
		Logger.recordOutput("Drivetrain/Imumode", limelight.getImuMode());
		Logger.recordOutput("Drivetrain/Mt1", this.limelight.getPoseMegatag1().pose);
	}

	public void disabledPeriodic() {
		if(this.limelight.hasValidTargets()){
			this.setAngle(this.limelight.getPoseMegatag1().pose.getRotation().getMeasure());
		}
		this.limelight.setRobotOrientation(this.est.getEstimatedPosition().getRotation().getDegrees()); 
	}
}
