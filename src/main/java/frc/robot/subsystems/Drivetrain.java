package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    public static enum SwerveModulePlace {
        FrontLeft(0),
        FrontRight(1),
        BackLeft(2),
        BackRight(3);

        private SwerveModulePlace(final int index) { this.index = index; }

        public final int index;
    }

    public static class SwerveModule {
        public final SwerveModulePlace place;

        public final WPI_TalonFX azimuth;
        public final WPI_TalonFX drive;
        public final WPI_CANCoder encoder;

        private final PIDController pid = Constants.Drivetrain.swerveAzimuthPID.createController();

        private boolean backwards = false;

        public Rotation2d target = new Rotation2d();
        public double targetVelocity = 0;

        public SwerveModule(final SwerveModulePlace place, final WPI_TalonFX azimuth, final WPI_TalonFX drive, final WPI_CANCoder encoder) {
            this.place = place;
            this.azimuth = azimuth;
            this.drive = drive;
            this.encoder = encoder;

            azimuth.setNeutralMode(NeutralMode.Brake);

            drive.setNeutralMode(NeutralMode.Brake);

            encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
            encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

            this.pid.enableContinuousInput(-180, 180);
        }

        public SwerveModulePosition pos() { return new SwerveModulePosition(Constants.Drivetrain.driveGearMotorToWheel.forward(Constants.Drivetrain.motorEncoderToRotations.forward(this.drive.getSelectedSensorPosition())), Rotation2d.fromDegrees(this.encoder.getAbsolutePosition())); }

        public void applyState(final SwerveModuleState state) {
            final double ffw = Constants.Drivetrain.driveFFW.calculate(-state.speedMetersPerSecond);

            this.target = state.angle.unaryMinus();
            this.targetVelocity = ffw;
        }

        public void halt() {
            this.drive.stopMotor();
        }

        void update() {
            SmartDashboard.putNumber(this.place.name() + " Angle", this.pos().angle.getDegrees());

            this.backwards = Constants.Drivetrain.Flags.wheelOptimization & Constants.angleDistance(this.target.getDegrees(), this.pos().angle.getDegrees()) > 90;
            final double target = this.backwards ? Constants.angleNorm(this.target.getDegrees() + 180) : this.target.getDegrees();

            SmartDashboard.putNumber(this.place.name() + " Target", target);
            SmartDashboard.putNumber(this.place.name() + " Distance From Target", Constants.angleDistance(target, this.pos().angle.getDegrees()));

            final double turn = this.pid.calculate(this.pos().angle.getDegrees(), target);

            this.azimuth.set(ControlMode.PercentOutput, Constants.Drivetrain.azimuthGearMotorToWheel.forward(-turn));
            this.drive.set(ControlMode.PercentOutput, this.backwards ? -this.targetVelocity : this.targetVelocity);
        }
    }

    public static class SwerveState {
        public final SwerveModuleState[] states;

        public SwerveState() { this.states = new SwerveModuleState[4]; }
        public SwerveState(final SwerveModuleState[] states) { this.states = states; }

        public static SwerveState forward() {
            return new SwerveState()
                .set(SwerveModulePlace.FrontLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
                .set(SwerveModulePlace.FrontRight, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
                .set(SwerveModulePlace.BackLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
                .set(SwerveModulePlace.BackRight, new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        }

        public static SwerveState locked() {
            return new SwerveState()
                .set(SwerveModulePlace.FrontLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
                .set(SwerveModulePlace.FrontRight, new SwerveModuleState(0, Rotation2d.fromDegrees(45)))
                .set(SwerveModulePlace.BackLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(-135)))
                .set(SwerveModulePlace.BackRight, new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        }

        public SwerveModuleState get(final SwerveModulePlace place) { return this.states[place.index]; }
        public SwerveState set(final SwerveModulePlace place, final SwerveModuleState state) {
            this.states[place.index] = state;
            return this;
        }
    }

    public final WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.CAN.pigeon);

    public final SwerveModule swerveFrontLeft = new SwerveModule(
        SwerveModulePlace.FrontLeft,
        new WPI_TalonFX(Constants.CAN.swerveFrontLeftAzimuth),
        new WPI_TalonFX(Constants.CAN.swerveFrontLeftDrive),
        new WPI_CANCoder(Constants.CAN.swerveFrontLeftEncoder)
    );
    public final SwerveModule swerveFrontRight = new SwerveModule(
        SwerveModulePlace.FrontRight,
        new WPI_TalonFX(Constants.CAN.swerveFrontRightAzimuth),
        new WPI_TalonFX(Constants.CAN.swerveFrontRightDrive),
        new WPI_CANCoder(Constants.CAN.swerveFrontRightEncoder)
    );
    public final SwerveModule swerveBackLeft = new SwerveModule(
        SwerveModulePlace.BackLeft,
        new WPI_TalonFX(Constants.CAN.swerveBackLeftAzimuth),
        new WPI_TalonFX(Constants.CAN.swerveBackLeftDrive),
        new WPI_CANCoder(Constants.CAN.swerveBackLeftEncoder)
    );
    public final SwerveModule swerveBackRight = new SwerveModule(
        SwerveModulePlace.BackRight,
        new WPI_TalonFX(Constants.CAN.swerveBackRightAzimuth),
        new WPI_TalonFX(Constants.CAN.swerveBackRightDrive),
        new WPI_CANCoder(Constants.CAN.swerveBackRightEncoder)
    );
    public final SwerveModule[] modules = {
        this.swerveFrontLeft,
        this.swerveFrontRight,
        this.swerveBackLeft,
        this.swerveBackRight,
    };

    public final SwerveDriveKinematics kinematics = Constants.Drivetrain.kinematics;
    public final SwerveDrivePoseEstimator est = new SwerveDrivePoseEstimator(this.kinematics, this.gyro.getRotation2d(), this.getModulePositions(), new Pose2d());

    public Drivetrain() {
        this.swerveFrontRight.drive.setInverted(true);
        this.swerveBackRight.drive.setInverted(true);
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(this.modules).map(SwerveModule::pos).toArray(SwerveModulePosition[]::new);
    }

    public void swerve(final SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.maxWheelSpeed);
        for(int i = 0; i < this.modules.length; i++) this.modules[i].applyState(states[i]);
    }

    public void swerve(final SwerveState state) {
        this.swerve(state.states);
    }

    public ChassisSpeeds fod(final ChassisSpeeds field) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(field, this.gyro.getRotation2d().unaryMinus());
    }

    public ChassisSpeeds compensate(final ChassisSpeeds original) {
        // using this function because its just an easy way to rotate the axial/lateral speeds
        return ChassisSpeeds.fromFieldRelativeSpeeds(original, Rotation2d.fromRadians(original.omegaRadiansPerSecond * Constants.Drivetrain.thetaCompensationFactor));
    }

    @Override
    public void periodic() {
        for(final SwerveModule swerve : this.modules) swerve.update();

        this.est.update(this.gyro.getRotation2d(), this.getModulePositions());
    }
}
