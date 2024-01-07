package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import com.ctre.phoenix6.motorcontrol.ControlMode;
//import com.ctre.phoenix6.motorcontrol.NeutralMode;
//import com.ctre.phoenix6.motorcontrol.can.TalonFX;
//import com.ctre.phoenix6.sensors.AbsoluteSensorRange;
//import com.ctre.phoenix6.sensors.SensorInitializationStrategy;
//import com.ctre.phoenix6.sensors.CANCoder;
//import com.ctre.phoenix6.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
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

        public final TalonFX azimuth;
        public final TalonFX drive;
        public final CANcoder encoder;

        private final PIDController pid = Constants.Drivetrain.swerveAzimuthPID.createController();

        private boolean backwards = false;

        public Rotation2d target = new Rotation2d();
        public double targetVelocity = 0;

        public SwerveModule(final SwerveModulePlace place, final TalonFX azimuth, final TalonFX drive, final CANcoder encoder) {
        	this.place = place;
            this.azimuth = azimuth;
            this.drive = drive;
            this.encoder = encoder;

            azimuth.setNeutralMode(NeutralModeValue.Brake);

            drive.setNeutralMode(NeutralModeValue.Brake);
			//AbsoluteSensorRange.Signed_PlusMinus180
			//SensorInitializationStrategy.BootToAbsolutePosition
            // encoder.configAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
            // encoder.configSensorInitializationStrategy("uhjg");

            this.pid.enableContinuousInput(-180, 180);
        }

        public SwerveModulePosition pos() { return new SwerveModulePosition(Constants.Drivetrain.driveGearMotorToWheel.forward(Constants.Drivetrain.motorEncoderToRotations.forward(this.drive.getRotorPosition().getValue())), Rotation2d.fromDegrees(this.encoder.getAbsolutePosition().getValue())); }

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

            this.azimuth.set(Constants.Drivetrain.azimuthGearMotorToWheel.forward(MathUtil.clamp(-turn, -90, 90)));
            this.drive.set(this.backwards ? -this.targetVelocity : this.targetVelocity);
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

    public final Pigeon2 gyro = new Pigeon2(Constants.CAN.pigeon);

    public final SwerveModule swerveFrontLeft = new SwerveModule(
        SwerveModulePlace.FrontLeft,
        new TalonFX(Constants.CAN.swerveFrontLeftAzimuth),
        new TalonFX(Constants.CAN.swerveFrontLeftDrive),
        new CANcoder(Constants.CAN.swerveFrontLeftEncoder)
    );
    public final SwerveModule swerveFrontRight = new SwerveModule(
        SwerveModulePlace.FrontRight,
        new TalonFX(Constants.CAN.swerveFrontRightAzimuth),
        new TalonFX(Constants.CAN.swerveFrontRightDrive),
        new CANcoder(Constants.CAN.swerveFrontRightEncoder)
    );
    public final SwerveModule swerveBackLeft = new SwerveModule(
        SwerveModulePlace.BackLeft,
        new TalonFX(Constants.CAN.swerveBackLeftAzimuth),
        new TalonFX(Constants.CAN.swerveBackLeftDrive),
        new CANcoder(Constants.CAN.swerveBackLeftEncoder)
    );
    public final SwerveModule swerveBackRight = new SwerveModule(
        SwerveModulePlace.BackRight,
        new TalonFX(Constants.CAN.swerveBackRightAzimuth),
        new TalonFX(Constants.CAN.swerveBackRightDrive),
        new CANcoder(Constants.CAN.swerveBackRightEncoder)
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
