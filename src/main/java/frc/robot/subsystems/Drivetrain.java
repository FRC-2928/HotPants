package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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

        private SwerveModulePlace(int index) { this.index = index; }

        public final int index;
    }

    public static class SwerveModule {
        public final SwerveModulePlace place;

        public final WPI_TalonFX azimuth;
        public final WPI_TalonFX drive;
        public final WPI_CANCoder encoder;

        private final PIDController pid = Constants.Drivetrain.swerveAzimuthPID.createController();

        public Rotation2d target = new Rotation2d();

        public SwerveModule(SwerveModulePlace place, WPI_TalonFX azimuth, WPI_TalonFX drive, WPI_CANCoder encoder) {
            this.place = place;
            this.azimuth = azimuth;
            this.drive = drive;
            this.encoder = encoder;

            azimuth.setNeutralMode(NeutralMode.Brake);

            drive.setNeutralMode(NeutralMode.Brake);

            this.pid.enableContinuousInput(-180, 180);
        }

        public Rotation2d angle() { return Rotation2d.fromDegrees(this.encoder.getAbsolutePosition()); }

        public void applyState(SwerveModuleState state) {
            double ffw = Constants.Drivetrain.driveFFW.calculate(state.speedMetersPerSecond);

            this.target = state.angle;
            this.drive.set(ControlMode.PercentOutput, ffw);
        }

        public void halt() {
            this.drive.stopMotor();
        }

        void update() {
            SmartDashboard.putNumber(this.place.name() + " Angle", this.angle().getDegrees());
            SmartDashboard.putNumber(this.place.name() + " Target", this.target.getDegrees());

            double turn = this.pid.calculate(this.angle().getDegrees(), this.target.getDegrees());

            // todo: correct angle
            // maybe invert encoder angles and remove the negation below?
            this.azimuth.set(ControlMode.PercentOutput, -turn * Constants.Drivetrain.azimuthGearRatio);
        }
    }

    public static class SwerveState {
        public final SwerveModuleState[] states;

        public SwerveState() { this.states = new SwerveModuleState[4]; }
        public SwerveState(SwerveModuleState[] states) { this.states = states; }

        public SwerveModuleState get(SwerveModulePlace place) { return this.states[place.index]; }
        public SwerveState set(SwerveModulePlace place, SwerveModuleState state) {
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
    public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(this.kinematics, this.gyro.getRotation2d(), null);

    public Drivetrain() {
        this.swerveFrontRight.drive.setInverted(true);
        this.swerveBackRight.drive.setInverted(true);
    }

    public void swerve(SwerveModuleState[] states) {
        for(int i = 0; i < this.modules.length; i++)
            this.modules[i].applyState(states[i]);
    }

    public void swerve(SwerveState state) {
        for(int i = 0; i < this.modules.length; i++)
            this.modules[i].applyState(state.states[i]);
    }

    public void periodic() {
        for(SwerveModule swerve : this.modules)
            swerve.update();
    }
}
