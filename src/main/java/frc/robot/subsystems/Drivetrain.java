package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils;

public class Drivetrain extends SubsystemBase {
    public static enum SwerveModulePlace {
        FrontLeft,
        FrontRight,
        BackLeft,
        BackRight
    }

    public class SwerveModule {
        public final SwerveModulePlace place;
        public final WPI_TalonFX azimuth;
        public final WPI_TalonFX drive;

        /// The absolute encoder for angle
        public final WPI_CANCoder encoder;

        public Rotation2d targetAngle = new Rotation2d();

        private final PIDController angleController = new PIDController(
            Constants.Drivetrain.swerveAnglePID.p,
            Constants.Drivetrain.swerveAnglePID.i,
            Constants.Drivetrain.swerveAnglePID.d
        );

        public SwerveModule(SwerveModulePlace place, WPI_TalonFX azimuth, WPI_TalonFX drive, WPI_CANCoder encoder) {
            this.place = place;
            this.azimuth = azimuth;
            this.drive = drive;
            this.encoder = encoder;



            this.angleController.enableContinuousInput(-180, 180);
        }

        public Rotation2d angle() { return Rotation2d.fromDegrees(this.encoder.getAbsolutePosition()); }

        public void applyState(SwerveModuleState state) {
            this.targetAngle = state.angle;

            double ffw = Constants.Drivetrain.driveFFW.calculate(state.speedMetersPerSecond);

            this.drive.set(ControlMode.Velocity, ffw);
        }

        public void halt() {
            this.drive.stopMotor();
        }

        void updateController() {
            double turn = this.angleController.calculate(this.angle().getDegrees(), this.targetAngle.getDegrees());

            this.azimuth.set(ControlMode.PercentOutput, turn);
        }
    }

    public final SwerveDriveKinematics kinematics = Constants.Drivetrain.kinematics;

    public final WPI_Pigeon2 gyro;

    public final SwerveModule swerveFrontLeft;
    public final SwerveModule swerveFrontRight;
    public final SwerveModule swerveBackLeft;
    public final SwerveModule swerveBackRight;
    public final SwerveModule[] modules;

    public Drivetrain() {
        Utils.ensureSingleton(this);

        this.gyro = new WPI_Pigeon2(Constants.CAN.pigeon);
        this.swerveFrontLeft = new SwerveModule(
            SwerveModulePlace.FrontLeft,
            new WPI_TalonFX(Constants.CAN.swerveFrontLeftAzimuth),
            new WPI_TalonFX(Constants.CAN.swerveFrontLeftDrive),
            new WPI_CANCoder(Constants.CAN.swerveFrontLeftEncoder)
        );
        this.swerveFrontRight = new SwerveModule(
            SwerveModulePlace.FrontRight,
            new WPI_TalonFX(Constants.CAN.swerveFrontRightAzimuth),
            new WPI_TalonFX(Constants.CAN.swerveFrontRightDrive),
            new WPI_CANCoder(Constants.CAN.swerveFrontRightEncoder)
        );
        this.swerveBackLeft = new SwerveModule(
            SwerveModulePlace.BackLeft,
            new WPI_TalonFX(Constants.CAN.swerveBackLeftAzimuth),
            new WPI_TalonFX(Constants.CAN.swerveBackLeftDrive),
            new WPI_CANCoder(Constants.CAN.swerveBackLeftEncoder)
        );
        this.swerveBackRight = new SwerveModule(
            SwerveModulePlace.BackRight,
            new WPI_TalonFX(Constants.CAN.swerveBackRightAzimuth),
            new WPI_TalonFX(Constants.CAN.swerveBackRightDrive),
            new WPI_CANCoder(Constants.CAN.swerveBackRightEncoder)
        );
        this.modules = new SwerveModule[] { this.swerveFrontLeft, this.swerveFrontRight, this.swerveBackLeft, this.swerveBackRight };
    }

    public void swerve(SwerveModuleState[] moduleStates) {
        for(int i = 0; i < this.modules.length; i++)
            this.modules[i].applyState(moduleStates[i]);
    }

    public void periodic() {
        for(SwerveModule swerve : this.modules)
            swerve.updateController();
    }
}
