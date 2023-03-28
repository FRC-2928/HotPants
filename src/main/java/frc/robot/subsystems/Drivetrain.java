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

    public class SwerveModule extends MotorSafety {
        public final SwerveModulePlace place;
        public final WPI_TalonFX motorU;
        public final WPI_TalonFX motorV;

        /// The absolute encoder for angle
        public final WPI_CANCoder encoder;

        public double speed = 0;
        public Rotation2d targetAngle = new Rotation2d();

        private final PIDController angleController = new PIDController(
            Constants.Drivetrain.swerveAnglePID.p,
            Constants.Drivetrain.swerveAnglePID.i,
            Constants.Drivetrain.swerveAnglePID.d
        );

        public SwerveModule(SwerveModulePlace place, WPI_TalonFX motorU, WPI_TalonFX motorV, WPI_CANCoder encoder) {
            super();

            motorU.setSafetyEnabled(false);
            motorV.setSafetyEnabled(false);

            this.place = place;
            this.motorU = motorU;
            this.motorV = motorV;
            this.encoder = encoder;
        }

        public Rotation2d angle() { return Rotation2d.fromDegrees(this.encoder.getAbsolutePosition()); }

        public void applyState(SwerveModuleState state) {
            this.speed = state.speedMetersPerSecond;
            this.targetAngle = state.angle;

            this.setSafetyEnabled(true);
            this.feed();
        }

        public void halt() {
            this.stopMotor();

            this.setSafetyEnabled(false);
        }

        @Override
        public void stopMotor() {
            this.speed = 0;

            this.setSafetyEnabled(false);
        }

        @Override
        public String getDescription() { return String.format("SwerveModule(%s)", this.place.name()); }

        void updateController() {
            double turn = this.angleController.calculate(this.angle().getDegrees(), this.targetAngle.getDegrees());

            this.motorU.set(ControlMode.PercentOutput, this.speed + turn);
            this.motorV.set(ControlMode.PercentOutput, this.speed - turn);
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
            new WPI_TalonFX(Constants.CAN.swerveFrontLeftMotorU),
            new WPI_TalonFX(Constants.CAN.swerveFrontLeftMotorV),
            new WPI_CANCoder(Constants.CAN.swerveFrontLeftEncoder)
        );
        this.swerveFrontRight = new SwerveModule(
            SwerveModulePlace.FrontRight,
            new WPI_TalonFX(Constants.CAN.swerveFrontRightMotorU),
            new WPI_TalonFX(Constants.CAN.swerveFrontRightMotorV),
            new WPI_CANCoder(Constants.CAN.swerveFrontRightEncoder)
        );
        this.swerveBackLeft = new SwerveModule(
            SwerveModulePlace.BackLeft,
            new WPI_TalonFX(Constants.CAN.swerveBackLeftMotorU),
            new WPI_TalonFX(Constants.CAN.swerveBackLeftMotorV),
            new WPI_CANCoder(Constants.CAN.swerveBackLeftEncoder)
        );
        this.swerveBackRight = new SwerveModule(
            SwerveModulePlace.BackRight,
            new WPI_TalonFX(Constants.CAN.swerveBackRightMotorU),
            new WPI_TalonFX(Constants.CAN.swerveBackRightMotorV),
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
