package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public static final class PIDValues {
        public double p;
        public double i;
        public double d;
        public double f;

        public PIDValues(double p, double i, double d, double f) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
        }

        public PIDController createController() {
            return new PIDController(this.p, this.i, this.d);
        }
    }

    public static final class CAN {
        public static final int pigeon = 0;

        public static final int swerveFrontLeftAzimuth = 16;
        public static final int swerveFrontLeftDrive = 15;
        public static final int swerveFrontLeftEncoder = 16;

        public static final int swerveFrontRightAzimuth = 3;
        public static final int swerveFrontRightDrive = 4;
        public static final int swerveFrontRightEncoder = 3;

        public static final int swerveBackLeftAzimuth = 17;
        public static final int swerveBackLeftDrive = 18;
        public static final int swerveBackLeftEncoder = 17;

        public static final int swerveBackRightAzimuth = 1;
        public static final int swerveBackRightDrive = 2;
        public static final int swerveBackRightEncoder = 1;
    }

    public static final class Drivetrain {
        // todo: tune
        public static final PIDValues swerveAzimuthPID = new PIDValues(0.075, 0, 0, 0);

        public static final double wheelPositionRadius = 0.3906711; // radius of the circle that wheels are positioned on

        public static final Translation2d swerveFrontLeftTranslation = new Translation2d(Constants.Drivetrain.wheelPositionRadius, Rotation2d.fromDegrees(-45));
        public static final Translation2d swerveFrontRightTranslation = new Translation2d(Constants.Drivetrain.wheelPositionRadius, Rotation2d.fromDegrees(45));
        public static final Translation2d swerveBackLeftTranslation = new Translation2d(Constants.Drivetrain.wheelPositionRadius, Rotation2d.fromDegrees(180 + 45));
        public static final Translation2d swerveBackRightTranslation = new Translation2d(Constants.Drivetrain.wheelPositionRadius, Rotation2d.fromDegrees(180 - 45));

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            Constants.Drivetrain.swerveFrontLeftTranslation,
            Constants.Drivetrain.swerveFrontRightTranslation,
            Constants.Drivetrain.swerveBackLeftTranslation,
            Constants.Drivetrain.swerveBackRightTranslation
        );

        // todo: fill
        public static final SimpleMotorFeedforward driveFFW = new SimpleMotorFeedforward(0, 1, 0);
        public static final double driveGearRatio = 1.0 / 6.75; // 6.75:1 (motor:wheel)
        public static final double azimuthGearRatio = 1.0 / (150.0 / 7.0); // (150 / 7):1 (motor:wheel)
        public static final double wheelRadius = 2.0 * 0.0254; // m

        public static final double axialSpeed = 1.0;
        public static final double lateralSpeed = 1.0;
        public static final double thetaSpeed = 90.0;
    }
}
