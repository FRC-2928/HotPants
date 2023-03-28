package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public static final class PIDValues {
        public double p;
        public double i;
        public double d;

        public PIDValues(double p, double i, double d) {
            this.p = p;
            this.i = i;
            this.d = d;
        }
    }
    
    public static final class CAN {
        public static final int pigeon = 0;

        public static final int swerveFrontLeftMotorU = 0;
        public static final int swerveFrontLeftMotorV = 0;
        public static final int swerveFrontLeftEncoder = 0;
        
        public static final int swerveFrontRightMotorU = 0;
        public static final int swerveFrontRightMotorV = 0;
        public static final int swerveFrontRightEncoder = 0;
        
        public static final int swerveBackLeftMotorU = 0;
        public static final int swerveBackLeftMotorV = 0;
        public static final int swerveBackLeftEncoder = 0;
        
        public static final int swerveBackRightMotorU = 0;
        public static final int swerveBackRightMotorV = 0;
        public static final int swerveBackRightEncoder = 0;
    }

    public static final class Drivetrain {
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            Constants.Drivetrain.swerveFrontLeftTranslation,
            Constants.Drivetrain.swerveFrontRightTranslation,
            Constants.Drivetrain.swerveBackLeftTranslation,
            Constants.Drivetrain.swerveBackRightTranslation
        );

        public static final PIDValues swerveAnglePID = new PIDValues(0.25, 0, 0);

        public static final Translation2d swerveFrontLeftTranslation = new Translation2d(0, 0);
        public static final Translation2d swerveFrontRightTranslation = new Translation2d(0, 0);
        public static final Translation2d swerveBackLeftTranslation = new Translation2d(0, 0);
        public static final Translation2d swerveBackRightTranslation = new Translation2d(0, 0);

        public static final double axialSpeed = 1.0;
        public static final double lateralSpeed = 1.0;
        public static final double thetaSpeed = 90.0;
    }
}
