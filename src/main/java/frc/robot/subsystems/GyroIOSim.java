package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroIOSim implements GyroIO {
    Drivetrain drivetrain;

    private Pose2d simOdometry = new Pose2d();
    double[] lastModulePositionsRad = {0, 0, 0, 0};

    public GyroIOSim(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        calcAngle();
        inputs.yawPosition = simOdometry.getRotation();
        inputs.heading = simOdometry.getRotation();
        SmartDashboard.putNumber("Sim Yaw Position", inputs.yawPosition.getDegrees());
    }

    private void calcAngle() {
        drivetrain.getSwerveModules();

        Rotation2d[] turnPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            turnPositions[i] = drivetrain.getSwerveModules()[i].getTurnPosition();
        }

        SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStatesDiff[i] = new SwerveModuleState(
                    (drivetrain.getSwerveModules()[i].getPosition().distanceMeters - lastModulePositionsRad[i])
                            * Units.inchesToMeters(2),
                    turnPositions[i]);
            lastModulePositionsRad[i] = drivetrain.getSwerveModules()[i].getPosition().distanceMeters;
        }

        simOdometry = simOdometry.exp(new Twist2d(
                drivetrain.getKinematics().toChassisSpeeds(measuredStatesDiff).vxMetersPerSecond,
                drivetrain.getKinematics().toChassisSpeeds(measuredStatesDiff).vyMetersPerSecond,
                drivetrain.getKinematics().toChassisSpeeds(measuredStatesDiff).omegaRadiansPerSecond));
    }

    @Override
    public void resetGyro(){
        simOdometry = new Pose2d();
    }
}
