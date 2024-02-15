package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroIOSim implements GyroIO {
    Drivetrain drivetrain;

    private Pose2d simOdometry = new Pose2d();
    double[] lastModulePositionMeters = {0, 0, 0, 0};
        
    private final Pigeon2 imu = new Pigeon2(0);
    private final Pigeon2SimState imuSim = imu.getSimState();

    public GyroIOSim(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        calcAngle();
        inputs.connected = true;
        // imuSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        // final StatusCode yaw = imuSim.setRawYaw(this.drivetrain.getRobotAngle().getRadians());
        // inputs.yawPosition = Rotation2d.fromRadians(yaw.value);
        inputs.yawPosition = simOdometry.getRotation();
    }

    private void calcAngle() {
        drivetrain.getSwerveModules();

        Rotation2d[] turnPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            turnPositions[i] = drivetrain.getSwerveModules()[i].getTurnPosition();
        }

        SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            double distanceMeters = drivetrain.getSwerveModules()[i].getModulePosition().distanceMeters;
            measuredStatesDiff[i] = new SwerveModuleState(distanceMeters - lastModulePositionMeters[i],
                                                        turnPositions[i]);
            lastModulePositionMeters[i] = distanceMeters;
            SmartDashboard.putNumber("SIM/turnPosition", turnPositions[0].getDegrees());
            SmartDashboard.putNumber("SIM/measuredSpeed", measuredStatesDiff[0].speedMetersPerSecond);
            SmartDashboard.putNumber("SIM/lastDistanceMeters", lastModulePositionMeters[0]);
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
