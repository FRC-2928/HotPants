package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import frc.robot.Robot;

public class GyroIOSim implements GyroIO {
	Drivetrain drivetrain;

	private Pose2d simOdometry = new Pose2d();
	double[] lastModulePositions = { 0, 0, 0, 0 };

	private final Pigeon2 imu = new Pigeon2(0);
	private final Pigeon2SimState imuSim = this.imu.getSimState();

	public GyroIOSim(final Drivetrain drivetrain) { this.drivetrain = drivetrain; }

	@Override
	public void updateInputs(final GyroIOInputs inputs) {
		this.calcAngle();
		inputs.connected = true;
		inputs.yawPosition = Units.Rotations.of(this.simOdometry.getRotation().getRotations());
	}

	private void calcAngle() {
		final Measure<Angle>[] azimuthPositions = new ImmutableMeasure[4];
		for(int i = 0; i < 4; i++) {
			azimuthPositions[i] = Robot.cont.drivetrain.modules[i].inputs.angle;
		}

		final SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
		for(int i = 0; i < 4; i++) {
			measuredStatesDiff[i] = new SwerveModuleState(
				Robot.cont.drivetrain.modules[i]
					.drivePosition()
					.times(Constants.Drivetrain.wheelCircumference.in(Units.Meters))
					.minus(Units.Meters.of(this.lastModulePositions[i]))
					.in(Units.Meters),
				Rotation2d.fromRadians(azimuthPositions[i].in(Units.Radians))
			);
			this.lastModulePositions[i] = Robot.cont.drivetrain.modules[i].drivePosition().in(Units.Meters);
		}

		final ChassisSpeeds cs = Robot.cont.drivetrain.kinematics.toChassisSpeeds(measuredStatesDiff);

		this.simOdometry = this.simOdometry
			.exp(new Twist2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond, cs.omegaRadiansPerSecond));
	}

	@Override
	public void reset() { this.simOdometry = new Pose2d(); }
}
