package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Drivetrain;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

public final class AutonomousRoutines {

	public static SendableChooser<Command> createAutonomousChooser(final Drivetrain drivetrain) {

		final SendableChooser<Command> chooser = new SendableChooser<>();

		chooser.addOption(
			"Drive test trajectory",
			new SequentialCommandGroup(
				// new WaitCommand(.75)
				runTrajectory("test", drivetrain)
			)
		);

		chooser.setDefaultOption(
			"Wait do nothing",
			new SequentialCommandGroup(
				new WaitCommand(.75)
			)
		);

		return chooser;
	}

	public static Command runTrajectory(String trajectoryName, Drivetrain drivetrain) {

		ChoreoTrajectory traj = Choreo.getTrajectory(trajectoryName);
		SmartDashboard.putString("Traj Name", trajectoryName);
		SmartDashboard.putNumber("Linear X", traj.getInitialPose().getX());
		SmartDashboard.putNumber("Linear Y", traj.getInitialPose().getY());
		
		var thetaController = new PIDController(0.5, 0, 0);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		// Whether or not to mirror the path based on alliance
		// Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
		// 		mirror = alliance.isPresent() && alliance.get() == Alliance.Red;
	
		Command swerveCommand = Choreo.choreoSwerveCommand(
			traj, // Choreo trajectory from above
			drivetrain::getPoseEstimation, // A function that returns the current field-relative pose of the robot: 
								// your wheel or vision odometry
			new PIDController(0.1, 0.0, 0.0), // PIDController for field-relative X
													// translation (input: X error in meters,// output: m/s).
			new PIDController(0.1, 0.0, 0.0), // PIDController for field-relative Y
													// translation (input: Y error in meters, output: m/s).
			thetaController, // PID constants to correct for rotation error
			(ChassisSpeeds speeds) -> drivetrain.drive( // needs to be robot-relative
				speeds.vxMetersPerSecond,
				speeds.vyMetersPerSecond,
				speeds.omegaRadiansPerSecond,
				false),
			() -> true, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
			drivetrain // The subsystem(s) to require, typically your drive subsystem only
		);

			return Commands.sequence(
			Commands.runOnce(() -> drivetrain.resetOdometry(traj.getInitialPose())),
			swerveCommand,
			drivetrain.run(() -> drivetrain.drive(0, 0, 0, false))
		);

	}
}
