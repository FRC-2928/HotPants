package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

public final class AutonomousRoutines {

	public static SendableChooser<Command> createAutonomousChooser(final Drivetrain drivetrain) {

		final SendableChooser<Command> chooser = new SendableChooser<>();

		chooser.addOption("Drive test trajectory", new SequentialCommandGroup(runTrajectory("test", drivetrain)));

		// chooser.addOption(
		// 	"Drive testorig trajectory",
		// 	new SequentialCommandGroup(
		// 		// new WaitCommand(.75)
		// 		runTrajectory("testorig", drivetrain)
		// 	)
		// );

		chooser.setDefaultOption("Wait do nothing", new SequentialCommandGroup(new WaitCommand(.75)));

		return chooser;
	}

	public static Command runTrajectory(String trajectoryName, Drivetrain drivetrain) {

		ChoreoTrajectory traj = Choreo.getTrajectory(trajectoryName);
		SmartDashboard.putString("Traj Name", trajectoryName);
		SmartDashboard.putNumber("Traj Linear X", traj.getInitialPose().getX());
		SmartDashboard.putNumber("Traj Linear Y", traj.getInitialPose().getY());
		SmartDashboard.putNumber("Traj Rotation", traj.getInitialPose().getRotation().getDegrees());

		var thetaController = new PIDController(0.1, 0, 0);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		// Whether or not to mirror the path based on alliance
		boolean mirror = Robot.instance.alliance.isPresent() && Robot.instance.alliance.get() == Alliance.Red;

		Command swerveCommand = Choreo
			.choreoSwerveCommand(
				traj, // Choreo trajectory from above
				drivetrain::getPose, // A function that returns the current field-relative pose of the robot: 
				// your wheel or vision odometry
				new PIDController(0.1, 0.0, 0.0), // PIDController for field-relative X
				// translation (input: X error in meters, output: m/s).
				new PIDController(0.1, 0.0, 0.0), // PIDController for field-relative Y
				// translation (input: Y error in meters, output: m/s).
				thetaController, // PID constants to correct for rotation error
				(ChassisSpeeds speeds) -> drivetrain
					.drive(
						// needs to be robot-relative
						speeds.vxMetersPerSecond,
						speeds.vyMetersPerSecond,
						speeds.omegaRadiansPerSecond,
						false
					),
				() -> mirror, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
				drivetrain // The subsystem(s) to require, typically your drive subsystem only
			);

		return Commands
			.sequence(
				Commands.runOnce(() -> drivetrain.setModuleStates(SwerveModule.State.forward())),
				// new WaitCommand(0.5),
				Commands.runOnce(() -> drivetrain.resetOdometry(traj.getInitialPose())),
				swerveCommand,
				drivetrain.run(() -> drivetrain.drive(0, 0, 0, false))
			);

	}
}
