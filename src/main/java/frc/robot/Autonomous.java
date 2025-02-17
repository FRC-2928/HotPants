package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.Trajectory;
import choreo.trajectory.SwerveSample;
import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.commands.drivetrain.VoltageRampCommand;

public final class Autonomous {
	public static SendableChooser<Command> createAutonomousChooser() {
		final SendableChooser<Command> chooser = new SendableChooser<>();
		AutoFactory autoFactory = Robot.cont.drivetrain.autoFactory;

		chooser
			.addOption(
				"[Test] Forward Back",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("forwardBack"),
					Autonomous.path("forwardBack")
				)
			);
			chooser
			.addOption(
				"[Test] Forward Back Choreo",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("forwardBack"),
					autoFactory.trajectoryCmd("forwardBack")
				)
			);

		chooser.addOption("Center On Limelight", 
			new CenterLimelight()
		);
		
		chooser.addOption("[testing] voltage ramp", new VoltageRampCommand());
		chooser.addOption("SimpleScore", Commands.sequence(autoFactory.trajectoryCmd("SimpleScore")));
		chooser.addOption("SimpleFromRight", Commands.sequence(
			// autoFactory.resetOdometry("SimpleFromRight"),
			autoFactory.trajectoryCmd("SimpleFromRight", 0),
			// new WaitCommand(2),
			autoFactory.trajectoryCmd("SimpleFromRight", 1),
			Robot.cont.drivetrain.haltCommand()
		));
		chooser.addOption("SimpleFromRight Path Planner gen", Commands.sequence(
			new PathPlannerAuto("SimpleFromRightTest")
		));
		chooser.addOption("GoFastRotate", 
		new RunCommand(() -> {
			Robot.cont.drivetrain.controlRobotOriented(
				new ChassisSpeeds(
					100,0,Math.PI
				));
			}
		).withTimeout(2));
		chooser.addOption("PP_SimpleFromRight", 
			Autonomous.path("SimpleFromRight")
		);
		return chooser;
	}

	public static AutoChooser getChoreoAutoChooser() {
		final AutoChooser choreoChooser = new AutoChooser();
		AutoFactory autoFactory = Robot.cont.drivetrain.autoFactory;

		choreoChooser.addCmd("SimpleFromRight", () -> Commands.sequence(
			// autoFactory.resetOdometry("SimpleFromRight"),
			new InstantCommand(() -> {Logger.recordOutput("Autonomous/StartedCommand", true);}),
			autoFactory.trajectoryCmd("StartToF"),
			new InstantCommand(() -> {Logger.recordOutput("Autonomous/FinishedPath", true);}),
			// new WaitCommand(2),
			autoFactory.trajectoryCmd("FtoB1Reverse")
			// Robot.cont.drivetrain.haltCommand()
		));

		choreoChooser.addCmd("SimpleScore", () -> Commands.sequence(autoFactory.trajectoryCmd("SimpleScore")));

		return choreoChooser;
	}

	public static Command setInitialPose(final String name) {
		final Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(name);
		// try {
			final Pose2d initial = traj.get().getPoses()[0];

			return Commands.runOnce(() -> {
				Robot.cont.drivetrain.reset(initial);
				

				Logger.recordOutput("Drivetrain/Auto/x0", initial.getX());
				Logger.recordOutput("Drivetrain/Auto/y0", initial.getY());
				Logger.recordOutput("Drivetrain/Auto/r0", initial.getRotation().getDegrees());
				Logger.recordOutput("Drivetrain/Auto/AllyPose", initial);
			});
		// } catch (Exception e) {
		// 	System.out.println(e.toString());
		// 	return new InstantCommand();
		// }
	}

	public static Command path(final String name) {
		try {
			final PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory(name);
			return AutoBuilder.followPath(choreoPath);
		} catch (Exception e) {
			System.out.println(e.toString());
			return new InstantCommand();
		}

	}

	public static Command pathPlannerpath(final String name) {
		try {
			final PathPlannerPath path = PathPlannerPath.fromPathFile(name);
			return AutoBuilder.followPath(path);
		} catch (Exception e) {
			System.out.println(e.toString());
			return new InstantCommand();
		}
	}

	public static Command dynamic(final String next, final double maxvel) {
		final Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(next);

		return AutoBuilder
			.pathfindToPoseFlipped(
				traj.get().getPoses()[0],
				new PathConstraints(maxvel, 2, Constants.Drivetrain.maxAngularVelocity.in(Units.RadiansPerSecond), 2)
			)
			.alongWith(
				new InstantCommand(() -> Logger.recordOutput("Drivetrain/Auto/DynamicTarget", traj.get().getPoses()[0]))
			);
	}

	public static Command dynamic(final String next) {
		return Autonomous.dynamic(next, Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond));
	}

	public static Command dynamicThen(final String next) {
		try {
			final PathPlannerPath traj = PathPlannerPath.fromChoreoTrajectory(next);

			return AutoBuilder
				.pathfindThenFollowPath(
					traj,
					new PathConstraints(
						Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond),
						3,
						Constants.Drivetrain.maxAngularVelocity.in(Units.RadiansPerSecond),
						2
					)
				)
				.alongWith(
					new InstantCommand(
						() -> Logger
							.recordOutput("Drivetrain/Auto/DynamicTarget", Choreo.loadTrajectory(next).get().getPoses()[0])
					)
				);
		} catch (Exception e) {
			System.out.println(e.toString());
			return new InstantCommand();
		}
	}

	/*
	 * Returns the original or mirrored pose depending on alliance color (since the field is flipped)
	 */
	private static Pose2d getPoseForAlliance(final Pose2d initialPose) {
		if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
			return new Pose2d(
				Constants.FIELD_LAYOUT.getFieldLength() - initialPose.getX(),
				Constants.FIELD_LAYOUT.getFieldWidth() - initialPose.getY(),
				initialPose.getRotation().unaryMinus()
			);
		} else return initialPose;
	}
}
