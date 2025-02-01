package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.Trajectory;
import choreo.trajectory.SwerveSample;
import choreo.Choreo;
import choreo.auto.AutoFactory;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.path.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.commands.drivetrain.DriveTime;
import frc.robot.commands.drivetrain.VoltageRampCommand;

public final class Autonomous {
	public static SendableChooser<Command> createAutonomousChooser() {
		final SendableChooser<Command> chooser = new SendableChooser<>();
		AutoFactory autoFactory = Robot.cont.drivetrain.autoFactory;

		/*chooser
			.addOption(
				"[comp] Five Note",
				new SequentialCommandGroup(
					new ReadyShooter(Units.Degrees.of(114), true),
					Autonomous.setInitialPose("MiddleFiveNote.1"),
					new WaitCommand(0.2),
					new ShootFixed(false, 2),
					Autonomous
						.path("MiddleFiveNote.1")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyIntake, false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamic("MiddleFiveNote.3")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					new ReadyShooter(Constants.Shooter.readyIntake, false),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamic("MiddleFiveNote.4")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					new ReadyShooter(Constants.Shooter.readyIntake, false),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamic("MiddleFiveNote.5")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("MiddleFiveNote.5"),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("MiddleFiveNote.6")
						.deadlineFor(
							new WaitCommand(2).andThen(new ReadyShooter(Constants.Shooter.readyShootRear, true))
						),
					new ShootSpeaker(false, 2)
				)
			);*/

		chooser
			.addOption(
				"[Test] Forward Back",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("forwardBack"),
					Autonomous.path("ForwardBack")
				)
			);

		/*chooser
			.addOption(
				"[comp] Amp Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("AmpSideCenterNote.1"),
					new ShootFixed(false, 2),
					Autonomous.path("AmpSideCenterNote.1").deadlineFor(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.2")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("AmpSideCenterNote.3").deadlineFor(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.4")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2)
				)
			);*/

		/*chooser
			.addOption(
				"[testing] Squirrel Amp Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("AmpSideCenterNote.1"),
					new ShootFixed(false, 2),
					Autonomous.path("AmpSideCenterNote.1").deadlineFor(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					new WaitCommand(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.2")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("AmpSideCenterNote.3").deadlineFor(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.4")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2)
				)
			);*/

		/*chooser
			.addOption(
				"[testing] Delayed Amp Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("AmpSideCenterNote.1"),
					new WaitCommand(10),
					new ShootFixed(false, 2),
					Autonomous.path("AmpSideCenterNote.1").deadlineFor(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.2")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("AmpSideCenterNote.3").deadlineFor(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(2),
					Autonomous
						.dynamicThen("AmpSideCenterNote.4")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2)
				)
			);*/

		/*chooser
			.addOption(
				"[comp] The Jamp",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("jamp.1"),
					new ShootFixedDiag(false, 2),
					Autonomous.path("jamp.1").deadlineFor(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(1.3),
					Autonomous
						.dynamicThen("jamp.2")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("jamp.3").deadlineFor(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(1.3),
					Autonomous
						.dynamicThen("jamp.4")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootRear, true)),
					new ShootSpeaker(false, 2)
				)
			);*/

		/*chooser
			.addOption(
				"[comp] The Close Jamp",
				new SequentialCommandGroup(
					Autonomous.setInitialPose("closeJamp.1"),
					new ShootFixedDiag(false, 2),
					Autonomous.path("closeJamp.1").deadlineFor(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(1.3),
					Autonomous
						.dynamicThen("closeJamp.2")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootFront, true)),
					new ShootSpeaker(false, 2),
					Autonomous.path("closeJamp.3").deadlineFor(new IntakeGround(false)),
					new IntakeGround(true).withTimeout(1.3),
					Autonomous
						.dynamicThen("closeJamp.4")
						.deadlineFor(new ReadyShooter(Constants.Shooter.readyShootFront, true)),
					new ShootSpeaker(false, 2)
				)
			);*/

		// chooser
		// 	.addOption(
		// 		"[comp] The Jource",
		// 		new SequentialCommandGroup(
		// 			Autonomous.setInitialPose("jource.1"),
		// 			new ShootFixedDiag(false, 2),
		// 			Autonomous.path("jource.1").deadlineFor(new IntakeGround(false)),
		// 			new IntakeGround(true).withTimeout(1.3),
		// 			Autonomous.dynamicThen("jource.2"),
		// 			new ShootSpeaker(false, 2),
		// 			Autonomous.path("jource.3").deadlineFor(new IntakeGround(false)),
		// 			new IntakeGround(true).withTimeout(1.3)
		// 		)
		// 	);
		// chooser
		// 	.addOption(
		// 		"[testing] Source Side Bulldoze",
		// 		new SequentialCommandGroup(
		// 			Autonomous.setInitialPose("Bulldoze.1"),
		// 			new ReadyShooter(Constants.Shooter.readyShootRear, true),
		// 			new ShootFixed(false, 2),
		// 			Autonomous.path("Bulldoze.1"),
		// 			new IntakeGround(true).withTimeout(2)
		// 		)
		// 	);
		// chooser
		// 	.addOption(
		// 		"[testing] forward only",
		// 		new SequentialCommandGroup(Autonomous.setInitialPose("forwardBack.1"), Autonomous.path("forwardBack.1"))
		// 	);
		// chooser
		// 	.addOption(
		// 		"[testing] forward Back",
		// 		new SequentialCommandGroup(
		// 			Autonomous.setInitialPose("forwardBack.1"),
		// 			Autonomous.path("forwardBack.1"),
		// 			new LookForNote(Units.Radians.of(Math.PI/3)),
		// 			new IntakeGround(true).withTimeout(2),
		// 			Autonomous.dynamic("forwardBack.2"),
		// 			Autonomous.path("forwardBack.2")
		// 		)
		// 	);

		chooser.addOption("Center On Limelight", 
			new CenterLimelight(Units.Meters.of(0.5),Units.Meters.of(0.5))
		);
		// chooser
		// 	.addOption("[testing] dynamic path back", new SequentialCommandGroup(Autonomous.dynamic("forwardBack.2")));
		// chooser.addOption("[testing] return to start", new SequentialCommandGroup(Autonomous.path("forwardBack.2")));
		// chooser
		// 	.addOption(
		// 		"[testing] forward path too back",
		// 		new SequentialCommandGroup(
		// 			Autonomous.setInitialPose("forwardBack.1"),
		// 			Autonomous.path("forwardBack.1"),
		// 			new IntakeGround(true).withTimeout(2),
		// 			Autonomous.dynamic("forwardBack.2")
		// 		)
		// 	);
		chooser.addOption("[testing] voltage ramp", new VoltageRampCommand());
		chooser.addOption("SimpleScore", Commands.sequence(autoFactory.resetOdometry("SimpleScore"),
		autoFactory.trajectoryCmd("SimpleScore")));
		return chooser;
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
				Constants.fieldWidth.in(Units.Meters) - initialPose.getX(),
				Constants.fieldDepth.in(Units.Meters) - initialPose.getY(),
				initialPose.getRotation().unaryMinus()
			);
		} else return initialPose;
	}
}
