package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
	public final LoggedDashboardChooser<
		Command> autoChooser = new LoggedDashboardChooser<>("Autonomous Routine", new SendableChooser<>());

	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	// public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));

	public final Drivetrain drivetrain = new Drivetrain();

	public RobotContainer() {
		this.driverOI.lock.whileTrue(new LockWheels(this.drivetrain, this.driverOI));
		this.driverOI.resetFOD.whileTrue(new RunCommand(() -> this.drivetrain.gyro.setYaw(0)));
	}

	// public void init() {}

	// public void disabled() {}

	// public void enabled() {}

	// public void auto() {}

	public void teleop() { this.drivetrain.setDefaultCommand(new Drive(this.drivetrain, this.driverOI)); }

	// public void test() {}
}
