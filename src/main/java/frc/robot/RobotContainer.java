package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.GyroIO;
import frc.robot.commands.drivetrain.GyroIOPigeon2;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.commands.drivetrain.ModuleIO;
import frc.robot.commands.drivetrain.ModuleIOSim;
import frc.robot.commands.drivetrain.ModuleIOTalonFX;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
	public final LoggedDashboardChooser<
		Command> autoChooser = new LoggedDashboardChooser<>("Autonomous Routine", new SendableChooser<>());

	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	// public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));

	public final Drivetrain drivetrain;

	public RobotContainer() { 
		this.configureDriverControls(); 
		switch (Constants.currentMode) {
			case REAL:
				// Real robot, instantiate hardware IO implementations      
				drivetrain = new Drivetrain(
					new GyroIOPigeon2(),
					new ModuleIOTalonFX(0),
					new ModuleIOTalonFX(1),
					new ModuleIOTalonFX(2),
					new ModuleIOTalonFX(3));
				break;

			case SIM:
				// Sim robot, instantiate physics sim IO implementations
				drivetrain =
					new Drivetrain(
						new GyroIO() {},
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim());
				break;

			default:
				// Replayed robot, disable IO implementations
				drivetrain =
					new Drivetrain(
						new GyroIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {});
				break;
		}
	}

	private void configureDriverControls() {
		// this.driverOI.resetFOD.whileTrue(new RunCommand(() -> this.drivetrain.gyro.setYaw(0)));
		this.driverOI.resetFOD.whileTrue(new RunCommand(() -> this.drivetrain.resetGyro()));
		this.driverOI.lock.whileTrue(new LockWheels(this.drivetrain, this.driverOI));
	}
	
	public void teleop() { this.drivetrain.setDefaultCommand(new JoystickDrive(this.drivetrain, this.driverOI)); }

}
