package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
	public final LoggedDashboardChooser<String> driveModeChooser;
	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));
	public final Drivetrain drivetrain;
	public static boolean ledState = false;
	private final AutoChooser autoChooser;
	public RobotContainer() {
		Robot.instance.container = this;
		Robot.cont = this;

		Tuning.flywheelVelocity.get(); // load the class to put the tuning controls on the dashboard
		this.drivetrain = new Drivetrain();

		autoChooser = Autonomous.getChoreoAutoChooser();
		SmartDashboard.putData("Autonomous Routine", autoChooser);
		RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

		this.driveModeChooser = new LoggedDashboardChooser<>(
			"Drive Mode",
			JoystickDrive.createDriveModeChooser()
		);

		this.driverOI.configureControls();
		this.operatorOI.configureControls();
	}

	public String getDriveMode() { return this.driveModeChooser.get(); }
}
