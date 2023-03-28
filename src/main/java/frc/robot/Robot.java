package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.oi.DriverOI;

public class Robot extends TimedRobot {
	public static Robot instance;

	private Command autonomousCommand;

	public DriverOI driverOI = new DriverOI(new XboxController(0));
	public DriverOI operatorOI = new DriverOI(new XboxController(1));

	public Robot() {
		Utils.ensureSingleton(this);
		Robot.instance = this;
	}

	// ROBOT //

	@Override
	public void robotInit() {}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	// DISABLED //

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	// AUTONOMOUS //

	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance().cancelAll();

		this.autonomousCommand = null;

		if(this.autonomousCommand != null) this.autonomousCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	// TELEOP //

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	// TEST //

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
