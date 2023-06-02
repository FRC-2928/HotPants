package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
	public static Robot instance;

	private Command autonomousCommand;

	public final DriverOI driverOI = new DriverOI(new XboxController(0));
	public final DriverOI operatorOI = new DriverOI(new XboxController(1));

    public final Drivetrain drivetrain = new Drivetrain();

	public Robot() {
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

        this.drivetrain.setDefaultCommand(new Drive(this.drivetrain, this.driverOI));
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
