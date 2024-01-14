package frc.robot;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
	public static Robot instance;

	public final RobotContainer cont = new RobotContainer();

	public Robot() {
		Robot.instance = this;

		ConduitApi.getInstance().configurePowerDistribution(Constants.CAN.pdh, ModuleType.kRev.value);
		Logger.start();
	}

	// ROBOT //

	@Override
	public void robotInit() { cont.init(); }

	@Override
	public void robotPeriodic() { CommandScheduler.getInstance().run(); }

	// DISABLED //

	@Override
	public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();
		cont.disabled();
	}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() { cont.enabled(); }

	// AUTONOMOUS //

	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance().cancelAll();

		cont.auto();
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	// TELEOP //

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();

		cont.teleop();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	// TEST //

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();

		cont.test();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
