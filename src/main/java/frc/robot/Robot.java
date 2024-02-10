package frc.robot;

import java.util.Optional;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
	public static Robot instance;
	public Optional<DriverStation.Alliance> alliance;

	private Command autonomousCommand;
	private RobotContainer container;
	public Constants.Mode currentMode = Constants.Mode.SIM;

	public Robot() {
		super();
		Robot.instance = this;
	}

	@Override
	public void robotInit() {
		ConduitApi.getInstance().configurePowerDistribution(Constants.CAN.pdh, ModuleType.kRev.value);

		if(Robot.isReal()){
			this.currentMode = Constants.Mode.REAL;
		}

		switch(this.currentMode) {
		case REAL:
			// Running on a real robot, log to a USB stick
			Logger.addDataReceiver(new WPILOGWriter("/U"));
			Logger.addDataReceiver(new NT4Publisher());
			break;

		case SIM:
			// Running a physics simulator, log to NT
			Logger.addDataReceiver(new NT4Publisher());
			break;

		case REPLAY:
			// Replaying a log, set up replay source
			setUseTiming(false); // Run as fast as possible
			String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
			break;
		}

		Logger.start();

		this.alliance = DriverStation.getAlliance();

		this.container = new RobotContainer();
	}

	@Override
	public void robotPeriodic() { CommandScheduler.getInstance().run(); }

	// DISABLED //
	@Override
	public void disabledInit() { CommandScheduler.getInstance().cancelAll(); }

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	// AUTONOMOUS //

	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance().cancelAll();

		// Get selected routine from the SmartDashboard
		this.autonomousCommand = this.container.getAutonomousCommand();

		// schedule the autonomous command (example)
		if(this.autonomousCommand != null) {
			this.autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	// TELEOP //

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();

		this.container.teleop();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	// TEST //

	@Override
	public void testInit() { CommandScheduler.getInstance().cancelAll(); }

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
