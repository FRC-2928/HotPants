package frc.robot;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
	private Command autonomousCommand;
	private RobotContainer robotContainer;

	@Override
	public void robotInit() { 		
		ConduitApi.getInstance().configurePowerDistribution(Constants.CAN.pdh, ModuleType.kRev.value);

		Logger.start();

		this.robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() { 
		CommandScheduler.getInstance().run(); 
	}

	// DISABLED //
	@Override
	public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	// AUTONOMOUS //

	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance().cancelAll();
		// Get selected routine from the SmartDashboard
		this.autonomousCommand = this.robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (this.autonomousCommand != null) {
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
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
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
