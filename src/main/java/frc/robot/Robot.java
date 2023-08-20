package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.LockWheels;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.Drivetrain;

public class Robot extends LoggedRobot {
	public static Robot instance;
    public static final Logger log = Logger.getInstance();

	private Command autonomousCommand;

	public final DriverOI driverOI = new DriverOI(new XboxController(0));
	public final OperatorOI operatorOI = new OperatorOI(new XboxController(1));

    public final Drivetrain drivetrain = new Drivetrain();

	public Robot() {
		Robot.instance = this;

        this.driverOI.lock.whileTrue(new LockWheels(this.drivetrain, this.driverOI));
        this.driverOI.resetFOD.whileTrue(new RunCommand(() -> {
            this.drivetrain.gyro.setYaw(0);
            System.out.printf("rst %s\n", this.drivetrain.gyro.getRotation2d());
        }));
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
