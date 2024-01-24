package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class LockWheels extends Command {
	private final Drivetrain drivetrain;
	private final DriverOI oi;

	public LockWheels(final Drivetrain drivetrain, final DriverOI oi) {
		this.drivetrain = drivetrain;
		this.oi = oi;

		this.addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		this.drivetrain.setModuleStates(SwerveModule.State.locked());
		if(this.oi != null) this.oi.hid.setRumble(RumbleType.kBothRumble, 0.25);
	}

	@Override
	public void end(final boolean interrupted) {
		this.drivetrain.setModuleStates(SwerveModule.State.forward());
		if(this.oi != null) this.oi.hid.setRumble(RumbleType.kBothRumble, 0);
	}
}
