package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DrivetrainModifier extends SubsystemBase {
	public static abstract class Modification extends Command {
		public Modification() { this.addRequirements(Robot.cont.mod); }

		public abstract ChassisSpeeds modify(ChassisSpeeds control);

		@Override
		public boolean runsWhenDisabled() { return false; }
	}

	public ChassisSpeeds modify(final ChassisSpeeds control) {
		return ((Modification) this.getCurrentCommand()).modify(control);
	}
}
