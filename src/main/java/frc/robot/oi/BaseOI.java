package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class BaseOI {
	public CommandXboxController controller;
	public final XboxController hid;

	/**
	 * C-Stop, the most useful command you'll ever have
	 * Press this magical button (start/left "window" button) to stop *all running commands*
	 */
	public final Trigger cstop;

	protected BaseOI(final CommandXboxController controller) {
		this.controller = controller;
		this.hid = controller.getHID();

		this.cstop = this.controller.start();
	}
}
