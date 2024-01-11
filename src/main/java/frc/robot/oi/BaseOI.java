package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class BaseOI {
	public XboxController controller;

	public final Trigger cstopButton;

	protected BaseOI(final XboxController controller) {
		this.controller = controller;

		this.cstopButton = new JoystickButton(this.controller, XboxController.Button.kStart.value);
	}
}
