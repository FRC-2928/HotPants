package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverOI extends BaseOI {
	public DriverOI(final XboxController controller) { super(controller); }

	public final Supplier<Double> moveAxial = this.controller::getLeftY;
	public final Supplier<Double> moveLateral = this.controller::getLeftX;

	public final Supplier<Double> moveTheta = this.controller::getRightX;

	public final Supplier<Double> moveRotationX = this.controller::getRightX;
	public final Supplier<Double> moveRotationY = this.controller::getRightY;

	public final Supplier<Double> slow = this.controller::getRightTriggerAxis;

	public final Trigger lock = new JoystickButton(this.controller, XboxController.Button.kLeftBumper.value);

	public final Trigger resetFOD = new JoystickButton(this.controller, XboxController.Button.kY.value);
}
