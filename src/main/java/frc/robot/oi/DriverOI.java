package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverOI extends BaseOI {
	public DriverOI(final CommandXboxController controller) {
		super(controller);

		this.moveAxial = this.controller::getLeftY;
		this.moveLateral = this.controller::getLeftX;

		this.moveTheta = this.controller::getRightX;

		this.moveRotationX = this.controller::getRightX;
		this.moveRotationY = this.controller::getRightY;

		this.slow = this.controller::getRightTriggerAxis;

		this.lock = this.controller.leftBumper();

		this.resetFOD = this.controller.y();
	}

	public final Supplier<Double> moveAxial;
	public final Supplier<Double> moveLateral;

	public final Supplier<Double> moveTheta;

	public final Supplier<Double> moveRotationX;
	public final Supplier<Double> moveRotationY;

	public final Supplier<Double> slow;

	public final Trigger lock;

	public final Trigger resetFOD;
}
