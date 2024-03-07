package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class DriverOI extends BaseOI {
	public DriverOI(final CommandXboxController controller) {
		super(controller);

		this.driveAxial = this.controller::getLeftY;
		this.driveLateral = this.controller::getLeftX;

		if(Constants.mode == Mode.REAL) {
			this.driveFORX = this.controller::getRightX;
			this.driveFORY = () -> -this.controller.getRightY();
		} else {
			this.driveFORX = () -> this.hid.getRawAxis(2);
			this.driveFORY = () -> this.hid.getRawAxis(3);
		}
		
		this.slow = () -> MathUtil.interpolate(1, 0.5, this.controller.getRightTriggerAxis());
		this.alignShooter = this.controller::getRightTriggerAxis;

		this.lockWheels = this.controller.leftBumper();

		this.resetFOD = this.controller.y();
		this.servoLeft = this.controller.a();
		this.servoRight = this.controller.b();
		
	}

	public final Supplier<Double> driveAxial;
	public final Supplier<Double> driveLateral;

	public final Supplier<Double> driveFORX;
	public final Supplier<Double> driveFORY;

	public final Supplier<Double> slow;
	public final Supplier<Double> alignShooter;

	public final Trigger lockWheels;
	public final Trigger servoLeft;
	public final Trigger servoRight;

	public final Trigger resetFOD;
}
