package frc.robot.oi;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;

public class DriverOI extends BaseOI {
    public DriverOI(XboxController controller) { super(controller); }

    public final Supplier<Double> moveAxial = this.controller::getLeftY;

    public final Supplier<Double> moveLateral = this.controller::getLeftX;

    public final Supplier<Double> moveTheta = this.controller::getRightX;
}
