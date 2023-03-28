package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;

public class DriverOI extends BaseOI {
    public DriverOI(XboxController controller) { super(controller); }

    public final double moveAxial() { return this.controller.getLeftX(); }

    public final double moveLateral() { return this.controller.getLeftY(); }

    public final double moveTheta() { return this.controller.getRightX(); }
}
