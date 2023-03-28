package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;

public abstract class BaseOI {
    public final XboxController controller;

    protected BaseOI(XboxController controller) {
        this.controller = controller;
    }
}
