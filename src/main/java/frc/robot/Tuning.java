package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.units.Units;

public final class Tuning {
    private Tuning() { throw new IllegalCallerException("Cannot instantiate `Tuning`"); }

    public static final LoggedDashboardNumber intakeSpeed = new LoggedDashboardNumber("Tuning/SpeedIntakePercent", .8);
}
