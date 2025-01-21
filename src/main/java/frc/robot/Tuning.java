package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.units.Units;

public final class Tuning {
	private Tuning() { throw new IllegalCallerException("Cannot instantiate `Tuning`"); }

	public static final LoggedDashboardNumber intakeSpeed = new LoggedDashboardNumber
		("Tuning/SpeedIntakePercent",
		.8);

	public static final LoggedDashboardNumber flywheelVelocity = new LoggedDashboardNumber(
		"Tuning/FlywheelSpeed",
		Constants.Shooter.flywheels.speakerVelocity.in(Units.RotationsPerSecond)
	);
	public static final LoggedDashboardNumber flywheelVelocityThreshold = new LoggedDashboardNumber(
		"Tuning/FlywheelSpeedThreshold",
		Constants.Shooter.flywheels.speakerVelocityThreshold.in(Units.RotationsPerSecond)
	);

	public static final LoggedDashboardNumber ampAngle = new LoggedDashboardNumber(
		"Tuning/AmpAngle",
		Constants.Shooter.shootAmp.in(Units.Degrees)
	);
	public static final LoggedDashboardNumber ampPower = new LoggedDashboardNumber(
		"Tuning/AmpPower",
		Constants.Shooter.flywheels.ampPower
	);
	public static final LoggedDashboardNumber drivetrainP = new LoggedDashboardNumber(
		"Tuning/Drivetrain P",
		0.15
	);
	public static final LoggedDashboardNumber shootSpeakerPivotThreshold = new LoggedDashboardNumber(
		"Tuning/ShootSpeakerPivotThreshold",
		1.25
	);
	public static final LoggedDashboardNumber shootSpeakerExponent = new LoggedDashboardNumber(
		"Tuning/ShootSpeakerExponent",
		1
	);

	public static final LoggedDashboardNumber ferryAngle = new LoggedDashboardNumber(
		"Tuning/FerryAngle",
		Constants.Shooter.shootFerry.in(Units.Degrees)
	);
	public static final LoggedDashboardNumber subAngle = new LoggedDashboardNumber(
		"Tuning/SubAngle",
		Constants.Shooter.shootSub.in(Units.Degrees)
	);
}
