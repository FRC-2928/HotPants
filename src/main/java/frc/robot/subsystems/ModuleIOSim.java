// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants;

public class ModuleIOSim implements ModuleIO {
	private static final double LOOP_PERIOD_SECS = 0.02;

	private final DCMotorSim drive = new DCMotorSim(DCMotor.getFalcon500(1), 6.75, 0.025);
	private final DCMotorSim azimuth = new DCMotorSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004);
	private final EncoderSim cancoder = new EncoderSim(null);

	@Override
	public void updateInputs(final ModuleIOInputs inputs) {
		this.drive.update(ModuleIOSim.LOOP_PERIOD_SECS);
		this.azimuth.update(ModuleIOSim.LOOP_PERIOD_SECS);

		inputs.driveVelocity = Units.MetersPerSecond
			.of(
				Units.RadiansPerSecond.of(this.drive.getAngularVelocityRadPerSec()).in(Units.RotationsPerSecond)
					* Constants.Drivetrain.wheelCircumference.in(Units.Meters)
			);
		inputs.driveCurrent = Units.Amps.of(Math.abs(this.drive.getCurrentDrawAmps()));

		inputs.angle = Units.Radians.of(this.azimuth.getAngularPositionRad());
		inputs.azimuthCurrent = Units.Amps.of(Math.abs(this.azimuth.getCurrentDrawAmps()));
	}

	@Override
	public void setDriveVoltage(final double volts) { this.drive.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0)); }

	@Override
	public void setAzimuthVoltage(final double volts) {
		this.azimuth.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
	}
}
