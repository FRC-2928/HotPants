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

import edu.wpi.first.units.*;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
	@AutoLog
	public static class GyroIOInputs {
		public boolean connected = false;
		public Measure<Angle> yawPosition = Units.Rotations.zero();
		public Measure<Velocity<Angle>> yawVelocityRadPerSec = Units.RotationsPerSecond.zero();
	}

	public default void updateInputs(final GyroIOInputs inputs) {}

	public default void reset() {}
}
