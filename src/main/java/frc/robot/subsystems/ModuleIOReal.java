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

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveModule.Place;

public class ModuleIOReal implements ModuleIO {
	public final Place place;

	public final TalonFX drive;
	public final TalonFX azimuth;
	public final CANcoder cancoder;

	public final StatusSignal<Double> drivePosition;
	public final StatusSignal<Double> driveVelocity;
	public final StatusSignal<Double> driveCurrent;

	public final StatusSignal<Double> azimuthCurrent;

	public final StatusSignal<Double> angle;

	public final Measure<Angle> absoluteEncoderOffset;

	public ModuleIOReal(final Place place) {
		this.place = place;

		switch(place) {
		case FrontLeft:
			this.azimuth = new TalonFX(Constants.CAN.CTRE.swerveFrontLeftAzimuth, Constants.CAN.CTRE.bus);
			this.drive = new TalonFX(Constants.CAN.CTRE.swerveFrontLeftDrive, Constants.CAN.CTRE.bus);
			this.cancoder = new CANcoder(Constants.CAN.CTRE.swerveFrontLeftAzimuth, Constants.CAN.CTRE.bus);
			this.absoluteEncoderOffset = Constants.Drivetrain.swerveFrontLeftOffset;
			break;
		case FrontRight:
			this.azimuth = new TalonFX(Constants.CAN.CTRE.swerveFrontRightAzimuth, Constants.CAN.CTRE.bus);
			this.drive = new TalonFX(Constants.CAN.CTRE.swerveFrontRightDrive, Constants.CAN.CTRE.bus);
			this.cancoder = new CANcoder(Constants.CAN.CTRE.swerveFrontRightAzimuth, Constants.CAN.CTRE.bus);
			this.absoluteEncoderOffset = Constants.Drivetrain.swerveFrontRightOffset;
			break;
		case BackRight:
			this.azimuth = new TalonFX(Constants.CAN.CTRE.swerveBackRightAzimuth, Constants.CAN.CTRE.bus);
			this.drive = new TalonFX(Constants.CAN.CTRE.swerveBackRightDrive, Constants.CAN.CTRE.bus);
			this.cancoder = new CANcoder(Constants.CAN.CTRE.swerveBackRightAzimuth, Constants.CAN.CTRE.bus);
			this.absoluteEncoderOffset = Constants.Drivetrain.swerveBackRightOffset;
			break;
		case BackLeft:
			this.azimuth = new TalonFX(Constants.CAN.CTRE.swerveBackLeftAzimuth, Constants.CAN.CTRE.bus);
			this.drive = new TalonFX(Constants.CAN.CTRE.swerveBackLeftDrive, Constants.CAN.CTRE.bus);
			this.cancoder = new CANcoder(Constants.CAN.CTRE.swerveBackLeftAzimuth, Constants.CAN.CTRE.bus);
			this.absoluteEncoderOffset = Constants.Drivetrain.swerveBackLeftOffset;
			break;
		default:
			throw new RuntimeException("Invalid module index");
		}

		Robot.cont.diag.motors.add(this.azimuth);
		Robot.cont.diag.motors.add(this.drive);

		final TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
		// Peak output of 40 amps
		azimuthConfig.CurrentLimits.StatorCurrentLimit = 40.0;
		azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		azimuthConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		azimuthConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
		azimuthConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

		// Supply current limits
		azimuthConfig.CurrentLimits.SupplyCurrentLimit = 35;
		azimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		azimuthConfig.CurrentLimits.SupplyCurrentThreshold = 60;
		azimuthConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

		this.azimuth.getConfigurator().apply(azimuthConfig);
		this.azimuth.setNeutralMode(NeutralModeValue.Brake);

		final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
		// Peak output amps
		driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

		// Supply current limits
		driveConfig.CurrentLimits.SupplyCurrentLimit = 35;
		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveConfig.CurrentLimits.SupplyCurrentThreshold = 60;
		driveConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

		// driveConfig.Feedback.SensorToMechanismRatio = Constants.Drivetrain.rotationsPerMeter;
		driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

		// PID values
		driveConfig.Slot0 = Constants.Drivetrain.driveGainsSlot0;
		driveConfig.Slot1 = Constants.Drivetrain.driveGainsSlot1;

		this.drive.getConfigurator().apply(driveConfig);
		this.drive.setNeutralMode(NeutralModeValue.Brake);

		if(place == Place.FrontRight || place == Place.BackRight) this.drive.setInverted(true);

		final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
		encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
		encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		encoderConfig.MagnetSensor.MagnetOffset = this.absoluteEncoderOffset.in(Units.Rotations);
		this.cancoder.getConfigurator().apply(encoderConfig);

		this.drivePosition = this.drive.getRotorPosition();
		this.driveVelocity = this.drive.getVelocity();
		this.driveCurrent = this.drive.getStatorCurrent();

		this.azimuthCurrent = this.azimuth.getStatorCurrent();

		this.angle = this.cancoder.getAbsolutePosition();

		BaseStatusSignal.setUpdateFrequencyForAll(100, this.drivePosition, this.angle); // Required for odometry, use faster rate
		BaseStatusSignal
			.setUpdateFrequencyForAll(
				50,
				this.driveCurrent,

				this.azimuthCurrent
			);
		this.drive.optimizeBusUtilization();
		this.azimuth.optimizeBusUtilization();
	}

	@Override
	public void setDriveVoltage(final double volts) {
		this.drive.setControl(new VoltageOut(volts, true, false, false, false));
	}

	@Override
	public void setAzimuthVoltage(final double volts) {
		this.azimuth.setControl(new VoltageOut(volts, true, false, false, false));
	}

	@Override
	public void updateInputs(final ModuleIOInputs inputs) {
		BaseStatusSignal
			.refreshAll(
				this.drivePosition,
				this.driveVelocity,
				this.driveCurrent,

				this.azimuthCurrent,

				this.angle
			);

		inputs.drivePosition = Units.Rotations
			.of(this.drivePosition.getValueAsDouble())
			.divide(Constants.Drivetrain.driveGearRatio);
		inputs.driveVelocity = Units.MetersPerSecond
			.of(this.driveVelocity.getValueAsDouble() * Constants.Drivetrain.wheelCircumference.in(Units.Meters))
			.divide(Constants.Drivetrain.driveGearRatio);
		inputs.driveCurrent = Units.Amps.of(this.driveCurrent.getValueAsDouble());

		inputs.azimuthCurrent = Units.Amps.of(this.azimuthCurrent.getValueAsDouble());

		inputs.angle = Units.Rotations.of(this.angle.getValueAsDouble());
		Logger.recordOutput("Drivetrain/" + this.place.name() + "/Angle", inputs.angle);
	}
}
