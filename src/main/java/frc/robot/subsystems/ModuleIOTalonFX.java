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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule.Place;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and CANcoder
 *
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to different hardware configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such that forward motion on the drive motor will propel the robot forward) and copy the reported values from the absolute encoders using AdvantageScope. These values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;
  private final StatusSignal<Double> driveRotorPosition;

  private final StatusSignal<Double> cancoderAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.746
  private final double TURN_GEAR_RATIO = 150.0 / 7.0; // 21.43

  private final boolean isTurnMotorInverted = true;
  private final double absoluteEncoderOffset;
  // private final Rotation2d absoluteEncoderOffset;

  // Target Variables. Used only for data logging
  private double targetVelocityMetersPerSeconds = 0;
  private double targetSteerPositionRad = 0;

  // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;


  public ModuleIOTalonFX(Place place) {
    switch(place) {
    case FrontLeft:
      this.turnTalon = new TalonFX(Constants.CAN.swerveFrontLeftAzimuth, "canivore");
      this.driveTalon = new TalonFX(Constants.CAN.swerveFrontLeftDrive, "canivore");
      this.cancoder = new CANcoder(Constants.CAN.swerveFrontLeftEncoder, "canivore");
      // this.absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(Constants.CAN.swerveFrontLeftOffset)); // MUST BE CALIBRATED
      this.absoluteEncoderOffset = Constants.CAN.swerveFrontLeftOffset; // MUST BE CALIBRATED
      break;
    case FrontRight:
      this.turnTalon = new TalonFX(Constants.CAN.swerveFrontRightAzimuth, "canivore");
      this.driveTalon = new TalonFX(Constants.CAN.swerveFrontRightDrive, "canivore");
      this.cancoder = new CANcoder(Constants.CAN.swerveFrontRightEncoder, "canivore");
      // this.absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(Constants.CAN.swerveFrontRightOffset)); // MUST BE CALIBRATED
      this.absoluteEncoderOffset = Constants.CAN.swerveFrontRightOffset; // MUST BE CALIBRATED
      break;
    case BackRight:
      this.turnTalon = new TalonFX(Constants.CAN.swerveBackRightAzimuth, "canivore");
      this.driveTalon = new TalonFX(Constants.CAN.swerveBackRightDrive, "canivore");
      this.cancoder = new CANcoder(Constants.CAN.swerveBackRightEncoder, "canivore");
      // this.absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(Constants.CAN.swerveBackRightOffset)); // MUST BE CALIBRATED
      this.absoluteEncoderOffset = Constants.CAN.swerveBackRightOffset;
      break;
    case BackLeft:
      this.turnTalon = new TalonFX(Constants.CAN.swerveBackLeftAzimuth, "canivore");
      this.driveTalon = new TalonFX(Constants.CAN.swerveBackLeftDrive, "canivore");
      this.cancoder = new CANcoder(Constants.CAN.swerveBackLeftEncoder, "canivore");
      // this.absoluteEncoderOffset = new Rotation2d(Units.rotationsToRadians(Constants.CAN.swerveBackLeftOffset)); // MUST BE CALIBRATED
      this.absoluteEncoderOffset = Constants.CAN.swerveBackLeftOffset;// MUST BE CALIBRATED
      break;
    default:
      throw new RuntimeException("Invalid module index");
    }

    var azimuthConfig = new TalonFXConfiguration();
      // Peak output of 40 amps
    azimuthConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    azimuthConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    azimuthConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    azimuthConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    azimuthConfig.Slot0 = Constants.Drivetrain.turnGains;
    turnTalon.getConfigurator().apply(azimuthConfig);

    turnTalon.setNeutralMode(NeutralModeValue.Brake);

    var driveConfig = new TalonFXConfiguration();
      // Peak output of 40 amps
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    // Peak output of 8 volts
    driveConfig.Voltage.PeakForwardVoltage = 8;
    driveConfig.Voltage.PeakReverseVoltage = -8;

    driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

    driveConfig.Slot0 = Constants.Drivetrain.driveGains;
    driveTalon.getConfigurator().apply(driveConfig);

    driveTalon.setNeutralMode(NeutralModeValue.Brake);

    if(place == Place.FrontRight || place == Place.BackRight) {
      driveTalon.setInverted(true); // Clockwise_Positive
    }

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
    cancoder.getConfigurator().apply(encoderConfig);

    // ----------------------------------------------------------
    // Inputs
    // ----------------------------------------------------------
    drivePosition = driveTalon.getPosition();
    driveRotorPosition = driveTalon.getRotorPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    cancoderAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, drivePosition, driveRotorPosition, turnPosition); // Required for odometry, use faster rate
    BaseStatusSignal
      .setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        cancoderAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent
      );
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal
      .refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveRotorPosition,
        cancoderAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent
      );

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] { driveCurrent.getValueAsDouble() };
    inputs.driveRotorPosition = driveRotorPosition.getValueAsDouble();
    inputs.targetDriveVelocityMetersPerSec = targetVelocityMetersPerSeconds;
    
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] { turnCurrent.getValueAsDouble() };
    inputs.targetSteerPositionRad = targetSteerPositionRad;

    inputs.cancoderAbsolutePosition = Rotation2d.fromRotations(cancoderAbsolutePosition.getValueAsDouble());
  }

  // ----------------------------------------------------------
  // Outputs
  // ----------------------------------------------------------
  @Override
  public void setDriveVoltage(double volts) { driveTalon.setControl(new VoltageOut(volts)); }

  @Override
  public void setTurnVoltage(double volts) { turnTalon.setControl(new VoltageOut(volts)); }

  @Override
  public void setDriveDutyCycle(double speed) { driveTalon.setControl(new DutyCycleOut(speed)); }

  @Override
  public void setTurnDutyCycle(double speed) { turnTalon.setControl(new DutyCycleOut(speed)); }

  @Override
    public void setTargetTurnPosition(double targetSteerPositionRad) {
      /* Start at position 0, enable FOC, no feed forward, use slot 0 */
      PositionVoltage voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
      turnTalon.setControl(voltagePosition.withPosition(targetSteerPositionRad));
      this.targetSteerPositionRad = targetSteerPositionRad;
    }

    @Override
    public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
      /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
      VelocityVoltage voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
      driveTalon.setControl(voltageVelocity.withVelocity(targetDriveVelocityMetersPerSec));
      this.targetVelocityMetersPerSeconds = targetDriveVelocityMetersPerSec;
    }

}
