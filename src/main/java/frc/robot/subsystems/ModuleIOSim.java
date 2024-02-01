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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim driveSim = new DCMotorSim(DCMotor.getFalcon500(1), 6.75, 0.025);
  private DCMotorSim turnSim = new DCMotorSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004);

  // private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d();
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  /**
     * PID Controller for drive motor. Will do our target drive velocity to voltage calculation
     */
    private final PIDController driveFeedback = new PIDController(10, 0, 0);
    /**
     * PID Controller for steer motor. Will do our target angular position to voltage calculation
     */
    private final PIDController steerFeedback = new PIDController(10, 0, 0);

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
    inputs.driveRotorPosition = driveSim.getAngularPositionRotations();

    inputs.cancoderAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  // Voltage is reversed since turn motors isInverted is false
  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(-volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  /** Run the drive motor at the specified duty cycle (-1 to 1). */
  public void setDriveDutyCycle(double speed) {
    double volts = 12.0 * speed;
    setDriveVoltage(volts);
  }

  /** Run the drive motor at the specified duty cycle (-1 to 1). */
  public void setTurnDutyCycle(double speed) {
    double volts = 12.0 * speed;
    setTurnVoltage(volts);
  }

  @Override
    public void setTargetDriveVelocity(double targetDriveVelocityMetersPerSec) {
        driveFeedback.setSetpoint(
                targetDriveVelocityMetersPerSec); // Setting the target point for the PID controller to calculate the
        // voltage to
    }

    @Override
    public void setTargetTurnPosition(double targetSteerPositionRad) {
        steerFeedback.setSetpoint(
                targetSteerPositionRad); // Setting the target point for the PID controller to calculate the voltage to
    }
}
