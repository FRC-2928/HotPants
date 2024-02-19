// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LockServo extends SubsystemBase {

  private final Servo m_servo;
  private double m_defaultAngle = 90;
  private double m_servoAngle = m_defaultAngle;
  private double m_minAngle = 0.0;
  private double m_maxAngle = 180.0;

  // Creates a new Servo subsystem
  public LockServo(final int channel) {
    m_servo = new Servo(channel);
    reset();
  }

  public void setDefaultAngle(double angle){
    m_defaultAngle = angle;
    reset();
  }
  // Reset position to resting state
  public void reset() {
    m_servo.setAngle(m_servoAngle);
  }

  /** 
   * Increment servo1 motor position
   * 
   * @param delta Amount to change motor position
   */
  public void incrementServo(double delta) {
    m_servoAngle = saturateLimit(m_servoAngle + delta, m_minAngle, m_maxAngle);
    m_servo.setAngle(m_servoAngle);
  }

  /** 
   * Set the min and max angle range for this servo
   * 
   * @param minAngle Minimum servo angle position
   * @param maxAngle Maximum servo angle position
   */
  public void setAngleRange(int minAngle, int maxAngle) {
    m_minAngle = minAngle;
    m_maxAngle = maxAngle;
  }

  // Get the current servo angle
  public double getCurrentAngle() {
    return m_servoAngle;
  }

  // Limit motor range to avoid moving beyond safe ranges
  public double saturateLimit(double val, double min_angle, double max_angle) {
    double outval = val;
    if(val > max_angle) {
      outval =  max_angle;
    } else if (val < min_angle) {
      outval = min_angle;
    }
    return outval;
  }

  /**
   * Is the servo at its max allowable angle
   * 
   * @return true if at max angle, false if not
   */
  public boolean atMaxAngle() {
    return getCurrentAngle() >= m_maxAngle;
  }

  /**
   * Is the servo at its min allowable angle
   * 
   * @return true if at min angle, false if not
   */
  public boolean atMinAngle() {
    return getCurrentAngle() <= m_minAngle;
  }

}
