// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterLimelight extends Command {
  /** Creates a new centerLimelight. */
  private Angle offsetX;
  private Angle offsetY;
  private double xSpeed;
  private double xSpeedPid;
  private double ySpeed;
  private double ySpeedPid;
  private PIDController centerPID;
  
  public CenterLimelight() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(Robot.cont.drivetrain);
    this.offsetX = Units.Rotations.of(0);
    this.offsetY = Units.Rotations.of(0);
    this.centerPID = Constants.Drivetrain.Auto.centerLimelight.createController();
  }

  public CenterLimelight(Angle offsetX, Angle offsetY) {
    this.addRequirements(Robot.cont.drivetrain);
    this.offsetX = offsetX;
    this.offsetY = offsetY;      
    this.centerPID = Constants.Drivetrain.Auto.centerLimelight.createController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  @Override
  public void execute() {
    xSpeed = (Robot.cont.drivetrain.limelight.getBotPose3d_TargetSpace().getX());
    ySpeed = (Robot.cont.drivetrain.limelight.getBotPose3d_TargetSpace().getY());
    xSpeedPid = centerPID.calculate(xSpeed,0);

    if(Robot.cont.drivetrain.limelight.hasValidTargets()){
      Robot.cont.drivetrain
          .control(
            Robot.cont.drivetrain
              .rod(
                new ChassisSpeeds(
                  0,
                  xSpeedPid,
                  0
                )
              )
      );
    }
      Logger.recordOutput("Drivetrain/Auto/XSpeed", xSpeed);
      Logger.recordOutput("Drivetrain/Auto/YSpeed",  ySpeed);
      Logger.recordOutput("Drivetrain/Auto/Center Is Finished", false);
      Logger.recordOutput("Drivetrain/Auto/CenterPid", xSpeedPid);
      Logger.recordOutput("Drivetrain/Auto/limelightHasValidTargets", Robot.cont.drivetrain.limelight.hasValidTargets());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      Robot.cont.drivetrain.halt();
      Logger.recordOutput("Drivetrain/Auto/Center Is Finished", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false/*(Math.abs(Robot.cont.drivetrain.limelight.getBotPose3d_TargetSpace().getX())) < 0.01 || !Robot.cont.drivetrain.limelight.hasValidTargets()*/;
  }
}
