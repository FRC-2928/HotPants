// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterLimelight extends Command {
  /** Creates a new centerLimelight. */
  private Distance offsetX;
  private Distance offsetY;
  private double xSpeed;
  private double xSpeedPid;
  private double ySpeed;
  private double ySpeedPid;
  private double thetaSpeed;
  private double thetaPid;
  private PIDController centerPID;
  private PIDController centerRotaionPid;
  private final Distance halfRobotWidth = Units.Inches.of(16.5);
  private Pose2d robotPoseTagspace;
  public static final Pose2d tag8 = new Pose2d(13.474,4.745,new Rotation2d(Math.PI/3));
  private int tag;
  public CenterLimelight() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(Robot.cont.drivetrain);
    this.offsetX = Units.Meters.of(0);
    this.offsetY = Units.Meters.of(0);
    this.centerPID = Constants.Drivetrain.Auto.centerLimelight.createController();
    this.centerRotaionPid = Constants.Drivetrain.Auto.centerTheta.createController();
    this.tag = Robot.cont.drivetrain.limelight.getTargetAprilTagID();
  }
  

  public CenterLimelight(Distance offsetX, Distance offsetY) {
    this.addRequirements(Robot.cont.drivetrain);
    this.offsetX = offsetX.plus(halfRobotWidth);
    this.offsetY = offsetY;      
    this.centerPID = Constants.Drivetrain.Auto.centerLimelight.createController();
    this.centerRotaionPid = Constants.Drivetrain.Auto.centerTheta.createController();
    this.tag = Robot.cont.drivetrain.limelight.getTargetAprilTagID();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.cont.drivetrain.setAngle(Robot.cont.drivetrain.limelight.getBluePose2d().getRotation().getMeasure());
  }
  @Override
  public void execute() {
    if(this.tag == Robot.cont.drivetrain.limelight.getTargetAprilTagID()){
      robotPoseTagspace = tag8.relativeTo(Robot.cont.drivetrain.limelight.getPoseMegatag2().pose);
    }
    xSpeed = robotPoseTagspace.getX();
    ySpeed = robotPoseTagspace.getY();
    thetaSpeed = robotPoseTagspace.getRotation().getRadians();
    xSpeedPid = centerPID.calculate(xSpeed,offsetX.in(Units.Meters));
    ySpeedPid = centerPID.calculate(ySpeed,offsetY.in(Units.Meters));
    thetaPid = centerRotaionPid.calculate(thetaSpeed,0);
    if(Robot.cont.drivetrain.limelight.hasValidTargets()){
      Robot.cont.drivetrain
          .control(
            Robot.cont.drivetrain
              .rod(
                new ChassisSpeeds(
                  0,
                  0,
                  thetaPid
                )
              )
      );
    }
      Logger.recordOutput("Drivetrain/Auto/XSpeed", xSpeed);
      Logger.recordOutput("Drivetrain/Auto/YSpeed",  ySpeed);
      Logger.recordOutput("Drivetrain/Auto/Center Is Finished", false);
      Logger.recordOutput("Drivetrain/Auto/CenterPid", xSpeedPid);
      Logger.recordOutput("Drivetrain/Auto/limelightHasValidTargets", Robot.cont.drivetrain.limelight.hasValidTargets());
      Logger.recordOutput("Drivetrain/Auto/Theta", Robot.cont.drivetrain.limelight.getBotPose3d_TargetSpace().getRotation().getAngle());
      Logger.recordOutput("Drivetrain/Auto/robotPoseTagSpace", robotPoseTagspace);
      Logger.recordOutput("Drivetrain/Auto/thetaSpeed", thetaSpeed);
      Logger.recordOutput("Drivetrain/Auto/thetaPid", thetaPid);
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
    return (!Robot.cont.drivetrain.limelight.hasValidTargets());
  }
}
