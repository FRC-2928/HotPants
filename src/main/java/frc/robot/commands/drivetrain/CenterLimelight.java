// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

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
  private PIDController centerPIDx;
  private PIDController centerPIDy;
  private PIDController centerRotaionPid;
  private final Distance halfRobotWidth = Units.Inches.of(20);
  private Pose2d robotPoseTagspace;
  public static final Pose2d tag8 = new Pose2d(13.474,4.745,new Rotation2d(Math.PI/3));
  private int tag;
  private Pose3d tagPose;
  private List<Integer> tagsToCheck;
  private final static List<Integer> reefTags = List.of(6,7,8,9,10,11,17,18,19,20,21,22);
    public CenterLimelight() {
      // Use addRequirements() here to declare subsystem dependencies.
      this.addRequirements(Robot.cont.drivetrain);
      this.offsetX = halfRobotWidth;
      this.offsetY = Units.Meters.of(0);
      this.centerPIDx = Constants.Drivetrain.Auto.centerLimelight.createController();
      this.centerPIDy = Constants.Drivetrain.Auto.centerLimelight.createController();
      this.centerRotaionPid = Constants.Drivetrain.Auto.centerTheta.createController();
      this.centerRotaionPid.enableContinuousInput(-Math.PI, Math.PI);
      this.tagsToCheck = new ArrayList<>();
    }
    public static CenterLimelight CenterLimelightLeft(){
      return new CenterLimelight(Units.Feet.of(0),Units.Inches.of(-6.5), reefTags);
    }
    public static CenterLimelight CenterLimelightRight(){
      return new CenterLimelight(Units.Feet.of(0),Units.Inches.of(6.5), reefTags);
    }
    public static CenterLimelight CenterLimelightCenter(){
      return new CenterLimelight(Units.Feet.of(0),Units.Inches.of(0), reefTags);
    }

  public CenterLimelight(Distance offsetX, Distance offsetY, final List<Integer> tagsToCheck) {
    this.addRequirements(Robot.cont.drivetrain);
    this.offsetX = offsetX.plus(halfRobotWidth);
    this.offsetY = offsetY;      
    this.centerPIDx = Constants.Drivetrain.Auto.centerLimelight.createController();
    this.centerPIDy = Constants.Drivetrain.Auto.centerLimelight.createController();
    this.centerRotaionPid = Constants.Drivetrain.Auto.centerTheta.createController();
    this.centerRotaionPid.enableContinuousInput(-Math.PI, Math.PI);
    this.tagsToCheck = tagsToCheck;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double smallst = Double.MAX_VALUE;
    tagPose = Constants.FIELD_LAYOUT.getTagPose(17).get();
    for(int tag : tagsToCheck){
      Pose2d distance = Constants.FIELD_LAYOUT.getTagPose(tag).get().toPose2d().relativeTo(Robot.cont.drivetrain.est.getEstimatedPosition());
      if(Math.hypot(distance.getX(), distance.getY()) < smallst){
          tagPose = Constants.FIELD_LAYOUT.getTagPose(tag).get();
          smallst = Math.hypot(distance.getX(), distance.getY());
      }
    }
    Logger.recordOutput("Drivetrain/Auto/tagpose", tagPose);
    
  }

  @Override
  public void execute() {
    robotPoseTagspace = tagPose.toPose2d().relativeTo(Robot.cont.drivetrain.est.getEstimatedPosition());
    xSpeed = robotPoseTagspace.getX();
    ySpeed = robotPoseTagspace.getY();
    thetaSpeed = robotPoseTagspace.getRotation().getRadians();
    xSpeedPid = -centerPIDx.calculate(xSpeed,offsetX.in(Units.Meters));
    ySpeedPid = -centerPIDy.calculate(ySpeed,offsetY.in(Units.Meters));
    thetaPid = -centerRotaionPid.calculate(thetaSpeed,Math.PI);
    Robot.cont.drivetrain
        .controlRobotOriented(
              new ChassisSpeeds(
                xSpeedPid,
                ySpeedPid,
                thetaPid
              )
    );
    Logger.recordOutput("Drivetrain/Auto/XSpeed", xSpeed);
    Logger.recordOutput("Drivetrain/Auto/YSpeed",  ySpeed);
    Logger.recordOutput("Drivetrain/Auto/Center Is Finished", false);
    Logger.recordOutput("Drivetrain/Auto/XSpeedPid", xSpeedPid);
    Logger.recordOutput("Drivetrain/Auto/YSpeedPid", ySpeedPid);
    Logger.recordOutput("Drivetrain/Auto/limelightHasValidTargets", Robot.cont.drivetrain.limelight.hasValidTargets());
    Logger.recordOutput("Drivetrain/Auto/Theta", Robot.cont.drivetrain.limelight.getBotPose3d_TargetSpace().getRotation().getAngle());
    Logger.recordOutput("Drivetrain/Auto/robotPoseTagSpace", robotPoseTagspace);
    Logger.recordOutput("Drivetrain/Auto/thetaSpeed", thetaSpeed);
    Logger.recordOutput("Drivetrain/Auto/thetaPid", thetaPid);
    Logger.recordOutput("Drivetrain/Auto/estPose", Robot.cont.drivetrain.est.getEstimatedPosition().getRotation());
    Logger.recordOutput("Drivetrain/Auto/offsetX", offsetX);
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
    return (Math.abs(xSpeedPid) < 0.09) && (Math.abs(ySpeedPid) < 0.2) && (Math.abs(thetaPid) < 0.15);
  }
}
