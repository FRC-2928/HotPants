// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;;

public class RunIntake extends Command {
    /** Creates a new RunIntake. */
    private Intake intake;
    public LoggedDashboardNumber speed;

    public RunIntake() {
        intake = new Intake();
        this.addRequirements(intake);
        this.speed = new LoggedDashboardNumber("Tuning/SpeedIntakePercent", 1);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { intake.runIntake(MathUtil.clamp(this.speed.get(), -1, 1)); }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { intake.runIntake(0); }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() { return false; }
}
