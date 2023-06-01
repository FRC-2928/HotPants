package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveModulePlace;
import frc.robot.subsystems.Drivetrain.SwerveState;

public class LockWheels extends CommandBase {
    private final Drivetrain drivetrain;

    public LockWheels(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.drivetrain.swerve(
            new SwerveState()
                .set(SwerveModulePlace.FrontLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
                .set(SwerveModulePlace.FrontRight, new SwerveModuleState(0, Rotation2d.fromDegrees(45)))
                .set(SwerveModulePlace.BackLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(-135)))
                .set(SwerveModulePlace.BackRight, new SwerveModuleState(0, Rotation2d.fromDegrees(135)))
        );
    }
}
