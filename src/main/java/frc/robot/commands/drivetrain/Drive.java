package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    public final Drivetrain drivetrain;
    public final DriverOI oi;

    public Drive(Drivetrain drivetrain, DriverOI oi) {
        this.drivetrain = drivetrain;
        this.oi = oi;

        this.addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double axial = MathUtil.applyDeadband(this.oi.moveAxial.get(), 0.1);
        double lateral = MathUtil.applyDeadband(this.oi.moveLateral.get(), 0.1);
        double theta = MathUtil.applyDeadband(this.oi.moveTheta.get(), 0.25);

        this.drivetrain.swerve(this.drivetrain.kinematics.toSwerveModuleStates(new ChassisSpeeds(
            axial * Constants.Drivetrain.axialSpeed,
            lateral * -Constants.Drivetrain.lateralSpeed,
            theta * Math.toRadians(Constants.Drivetrain.thetaSpeed)
        )));
    }
}
