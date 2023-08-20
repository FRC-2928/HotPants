package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    public final Drivetrain drivetrain;
    public final DriverOI oi;

    public boolean fod = true;
    public boolean absoluteRotation = false;

    public Rotation2d absoluteTarget = new Rotation2d();
    public double absoluteTargetMagnitude = 0;
    private final PIDController absoluteController = Constants.Drivetrain.absoluteRotationPID.createController();

    public Drive(final Drivetrain drivetrain, final DriverOI oi) {
        this.drivetrain = drivetrain;
        this.oi = oi;

        this.addRequirements(drivetrain);

        this.absoluteController.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        final double mul = MathUtil.interpolate(1, 0.5, this.oi.slow.get());

        final double axial = MathUtil.applyDeadband(this.oi.moveAxial.get(), 0.1);
        final double lateral = MathUtil.applyDeadband(this.oi.moveLateral.get(), 0.1);
        final double theta;

        if(this.absoluteRotation) { // todo debug graphs and make this work
            final double rotX = this.oi.moveRotationX.get();
            final double rotY = this.oi.moveRotationY.get();

            this.absoluteTargetMagnitude = Math.sqrt(rotX * rotX + rotY * rotY);
            if(this.absoluteTargetMagnitude > 0.25) this.absoluteTarget = Rotation2d.fromRadians(Math.atan2(rotX, -rotY));

            theta = this.absoluteController.calculate(Constants.mod(this.drivetrain.gyro.getRotation2d().unaryMinus().getRotations(), 1) - 0.5, this.absoluteTarget.getRotations());
            SmartDashboard.putNumber("absolute target", this.absoluteTarget.getRotations());
            SmartDashboard.putNumber("absolute current", Constants.mod(this.drivetrain.gyro.getRotation2d().unaryMinus().getRotations(), 1) - 0.5);
        } else theta = MathUtil.applyDeadband(this.oi.moveTheta.get(), 0.25);

        ChassisSpeeds desired = new ChassisSpeeds(
            axial * -Constants.Drivetrain.axialLateralSpeed * mul,
            lateral * Constants.Drivetrain.axialLateralSpeed * mul,
            theta * Math.toRadians(Constants.Drivetrain.thetaSpeed) * mul
        );

        if(this.fod) {
            final ChassisSpeeds fod = this.drivetrain.fod(desired);
            //System.out.printf("fod %s -> %s\n", desired, fod);
            desired = fod;
        }

        this.drivetrain.swerve(this.drivetrain.kinematics.toSwerveModuleStates(desired));
    }
}
