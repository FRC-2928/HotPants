package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.ModuleIO;
import frc.robot.commands.drivetrain.ModuleIOInputsAutoLogged;

public class SwerveModule {
    public static enum Place {
        FrontLeft(0), FrontRight(1), BackLeft(2), BackRight(3);

        private Place(final int index) { this.index = index; }

        public final int index;
    }

    public static class State {
        public final SwerveModuleState[] states;

        public State() { this.states = new SwerveModuleState[4]; }

        public State(final SwerveModuleState[] states) { this.states = states; }

        public static State forward() {
            return new State()
                .set(Place.FrontLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
                .set(Place.FrontRight, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
                .set(Place.BackLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
                .set(Place.BackRight, new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        }

        public static State locked() {
            return new State()
                .set(Place.FrontLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
                .set(Place.FrontRight, new SwerveModuleState(0, Rotation2d.fromDegrees(45)))
                .set(Place.BackLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(-135)))
                .set(Place.BackRight, new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        }

        public SwerveModuleState get(final Place place) { return this.states[place.index]; }

        public State set(final Place place, final SwerveModuleState state) {
            this.states[place.index] = state;
            return this;
        }
    }

    public final Place place;
    public final ModuleIO io;    
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private final PIDController pid = Constants.Drivetrain.swerveAzimuthPID.createController();

    private boolean backwards = false;

    public Rotation2d target = new Rotation2d();
    public double targetVelocity = 0;

    public SwerveModule(final ModuleIO io, final Place place){
        this.io = io;
        this.place = place;
        this.pid.enableContinuousInput(-180, 180);
        setBrakeMode(true);
    }

    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }
    public SwerveModulePosition updateModulePosition() {
        return new SwerveModulePosition(
            Constants.Drivetrain.driveGearMotorToWheel
                .forward(
                    // Constants.Drivetrain.motorEncoderToRotations.forward(this.drive.getRotorPosition().getValue())
                    Constants.Drivetrain.motorEncoderToRotations.forward(this.inputs.drivePositionRotations)
                ),
            // Rotation2d.fromRotations(this.encoder.getAbsolutePosition().getValue())
            this.inputs.turnAbsolutePosition
        );
    }

    public void applyState(final SwerveModuleState state) {
        final double ffw = Constants.Drivetrain.driveFFW.calculate(state.speedMetersPerSecond);

        this.target = state.angle.unaryMinus();
        this.targetVelocity = ffw;
    }

    public void stop() { 
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);
    }

    void update() {
        SmartDashboard.putNumber(this.place.name() + " Angle", this.updateModulePosition().angle.getDegrees());
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(this.place.index), inputs);
        this.backwards = Constants.Drivetrain.Flags.wheelOptimization
            && Constants.angleDistance(this.target.getDegrees(), this.updateModulePosition().angle.getDegrees()) > 90;
        final double target = this.backwards
            ? Constants.angleNorm(this.target.getDegrees() + 180)
            : this.target.getDegrees();

        SmartDashboard.putNumber(this.place.name() + " Target", target);
        SmartDashboard
            .putNumber(
                this.place.name() + " Distance From Target",
                Constants.angleDistance(target, this.updateModulePosition().angle.getDegrees())
            );

        final double turn = this.pid.calculate(this.updateModulePosition().angle.getDegrees(), target);

       // this.azimuth.set(Constants.Drivetrain.azimuthGearMotorToWheel.forward(MathUtil.clamp(-turn, -90, 90)));
        io.setTurnVoltage(Constants.Drivetrain.azimuthGearMotorToWheel.forward(MathUtil.clamp(-turn, -90, 90)));

        // this.drive.set(this.backwards ? -this.targetVelocity : this.targetVelocity);
        io.setDriveVoltage(this.backwards ? -this.targetVelocity : this.targetVelocity);
    }
}
