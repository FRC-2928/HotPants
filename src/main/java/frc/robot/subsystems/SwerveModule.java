package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

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

    private final PIDController turnPID = Constants.Drivetrain.swerveAzimuthPID.createController();

    private boolean backwards = false;

    public Rotation2d targetAngle = new Rotation2d();
    public double targetVelocity = 0;

    public SwerveModule(final ModuleIO io, final Place place){
        this.io = io;
        this.place = place;
        this.turnPID.enableContinuousInput(-180, 180);
    }

    public SwerveModulePosition updateModulePosition() {
        return new SwerveModulePosition(getDistanceMeters(),getAngle());
    }

    /**
     * Position of the motor rotor in rotation units. 
     * This position is only affected by the RotorOffset config.
     * 
     * @return
     */
    public double getDistanceMeters() {
        return 
            Constants.Drivetrain.driveGearMotorToWheel
                .forward(
                    // Constants.Drivetrain.motorEncoderToRotations.forward(this.drive.getRotorPosition().getValue())
                    Constants.Drivetrain.motorEncoderToRotations.forward(this.inputs.driveRotorPosition)
                );
    }

    /**
     * Absolute Position of the device in rotations.
     * 
     * @return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble());
     */
    public Rotation2d getAngle() {
        return  this.inputs.turnAbsolutePosition;
    }

    /**
     * Calculates the feedforward for the drive velocity.
     * 
     * @param state - required speed in meters per/sec and the angle
     */
    public void applyState(final SwerveModuleState state) {
        final double ffw = Constants.Drivetrain.driveFFW.calculate(state.speedMetersPerSecond);
        this.targetVelocity = ffw;

        this.targetAngle = state.angle.unaryMinus();  
    }

    public void stop() { 
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);
    }

    void update() {
        SmartDashboard.putNumber(this.place.name() + " Angle", this.updateModulePosition().angle.getDegrees());
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(this.place.index), inputs);

        // 8. WHEEL DIRECTION OPTIMIZATION
        this.backwards = Constants.Drivetrain.Flags.wheelOptimization
            && Constants.angleDistance(this.targetAngle.getDegrees(), this.updateModulePosition().angle.getDegrees()) > 90;
        final double targetAngle = this.backwards
            ? Constants.angleNorm(this.targetAngle.getDegrees() + 180)
            : this.targetAngle.getDegrees();

        SmartDashboard.putNumber(this.place.name() + " Target Angle", targetAngle);
        SmartDashboard
            .putNumber(
                this.place.name() + " Distance From Target",
                Constants.angleDistance(targetAngle, this.updateModulePosition().angle.getDegrees())
            );

        // 9. APPLY POWER
        final double turn = this.turnPID.calculate(this.updateModulePosition().angle.getDegrees(), targetAngle);
       // this.azimuth.set(Constants.Drivetrain.azimuthGearMotorToWheel.forward(MathUtil.clamp(-turn, -90, 90)));
        io.setTurnVoltage(Constants.Drivetrain.azimuthGearMotorToWheel.forward(MathUtil.clamp(-turn, -90, 90)));

        // this.drive.set(this.backwards ? -this.targetVelocity : this.targetVelocity);
        io.setDriveVoltage(this.backwards ? -this.targetVelocity : this.targetVelocity);
    }
}
