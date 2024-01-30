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

    public Rotation2d targetAngle = new Rotation2d(); // Setpoint for the module angle
    public double targetVelocity = 0; // Setpoint for the velocity in meters per/sec
    private double lastPositionMeters = 0.0; // Used for delta calculation

    public SwerveModule(final ModuleIO io, final Place place) {
        this.io = io;
        this.place = place;
        this.turnPID.enableContinuousInput(-180, 180);
    }

    /**
     * Position of the motor rotor in rotations. This position is only affected by the RotorOffset config.
     * 
     * @return Position of the drive motor rotor in rotations.
     */
    public double getDriveRotorPosition() { return this.inputs.driveRotorPosition; }

    /**
     * Position of the device in mechanism rotations. Converts device rotations to radians and then applies the drive gear ratio to get wheel radians
     * 
     * @return drive position of the module in radians
     */
    public double getDrivePosition() { return this.inputs.drivePositionRad; }

    /**
     * Should be equivalent to getDistanceMeters()
     * 
     * @return current drive position of the module in meters.
     */
    public double getDrivePositionMeters() { return getDrivePosition() * Constants.Drivetrain.wheelRadius; }

    /**
     * Position of the motor rotor in rotation units. This position is only affected by the RotorOffset config.
     * 
     * @return
     */
    public double getDistanceMeters() {
        return Constants.Drivetrain.driveGearMotorToWheel
            .forward(
                // Constants.Drivetrain.motorEncoderToRotations.forward(this.drive.getRotorPosition().getValue())
                Constants.Drivetrain.motorEncoderToRotations.forward(this.inputs.driveRotorPosition)
            );
    }

    /**
     * Starts with velocity of the motor in rotations per second. 
     * Converts motor rotations to radians and then applies the drive gear ratio to get wheel radians per/sec
     * Finally, computes meters per/sec by applying the robot's wheel radius.
     * 
     * @return drive velocity in meters per/sec
     */
    public double getDriveVelocity() {
        return (this.inputs.driveVelocityRadPerSec / (2 * Constants.Drivetrain.pi)) * Constants.Drivetrain.wheelRadius;
    }

    /**
     * Position of the turn motor in rotations. 
     * Converts motor rotations to radians and applies the turn gear ratio.
     * 
     * @return position of the turn motor as a Rotation2d
     */
    public Rotation2d getTurnPosition() { return this.inputs.turnPosition; }

    /**
     * Velocity of the turn motor in mechanism rotations per second. 
     * Converts motor rotations to radians and then applies the turn gear ratio to get wheel radians per/sec
     * 
     * @return turn motor velocity in radians per/sec
     */
    public double getTurnVelocity() { return this.inputs.turnVelocityRadPerSec; }

    /**
     * Starts with the Absolute Position of the cancoder in rotations. Min Value: -0.5 Max Value: 0.999755859375 converted to radians and then creates a Rotation2d object. Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble())
     * 
     * @return Absolute Position of the cancoder as Rotation2d
     */
    public Rotation2d getCancoderAbsolutePosition() { return this.inputs.cancoderAbsolutePosition; }

    
    /**
     * 
     * @param state - required speed in meters per/sec and the angle in radians
     */
    public void applyState(final SwerveModuleState state) {
        this.targetVelocity = state.speedMetersPerSecond;
        this.targetAngle = state.angle.unaryMinus();
    }

    public void stop() {
        this.io.setTurnVoltage(0.0);
        this.io.setDriveVoltage(0.0);
    }

    public SwerveModulePosition updateModulePosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getCancoderAbsolutePosition());
    }

    /** Returns the module position delta since the last call to this method. */
    public SwerveModulePosition getPositionDelta() {
        var delta = new SwerveModulePosition(getDrivePositionMeters() - lastPositionMeters, getTurnPosition());
        lastPositionMeters = getDrivePositionMeters();
        return delta;
    }

    // ----------------------------------------------------------
    // Process Logic
    // ----------------------------------------------------------
    void update() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(this.place.index), inputs);

        double currentAngle = this.updateModulePosition().angle.getDegrees();

        SmartDashboard.putNumber(this.place.name() + " Angle", currentAngle);
        SmartDashboard.putNumber(this.place.name() + " Angle Target", this.targetAngle.getDegrees());

        // 9. WHEEL DIRECTION OPTIMIZATION
        this.backwards = Constants.Drivetrain.Flags.wheelOptimization
            && Constants.angleDistance(this.targetAngle.getDegrees(), currentAngle) > 90;

        final double targetAngle = this.backwards
            ? Constants.angleNorm(this.targetAngle.getDegrees() + 180)
            : this.targetAngle.getDegrees();

        // SmartDashboard.putNumber(this.place.name() + " Angle Target Optimized", targetAngle);
        SmartDashboard
            .putNumber(this.place.name() + " Angle Error", Constants.angleDistance(targetAngle, currentAngle));

        // 10. APPLY POWER    

        // Calculate power required to reach the setpoint
        final double turn = this.turnPID.calculate(currentAngle, targetAngle);

        // Restrict the turn power and reverse the direction
        final double turnPower = MathUtil.clamp(-turn, -90, 90); // MAY NEED TO SWITCH turn POSITIVE
        SmartDashboard.putNumber(this.place.name() + " turnPower", turnPower);

        // Calculate the dutyCycle (-1 to 1) taking account of the turn motor gear ratio
        final double turnDutyCycle = turnPower / Constants.Drivetrain.azimuthGearRatio;
        SmartDashboard.putNumber(this.place.name() + " Turn DutyCycle", turnDutyCycle);
        this.io.setTurnDutyCycle(turnDutyCycle);

        final double ffw = Constants.Drivetrain.driveFFW.calculate(this.targetVelocity);
        final double output = Constants.Drivetrain.drivePID.calculate(getDriveVelocity(), this.targetVelocity);
        final double driveDutyCycle = MathUtil.clamp(ffw + output, -1, 1);
        SmartDashboard.putNumber(this.place.name() + " Drive Output", output);
        SmartDashboard.putNumber(this.place.name() + " Drive FFW", ffw);
        SmartDashboard.putNumber(this.place.name() + " Drive DutyCycle", driveDutyCycle);
        this.io.setDriveDutyCycle(this.backwards ? -driveDutyCycle : driveDutyCycle);

    }

}
