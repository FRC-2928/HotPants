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

    /**
     * Position of the motor rotor in rotations. 
     * This position is only affected by the RotorOffset config.
     * 
     * @return Position of the drive motor rotor in rotations.
     */
    public double getDriveRotorPosition() {
        return this.inputs.driveRotorPosition;
    }

    /**
     * Position of the device in mechanism rotations.
     * Converts device rotations to radians and then 
     * applies the drive gear ratio to get wheel radians
     * 
     * @return drive position of the module in radians
     */
    public double getDrivePosition() {
        return this.inputs.drivePositionRad;
    }

    /** 
     * Should be equivalent to getDistanceMeters()
     * 
     * @return current drive position of the module in meters. 
    */
    public double getDrivePositionMeters() {
        return getDrivePosition() * Constants.Drivetrain.wheelRadius;
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
     * Velocity of the motor in mechanism rotations per second.
     * Converts motor rotations to radians and then applies 
     * the drive gear ratio to get wheel radians per/sec
     * 
     * @return drive velocity in radians per/sec
     */
    public double getDriveVelocity() {
        return this.inputs.driveVelocityRadPerSec;
    }

    /**
     * Position of the turn motor in mechanism rotations.
     * Converts motor rotations to radians and applies
     * the turn gear ratio, then converts to Rotation2d
     * 
     * @return position of the turn motor as a Rotation2d
     */
    public Rotation2d getTurnPosition() {
        return this.inputs.turnPosition;
    }

    /**
     * Velocity of the turn motor in mechanism rotations per second.
     * Converts motor rotations to radians and then applies 
     * the turn gear ratio to get wheel radians per/sec
     * 
     * @return turn motor velocity in radians per/sec
     */
    public double getTurnVelocity() {
        return this.inputs.turnVelocityRadPerSec;
    }

    /**
     * Starts with the Absolute Position of the cancoder in rotations.
     * Min Value: -0.5  Max Value: 0.999755859375 converted to radians 
     * and then creates a Rotation2d object.
     * Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble())
     * 
     * @return Absolute Position of the cancoder as Rotation2d
     */
    public Rotation2d getCancoderAbsolutePosition() {
        return this.inputs.turnAbsolutePosition;
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

    public SwerveModulePosition updateModulePosition() {
        return new SwerveModulePosition(getDrivePositionMeters(),getCancoderAbsolutePosition());
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
        // io.setTurnDutyCycle(Constants.Drivetrain.azimuthGearMotorToWheel.forward(MathUtil.clamp(-turn, -90, 90)));
        io.setTurnVoltage(Constants.Drivetrain.azimuthGearMotorToWheel.forward(MathUtil.clamp(-turn, -90, 90)));

        // this.drive.set(this.backwards ? -this.targetVelocity : this.targetVelocity);
        // io.setDriveDutyCycle(this.backwards ? -this.targetVelocity : this.targetVelocity);
        io.setDriveVoltage(this.backwards ? -this.targetVelocity : this.targetVelocity);
    }

    // public void turnControl() {
    //     // Run closed loop turn control
    //     if (angleSetpoint != null) {
    //         io.setTurnVoltage(
    //             turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));
    
    //         // Run closed loop drive control
    //         // Only allowed if closed loop turn control is running
    //         if (speedSetpoint != null) {
    //         // Scale velocity based on turn error
    //         //
    //         // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
    //         // towards the setpoint, its velocity should increase. This is achieved by
    //         // taking the component of the velocity in the direction of the setpoint.
    //         double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());
    
    //         // Run drive controller
    //         double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;
    //         io.setDriveVoltage(
    //             driveFeedforward.calculate(velocityRadPerSec)
    //                 + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
    //         }
    //     }
    // }

    // /** Runs the module with the specified setpoint state. Returns the optimized state. */
    // public SwerveModuleState runSetpoint(SwerveModuleState state) {
    //     // Optimize state based on current angle
    //     // Controllers run in "periodic" when the setpoint is not null
    //     var optimizedState = SwerveModuleState.optimize(state, getAngle());

    //     // Update setpoints, controllers run in "periodic"
    //     angleSetpoint = optimizedState.angle;
    //     speedSetpoint = optimizedState.speedMetersPerSecond;

    //     return optimizedState;
    // }
}
