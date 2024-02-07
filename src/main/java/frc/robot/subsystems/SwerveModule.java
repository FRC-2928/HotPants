package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    public final String moduleName;
    public final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private final PIDController turnPID = Constants.Drivetrain.swerveAzimuthPID.createController();
    private final SimpleMotorFeedforward driveFFW = Constants.Drivetrain.driveFFW;
    private final PIDController drivePID = Constants.Drivetrain.drivePID;

    private boolean backwards = false;

    public Rotation2d targetAngle = new Rotation2d(); // Setpoint for the module angle
    public double targetVelocity = 0; // Setpoint for the velocity in meters per/sec
    public SwerveModuleState swerveModuleState = new SwerveModuleState();
    private Rotation2d lastAngle = new Rotation2d();

    public SwerveModule(final ModuleIO io, final Place place) {
        this.io = io;
        this.place = place;
        this.turnPID.enableContinuousInput(-180, 180);
        this.moduleName = this.place.name();
    }

    /**
     * Position of the motor rotor in rotations. This position is only affected by the RotorOffset config.
     * 
     * @return Position of the drive motor rotor in rotations.
     */
    public double getDriveRotorPosition() { return this.inputs.driveRotorPosition; }

    /**
     * Position of the device in motor rotations.  Converts device rotations to 
     * radians and then applies the drive gear ratio to get wheel radians
     * 
     * @return drive position of the module in radians
     */
    public double getDrivePosition() { return this.inputs.drivePositionRad; }

    /**
     * 
     * @return current drive position of the module in meters.
     */
    public double getDrivePositionMeters() { return getDrivePosition() * Constants.Drivetrain.wheelRadius; }

    /**
     * Starts with velocity of the motor in rotations per second. 
     * Converts motor rotations to radians and then applies the drive gear ratio to get wheel radians per/sec
     * Finally, computes meters per/sec by applying the robot's wheel radius.
     * 
     * @return drive velocity in meters per/sec
     */
    public double getDriveVelocity() {
        return (this.inputs.driveVelocityRadPerSec / (2 * Math.PI)) * Constants.Drivetrain.wheelRadius;
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
     * Starts with the Absolute Position of the cancoder in rotations. using 
     * cancoder.getAbsolutePosition() Min Value: -0.5 Max Value: 0.999755859375 
     * converted to radians and then creates a Rotation2d object. 
     * 
     * @return Absolute Position of the cancoder as Rotation2d
     */
    public Rotation2d getCancoderAbsolutePosition() { return this.inputs.cancoderAbsolutePosition; }

    /**
     * 
     * @return applied speed in meters per/sec and the angle in radians
     */
    public SwerveModuleState getModuleState() {
        return this.swerveModuleState;
    }

    /**
     * 
     * @param state - required speed in meters per/sec and the angle in radians
     */
    public void applyState(final SwerveModuleState state) {
        this.swerveModuleState.speedMetersPerSecond = state.speedMetersPerSecond;
        this.swerveModuleState.angle = state.angle.unaryMinus();
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

    // ----------------------------------------------------------
    // Process Logic
    // ----------------------------------------------------------
    private SwerveModuleState optimizeWheelDirection(Rotation2d currentAngleRotation) {
        this.backwards = Constants.Drivetrain.Flags.wheelOptimization
            && Constants.angleDistance(this.targetAngle.getDegrees(), currentAngleRotation.getDegrees()) > 90;

        final double targetAngle = this.backwards
            ? Constants.angleNorm(this.targetAngle.getDegrees() + 180)
            : this.targetAngle.getDegrees();
    
        final double targetVelocity = this.backwards  
            ? -this.targetVelocity : this.targetVelocity;

        SmartDashboard.putNumber(this.place.name() + " Angle Error", Constants.angleDistance(targetAngle, currentAngleRotation.getDegrees()));
        Rotation2d angle = Rotation2d.fromDegrees(targetAngle);
        SwerveModuleState optimizedState = new SwerveModuleState(targetVelocity, angle);
        return optimizedState;
    }

    private SwerveModuleState optimize(Rotation2d currentAngleRotation) {
        SwerveModuleState desiredState = new SwerveModuleState(this.targetVelocity, this.targetAngle);
        if (Constants.Drivetrain.Flags.wheelOptimization) {
            return SwerveModuleState.optimize(desiredState, currentAngleRotation);
        } else {
            return desiredState;
        }
    }

    /**
     * Prevent rotating module if speed is less then 1%. Prevents Jittering.
     * 
     * @param state desired state
     * @return 
     */
    private SwerveModuleState preventJittering(SwerveModuleState state) {
        Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= (Constants.Drivetrain.maxVelocityMetersPerSec * 0.01)) ? this.lastAngle : state.angle; 
        this.lastAngle = angle;
        SwerveModuleState newState = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
        return newState;
    }

    /**
     * Computes the voltage output required and sends it to the turn motor
     * 
     * @param currentAngle - in degrees
     * @param targetAngle - in degrees
     */
    private void applyTurnVolts(double currentAngle, double desiredAngle) {

        // Calculate turn power required to reach the setpoint
        final double turn = this.turnPID.calculate(currentAngle, desiredAngle);

        // Restrict the turn power and reverse the direction
        final double turnVolts = MathUtil.clamp(-turn, -10, 10);

        // inputs.turnAppliedVolts will track the applied voltage
        this.io.setTurnVoltage(turnVolts);
    }

    private void applyDriveVolts(SwerveModuleState desiredState) {
        // Calculate drive power
        final double ffw = this.driveFFW.calculate(desiredState.speedMetersPerSecond);       
        final double output = this.drivePID.calculate(getDriveVelocity(), desiredState.speedMetersPerSecond);
        final double driveVolts = MathUtil.clamp(ffw + output, -10, 10);
        
        // inputs.driveAppliedVolts will track the applied voltage
        this.io.setDriveVoltage(driveVolts);
    }

    void update() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(this.place.index), inputs);

        Rotation2d currentAngleRotation = this.updateModulePosition().angle;
        double currentAngle = currentAngleRotation.getDegrees();
        SmartDashboard.putNumber(this.place.name() + " Angle", currentAngle);
        SmartDashboard.putNumber(this.place.name() + " Angle Target", this.targetAngle.getDegrees());

        // 7. WHEEL DIRECTION OPTIMIZATION
        SwerveModuleState optimizedState = optimizeWheelDirection(currentAngleRotation);
        // SwerveModuleState optimizedState = optimize(currentAngleRotation);

        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        // optimizedState = preventJittering(optimizedState);

        // 8. APPLY POWER          
        applyDriveVolts(optimizedState);
        applyTurnVolts(currentAngle, optimizedState.angle.getDegrees());

        // applyDriveVelocity(optimizedState.speedMetersPerSecond);
        // // applyDriveTorque(optimizedState.speedMetersPerSecond);
        // applyTurnPosition(optimizedState.angle);
    }

    /**
     * Computes the velocity, torque, or position output required and sends it to the motors
     * Uses the closed loop functions of the Phoenix motors
     */
    public void applyDriveVelocity(double requiredSpeedMetersPerSecond) {
        this.io.setTargetDriveVelocity(requiredSpeedMetersPerSecond);
    }

    public void applyDriveTorque(double requiredSpeedMetersPerSecond) {
        this.io.setTargetDriveTorque(requiredSpeedMetersPerSecond);
    }

    // Will use the CANcoder as the feedback device.  See turn motor config.
    public void applyTurnPosition(Rotation2d requiredAngle) {
        this.io.setTargetTurnPosition(requiredAngle.getRadians());
    }
}
