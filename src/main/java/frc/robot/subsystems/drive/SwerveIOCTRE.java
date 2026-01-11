package frc.robot.subsystems.drive;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

public class SwerveIOCTRE extends SwerveDrivetrain implements SwerveIO {

    HashMap<String, BaseStatusSignal> frontLeftSignals = new HashMap<>();
    HashMap<String, BaseStatusSignal> frontRightSignals = new HashMap<>();
    HashMap<String, BaseStatusSignal> backLeftSignals = new HashMap<>();
    HashMap<String, BaseStatusSignal> backRightSignals = new HashMap<>();

    public Map<Integer, HashMap<String, BaseStatusSignal>> signalsMap = new HashMap<>();

    public SwerveIOCTRE(
            SwerveDrivetrainConstants constants,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
                    moduleConstants) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, moduleConstants);
        this.resetRotation(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? Rotation2d.kZero : Rotation2d.k180deg);

        signalsMap.put(0, frontLeftSignals);
        signalsMap.put(1, frontRightSignals);
        signalsMap.put(2, backLeftSignals);
        signalsMap.put(3, backRightSignals);

        for (int i = 0; i < 4; i++) {
            var driveMotor = this.getModule(i).getDriveMotor();
            var steerMotor = this.getModule(i).getSteerMotor();

            var moduleMap = signalsMap.get(i);

            moduleMap.put("driveSupplyCurrentAmps", driveMotor.getSupplyCurrent());
            moduleMap.put("driveStatorCurrentAmps", driveMotor.getStatorCurrent());
            moduleMap.put("driveAppliedVolts", driveMotor.getMotorVoltage());
            moduleMap.put("driveTemperature", driveMotor.getDeviceTemp());

            moduleMap.put("steerSupplyCurrentAmps", steerMotor.getSupplyCurrent());
            moduleMap.put("steerStatorCurrentAmps", steerMotor.getStatorCurrent());
            moduleMap.put("steerAppliedVolts", steerMotor.getMotorVoltage());
            moduleMap.put("steerTemperature", steerMotor.getDeviceTemp());
        }

    }

    @SuppressWarnings("unchecked")
    @Override
    public void registerTelemetryFunction(SwerveIOInputs inputs) {
        this.registerTelemetry(state -> {
            SwerveDriveState modifiedState = (SwerveDriveState) state;
            modifiedState.Speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    ((SwerveDriveState) state).Speeds, ((SwerveDriveState) state).Pose.getRotation());
            inputs.logState(modifiedState);
        });
    }

    @Override
    public void updateSwerveInputs(SwerveIOInputs inputs) {
        var state = this.getStateCopy();
        state.Speeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
        inputs.logState(state);
    }

    @Override
    public void setSwerveState(SwerveRequest request) {
        this.setControl(request);
    }

    @Override
    public void resetRotation() {
        this.resetRotation(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? Rotation2d.kZero : Rotation2d.k180deg);
    }

    @Override
    public void resetToParamaterizedRotation(Rotation2d rotation2d) {
        this.resetRotation(rotation2d);
    }

    @Override
    public void resetRobotTranslation(Translation2d translation2d) {
        this.resetTranslation(translation2d);
    }

    @Override
    public void updateSimState() {
        //this.updateSimState(Robot.defaultPeriod, 13;)
        this.updateSimState(0.02, 13.00);
    }

    public void updateModuleInputs(ModuleIOInputs... inputs) {
        for (int i = 0; i < 4; i++) {
            var moduleMap = signalsMap.get(i);

            inputs[i].driveSupplyCurrentAmps =
                    moduleMap.get("driveSupplyCurrentAmps").getValueAsDouble();
            inputs[i].driveStatorCurrentAmps =
                    moduleMap.get("driveStatorCurrentAmps").getValueAsDouble();
            inputs[i].driveAppliedVolts = moduleMap.get("driveAppliedVolts").getValueAsDouble();
            inputs[i].driveTemperature = moduleMap.get("driveTemperature").getValueAsDouble();

            inputs[i].steerSupplyCurrentAmps =
                    moduleMap.get("steerSupplyCurrentAmps").getValueAsDouble();
            inputs[i].steerStatorCurrentAmps =
                    moduleMap.get("steerStatorCurrentAmps").getValueAsDouble();
            inputs[i].steerAppliedVolts = moduleMap.get("steerAppliedVolts").getValueAsDouble();
            inputs[i].steerTemperature = moduleMap.get("steerTemperature").getValueAsDouble();
        }
    }

    @Override
    public void refreshData() {
        for (int i = 0; i < 4; i++) {
            var moduleMap = signalsMap.get(i);
            BaseStatusSignal.refreshAll(moduleMap.values().toArray(new BaseStatusSignal[] {}));
        }
    }

    @Override
    public void runCharacterization(final double volts) {
		for(int i = 0; i < 4; i++) {
			getModule(i).getSteerMotor().setControl(new PositionVoltage(Units.Degrees.of(0)));
            getModule(i).getDriveMotor().setControl(new VoltageOut(volts).withEnableFOC(
			Robot.cont.operatorOI.foc.getAsBoolean() || DriverStation.isAutonomous()));
		}
		
	}

}
