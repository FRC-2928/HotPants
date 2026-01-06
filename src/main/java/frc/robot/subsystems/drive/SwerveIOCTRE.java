package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveModule.Place;

import java.util.HashMap;
import java.util.Map;

public class SwerveIOCTRE extends SwerveDrivetrain implements SwerveIO {
    HashMap<String, BaseStatusSignal> signal = new HashMap<>();
    Place place;

    Map<Integer, HashMap<String, BaseStatusSignal>> signalsMap = new HashMap<>();

    public SwerveIOCTRE(
            Place place,
            SwerveDrivetrainConstants constants,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
                    moduleConstants) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, moduleConstants);
        this.resetRotation(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? Rotation2d.kZero : Rotation2d.k180deg);

        signalsMap.put(0, signal);
        
            var driveMotor = this.getModule(place.getPlace()).getDriveMotor();
            var steerMotor = this.getModule(place.getPlace()).getSteerMotor();

            var moduleMap = signalsMap.get(place.getPlace());

            moduleMap.put("driveSupplyCurrentAmps", driveMotor.getSupplyCurrent());
            moduleMap.put("driveStatorCurrentAmps", driveMotor.getStatorCurrent());
            moduleMap.put("driveAppliedVolts", driveMotor.getMotorVoltage());
            moduleMap.put("driveTemperature", driveMotor.getDeviceTemp());

            moduleMap.put("steerSupplyCurrentAmps", steerMotor.getSupplyCurrent());
            moduleMap.put("steerStatorCurrentAmps", steerMotor.getStatorCurrent());
            moduleMap.put("steerAppliedVolts", steerMotor.getMotorVoltage());
            moduleMap.put("steerTemperature", steerMotor.getDeviceTemp());
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

	//TODO: find out what this does
    @Override
    public void updateSimState() {
        this.updateSimState(0.02, 13.00);
    }

    public void updateModuleInputs(ModuleIOInputs inputs) {
            var moduleMap = signalsMap.get(this.place.getPlace());

            inputs.driveSupplyCurrentAmps =
                    moduleMap.get("driveSupplyCurrentAmps").getValueAsDouble();
            inputs.driveStatorCurrentAmps =
                    moduleMap.get("driveStatorCurrentAmps").getValueAsDouble();
            inputs.driveAppliedVolts = moduleMap.get("driveAppliedVolts").getValueAsDouble();
            inputs.driveTemperature = moduleMap.get("driveTemperature").getValueAsDouble();

            inputs.steerSupplyCurrentAmps =
                    moduleMap.get("steerSupplyCurrentAmps").getValueAsDouble();
            inputs.steerStatorCurrentAmps =
                    moduleMap.get("steerStatorCurrentAmps").getValueAsDouble();
            inputs.steerAppliedVolts = moduleMap.get("steerAppliedVolts").getValueAsDouble();
            inputs.steerTemperature = moduleMap.get("steerTemperature").getValueAsDouble();
    }

    @Override
    public void refreshData() {
            var moduleMap = signalsMap.get(place.getPlace());
            BaseStatusSignal.refreshAll(moduleMap.values().toArray(new BaseStatusSignal[] {}));
    }
}
