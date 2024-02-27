package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class Diagnostics extends SubsystemBase {
	public final class SystemOverride extends Command {
		public SystemOverride() { this.addRequirements(Robot.cont.drivetrain); }

		@Override
		public void end(final boolean interrupted) { CommandScheduler.getInstance().cancelAll(); }

		@Override
		public InterruptionBehavior getInterruptionBehavior() { return InterruptionBehavior.kCancelIncoming; }
	}

	public final class Release extends Command {
		public Release() { this.addRequirements(Robot.cont.drivetrain); }

		@Override
		public void initialize() {
			for(final SwerveModule module : Robot.cont.drivetrain.modules) {
				final ModuleIOReal io = (ModuleIOReal) module.io;
				io.azimuth.setNeutralMode(NeutralModeValue.Coast);
				io.drive.setNeutralMode(NeutralModeValue.Coast);
			}

			Diagnostics.this.chirp(400, 500);
		}

		@Override
		public void end(final boolean interrupted) {
			for(final SwerveModule module : Robot.cont.drivetrain.modules) {
				final ModuleIOReal io = (ModuleIOReal) module.io;
				io.azimuth.setNeutralMode(NeutralModeValue.Brake);
				io.drive.setNeutralMode(NeutralModeValue.Brake);
			}

			Diagnostics.this.chirp(800, 500);
		}

		@Override
		public boolean runsWhenDisabled() { return true; }
	}

	private static final class Chirp {
		public Chirp(final int freq, final int ms) {
			this.freq = freq;
			this.us = ms * 1000;
		}

		public int freq;
		public long us;
		public long start = 0;
	}

	public final ArrayList<TalonFX> motors = new ArrayList<>();

	private final ArrayList<Chirp> chirps = new ArrayList<>();

	public final DigitalInput releaseInput = new DigitalInput(0);
	public final Trigger release = new Trigger(() -> DriverStation.isDisabled() && !this.releaseInput.get());

	public final DigitalInput lockoutInput = new DigitalInput(1);
	public final Trigger lockout = new Trigger(() -> DriverStation.isTestEnabled() && !this.lockoutInput.get());

	public void chirp(final boolean good) { this.chirps.add(new Chirp(good ? 500 : 125, 500)); }

	public void chirp(final int freq, final int ms) { this.chirps.add(new Chirp(freq, ms)); }

	@Override
	public void periodic() {
		Logger.recordOutput("Diagnostics/Release", this.releaseInput.get());
		Logger.recordOutput("Diagnostics/Lockout", this.lockoutInput.get());

		if(this.chirps.size() > 0) {
			final Chirp chirp = this.chirps.get(0);
			if(chirp.start == 0) chirp.start = Logger.getRealTimestamp();

			for(final TalonFX fx : this.motors)
				fx.setControl(new MusicTone(chirp.freq));

			if(Logger.getRealTimestamp() - chirp.start >= chirp.us) {
				for(final TalonFX fx : this.motors)
					fx.setControl(new MusicTone(0));

				this.chirps.remove(0);
			}
		}
	}
}
