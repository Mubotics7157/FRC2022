package frc.auto;

import edu.wpi.first.wpilibj.Timer;

public class Delay extends AutoCommand {

	private double seconds;
	private double start;

	public Delay(double seconds) {
		this.seconds = seconds;
		this.setBlocking(true);
	}

	@Override
	public void start() {
		Timer.delay(seconds);
		start = Timer.getFPGATimestamp();
	}

	@Override
	public boolean isFinished() {
		return Math.abs(Timer.getFPGATimestamp() - start) < seconds;
	}

}