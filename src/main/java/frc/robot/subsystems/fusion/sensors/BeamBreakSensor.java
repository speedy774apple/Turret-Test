package frc.robot.subsystems.fusion.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

/**
 * Beam break sensor interface for detecting game pieces in the intake.
 * 
 * This sensor uses a photoelectric beam break sensor (typically connected to a
 * digital input) to detect when an object is present in the intake mechanism.
 * 
 * Update rate: Polled at fusion loop rate (~50 Hz) to avoid excessive CPU
 * usage.
 * Note: While the sensor hardware can respond very quickly, we poll it at the
 * fusion loop rate rather than continuously. This provides adequate
 * responsiveness
 * (20ms latency) while keeping CPU usage reasonable. For applications requiring
 * faster detection, consider using digital input interrupts, but note that most
 * FRC control loops run at 20-50 Hz anyway.
 * 
 * Accuracy: Very high (near-instantaneous hardware response, ~20ms polling
 * latency)
 */
public class BeamBreakSensor {
	private final DigitalInput sensor;
	private final int channel;
	private boolean lastState = false;
	private double lastStateChangeTime = 0.0;
	private final double debounceTime;

	/**
	 * Creates a new beam break sensor.
	 * 
	 * @param channel
	 *            Digital I/O channel for the sensor
	 * @param debounceTime
	 *            Minimum time between state changes (seconds)
	 */
	public BeamBreakSensor(int channel, double debounceTime) {
		this.channel = channel;
		this.debounceTime = debounceTime;
		this.sensor = new DigitalInput(channel);
		this.lastStateChangeTime = Timer.getFPGATimestamp();
	}

	/**
	 * Reads the current state of the beam break sensor.
	 * 
	 * @return true if beam is broken (object detected), false otherwise
	 */
	public boolean isBeamBroken() {
		return !sensor.get(); // Typically sensors are inverted (false = broken)
	}

	/**
	 * Checks if a game piece is currently detected (with debouncing).
	 * 
	 * @return true if object is detected
	 */
	public boolean hasGamePiece() {
		boolean currentState = isBeamBroken();
		double currentTime = Timer.getFPGATimestamp();

		// Debounce: only accept state changes after debounce time
		if (currentState != lastState) {
			if (currentTime - lastStateChangeTime >= debounceTime) {
				lastState = currentState;
				lastStateChangeTime = currentTime;
			}
		}

		return lastState;
	}

	/**
	 * Gets the timestamp of the last state change.
	 * 
	 * @return Timestamp in seconds (FPGA time)
	 */
	public double getLastStateChangeTime() {
		return lastStateChangeTime;
	}

	/**
	 * Checks if this is a new detection (transition from no object to object).
	 * 
	 * @return true if object was just detected
	 */
	public boolean isNewDetection() {
		boolean hasPiece = hasGamePiece();
		boolean wasJustDetected = hasPiece && (Timer.getFPGATimestamp() - lastStateChangeTime < 0.1);
		return wasJustDetected && lastState;
	}
}
