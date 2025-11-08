package frc.robot.subsystems.fusion.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simulated beam break sensor for testing in simulation.
 * 
 * In simulation, you can manually control when the beam is broken by calling
 * setBeamBroken(). This makes it easy to test game piece detection logic.
 * 
 * Usage in simulation:
 * BeamBreakSensorSim sensor = new BeamBreakSensorSim(0, 0.05);
 * sensor.setBeamBroken(true); // Simulate game piece detected
 * sensor.setBeamBroken(false); // Simulate game piece ejected
 */
public class BeamBreakSensorSim extends BeamBreakSensor {
	private boolean simulatedState = false;

	/**
	 * Creates a simulated beam break sensor.
	 * 
	 * @param channel
	 *            Digital I/O channel (for compatibility, but not used in sim)
	 * @param debounceTime
	 *            Minimum time between state changes (seconds)
	 */
	public BeamBreakSensorSim(int channel, double debounceTime) {
		super(channel, debounceTime);
	}

	/**
	 * Manually sets the beam break state for simulation testing.
	 * 
	 * @param broken
	 *            true to simulate beam broken (game piece present), false otherwise
	 */
	public void setBeamBroken(boolean broken) {
		simulatedState = broken;
	}

	/**
	 * Reads the simulated state.
	 * In simulation, returns the manually set state rather than reading hardware.
	 * 
	 * @return true if beam is broken (object detected), false otherwise
	 */
	@Override
	public boolean isBeamBroken() {
		// In simulation, return the manually set state
		return simulatedState;
	}
}
