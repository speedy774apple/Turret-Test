package frc.robot.subsystems.fusion;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.fusion.objectdetection.DetectionStatus;
import frc.robot.subsystems.fusion.objectdetection.ObjectDetectionSystemSim;
import frc.robot.subsystems.fusion.sensors.BeamBreakSensorSim;

/**
 * Helper class for easy simulation testing of the sensor fusion system.
 * 
 * This class provides convenient methods to simulate sensor inputs during
 * testing.
 * 
 * Example usage:
 * 
 * // In RobotContainer or test code:
 * SensorFusionManager fusion = ...;
 * SensorFusionSimHelper simHelper = new SensorFusionSimHelper(fusion);
 * 
 * // Simulate game piece detection
 * simHelper.simulateGamePieceDetected();
 * 
 * // Simulate target visible
 * simHelper.simulateTargetDetected(5.0, -10.0, 2.5, "apriltag_1", 0.8);
 * 
 * // Check fusion outputs
 * Pose2d pose = fusion.getFusedPose();
 */
public class SensorFusionSimHelper {
	private final SensorFusionManager fusionManager;
	private final BeamBreakSensorSim beamBreakSim;
	private final ObjectDetectionSystemSim objectDetectionSim;

	/**
	 * Creates a simulation helper.
	 * 
	 * @param fusionManager
	 *            The sensor fusion manager to control
	 * @param beamBreakSim
	 *            Simulated beam break sensor (null if not using)
	 * @param objectDetectionSim
	 *            Simulated object detection system (null if not using)
	 */
	public SensorFusionSimHelper(
			SensorFusionManager fusionManager,
			BeamBreakSensorSim beamBreakSim,
			ObjectDetectionSystemSim objectDetectionSim) {
		this.fusionManager = fusionManager;
		this.beamBreakSim = beamBreakSim;
		this.objectDetectionSim = objectDetectionSim;
	}

	/**
	 * Simulates a game piece being detected in the intake.
	 */
	public void simulateGamePieceDetected() {
		if (beamBreakSim != null) {
			beamBreakSim.setBeamBroken(true);
		}
	}

	/**
	 * Simulates a game piece being ejected from the intake.
	 */
	public void simulateGamePieceEjected() {
		if (beamBreakSim != null) {
			beamBreakSim.setBeamBroken(false);
		}
	}

	/**
	 * Simulates a target being detected by the camera.
	 * 
	 * @param angleX
	 *            Horizontal angle (degrees, negative = left, positive = right)
	 * @param angleY
	 *            Vertical angle (degrees)
	 * @param distance
	 *            Distance (meters)
	 * @param targetType
	 *            Target type (e.g., "apriltag_1")
	 * @param confidence
	 *            Confidence (0.0 to 1.0)
	 */
	public void simulateTargetDetected(
			double angleX,
			double angleY,
			double distance,
			String targetType,
			double confidence) {
		if (objectDetectionSim != null) {
			objectDetectionSim.simulateTargetDetected(angleX, angleY, distance, targetType, confidence);
		}
	}

	/**
	 * Simulates no target visible.
	 */
	public void simulateNoTarget() {
		if (objectDetectionSim != null) {
			objectDetectionSim.simulateNoTarget();
		}
	}

	/**
	 * Gets the current fused pose estimate.
	 * 
	 * @return Current pose
	 */
	public Pose2d getFusedPose() {
		return fusionManager.getFusedPose();
	}

	/**
	 * Gets the current detection status.
	 * 
	 * @return Detection status
	 */
	public DetectionStatus getDetectionStatus() {
		return fusionManager.getDetectionStatus();
	}
}
