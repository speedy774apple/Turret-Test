package frc.robot.subsystems.fusion;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.fusion.objectdetection.ObjectDetectionSystem;
import frc.robot.subsystems.fusion.sensors.BeamBreakSensor;
import frc.robot.subsystems.vision.VisionLocalizer;

/**
 * Factory class for creating and configuring the sensor fusion system.
 * 
 * This class simplifies the setup of the complete fusion system by handling
 * the initialization of all components and their interconnections.
 * 
 * SIMULATION SUPPORT:
 * For simulation testing, you can pass pre-created simulated components:
 * - BeamBreakSensorSim for manual control of beam break sensor
 * - ObjectDetectionSystemSim for manual control of object detection
 * 
 * Use createFusionSystemWithComponents() to pass pre-created simulated
 * components.
 */
public class SensorFusionFactory {

	/**
	 * Creates a complete sensor fusion system with all layers integrated.
	 * 
	 * @param drive
	 *            Drive subsystem (must already be initialized)
	 * @param vision
	 *            Vision localizer (must already be initialized)
	 * @param limelightTableName
	 *            NetworkTables name for Limelight (or null to skip)
	 * @param beamBreakChannel
	 *            Digital I/O channel for beam break sensor (or -1 to skip)
	 * @return Configured SensorFusionManager instance
	 */
	public static SensorFusionManager createFusionSystem(
			Drive drive,
			VisionLocalizer vision,
			String limelightTableName,
			int beamBreakChannel) {

		// Create enhanced pose estimator that wraps the Drive subsystem
		// The Drive subsystem already has its own SwerveDrivePoseEstimator
		EnhancedPoseEstimator enhancedEstimator = new EnhancedPoseEstimator(drive::getPose);

		// Create beam break sensor (if channel specified)
		BeamBreakSensor beamBreak = null;
		if (beamBreakChannel >= 0) {
			beamBreak = new BeamBreakSensor(
					beamBreakChannel,
					SensorFusionConstants.BEAM_BREAK_DEBOUNCE_TIME);
		}

		// Create object detection system (if sensors available)
		ObjectDetectionSystem objectDetection = null;
		if (beamBreak != null || limelightTableName != null) {
			String limelightName = (limelightTableName != null)
					? limelightTableName
					: SensorFusionConstants.LIMELIGHT_TABLE_NAME;

			objectDetection = new ObjectDetectionSystem(limelightName, beamBreak);
		}

		// Create and return fusion manager
		return new SensorFusionManager(drive, vision, enhancedEstimator, objectDetection);
	}

	/**
	 * Creates a minimal sensor fusion system (pose estimation only, no object
	 * detection).
	 * 
	 * @param drive
	 *            Drive subsystem
	 * @param vision
	 *            Vision localizer
	 * @return Configured SensorFusionManager instance
	 */
	public static SensorFusionManager createMinimalFusionSystem(
			Drive drive,
			VisionLocalizer vision) {

		return createFusionSystem(drive, vision, null, -1);
	}

	/**
	 * Creates a sensor fusion system with pre-created components (for
	 * simulation/testing).
	 * 
	 * @param drive
	 *            Drive subsystem
	 * @param vision
	 *            Vision localizer
	 * @param enhancedEstimator
	 *            Pre-created enhanced pose estimator (null to create new)
	 * @param objectDetection
	 *            Pre-created object detection system (null to skip)
	 * @return Configured SensorFusionManager instance
	 */
	public static SensorFusionManager createFusionSystemWithComponents(
			Drive drive,
			VisionLocalizer vision,
			EnhancedPoseEstimator enhancedEstimator,
			ObjectDetectionSystem objectDetection) {

		// Use provided estimator or create new one
		if (enhancedEstimator == null) {
			enhancedEstimator = new EnhancedPoseEstimator(drive::getPose);
		}

		return new SensorFusionManager(drive, vision, enhancedEstimator, objectDetection);
	}
}
