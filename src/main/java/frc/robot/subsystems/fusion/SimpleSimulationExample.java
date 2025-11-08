package frc.robot.subsystems.fusion;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.fusion.objectdetection.DetectionStatus;
import frc.robot.subsystems.fusion.objectdetection.ObjectDetectionSystemSim;
import frc.robot.subsystems.fusion.sensors.BeamBreakSensorSim;
import frc.robot.subsystems.vision.VisionLocalizer;

/**
 * Simple example showing how to set up sensor fusion for simulation testing.
 * 
 * Copy this pattern into your RobotContainer.java
 */
public class SimpleSimulationExample {

	/**
	 * Example: How to create sensor fusion for simulation
	 */
	public static SensorFusionManager createForSimulation(
			Drive drive,
			VisionLocalizer vision) {

		// 1. Create simulated sensors
		BeamBreakSensorSim beamBreakSim = new BeamBreakSensorSim(
				SensorFusionConstants.INTAKE_BEAM_BREAK_PORT,
				SensorFusionConstants.BEAM_BREAK_DEBOUNCE_TIME);

		ObjectDetectionSystemSim objectDetectionSim = new ObjectDetectionSystemSim(
				SensorFusionConstants.LIMELIGHT_TABLE_NAME,
				beamBreakSim);

		// 2. Create enhanced pose estimator
		EnhancedPoseEstimator enhancedEstimator = new EnhancedPoseEstimator(drive::getPose);

		// 3. Create fusion manager with simulated components
		SensorFusionManager fusion = SensorFusionFactory.createFusionSystemWithComponents(
				drive,
				vision,
				enhancedEstimator,
				objectDetectionSim);

		return fusion;
	}

	/**
	 * Example: How to test sensor fusion in simulation
	 */
	public static void testExample(SensorFusionManager fusion, BeamBreakSensorSim beamBreakSim) {
		// Test game piece detection
		beamBreakSim.setBeamBroken(true);

		// Wait a bit for system to update
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
		}

		// Check detection status
		DetectionStatus status = fusion.getDetectionStatus();
		if (status != null) {
			System.out.println("Has game piece: " + status.hasGamePiece());
		}

		// Get fused pose
		Pose2d pose = fusion.getFusedPose();
		System.out.println("Current pose: " + pose);

		// Get fusion statistics
		var stats = fusion.getFusionStatistics();
		System.out.println("Vision confidence: " + stats.visionConfidence());
		System.out.println("Vision measurements: " + stats.visionMeasurements());
	}
}
