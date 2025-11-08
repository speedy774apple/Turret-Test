package frc.robot.subsystems.fusion;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionLocalizer;

/**
 * Example integration of the sensor fusion system into RobotContainer.
 * 
 * This file demonstrates how to integrate the sensor fusion system.
 * Copy the relevant parts into your RobotContainer.java file.
 * 
 * EXAMPLE USAGE:
 * 
 * 1. Add field to RobotContainer:
 * 
 * private SensorFusionManager sensorFusion;
 * 
 * 2. Initialize in RobotContainer constructor:
 * 
 * // After initializing drive and vision:
 * sensorFusion = SensorFusionFactory.createFusionSystem(
 * drive,
 * vision,
 * "limelight", // Limelight table name (null if not using)
 * SensorFusionConstants.INTAKE_BEAM_BREAK_PORT // Beam break DIO channel (-1 if
 * not using)
 * );
 * 
 * 3. Use in commands/autonomous:
 * 
 * // Get fused pose estimate
 * Pose2d robotPose = sensorFusion.getFusedPose();
 * 
 * // Check for game pieces
 * DetectionStatus detection = sensorFusion.getDetectionStatus();
 * if (detection != null && detection.hasGamePiece()) {
 * // Handle game piece detected
 * }
 * 
 * // Check for alignment targets
 * if (detection != null && detection.hasTarget()) {
 * double angleToTarget = detection.targetAngleX();
 * // Use for auto-alignment
 * }
 * 
 * 4. Reset pose (e.g., at match start):
 * 
 * sensorFusion.resetPose(new Pose2d(x, y, rotation));
 * 
 * 5. Monitor fusion statistics:
 * 
 * var stats = sensorFusion.getFusionStatistics();
 * System.out.println("Vision confidence: " + stats.visionConfidence());
 * 
 * NOTE: The SensorFusionManager extends SubsystemBase, so it will
 * automatically update in the robot periodic loop. No manual update calls
 * needed!
 */
public class SensorFusionIntegrationExample {
	// This is just an example/documentation file
	// See the comments above for integration instructions
}
