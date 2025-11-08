package frc.robot.subsystems.fusion.objectdetection;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.fusion.sensors.BeamBreakSensor;

/**
 * Simulated object detection system for testing in simulation.
 * 
 * This extends ObjectDetectionSystem and adds simulation-friendly controls
 * for testing detection behavior without requiring actual cameras or sensors.
 * 
 * Usage in simulation:
 * ObjectDetectionSystemSim detection = new ObjectDetectionSystemSim(...);
 * detection.simulateTargetDetected(angleX, angleY, distance); // Simulate
 * seeing a target
 * detection.simulateNoTarget(); // Simulate no target visible
 */
public class ObjectDetectionSystemSim extends ObjectDetectionSystem {

	/**
	 * Creates a simulated object detection system.
	 * 
	 * @param limelightTableName
	 *            NetworkTables name for Limelight
	 * @param intakeBeamBreak
	 *            Beam break sensor (can be BeamBreakSensorSim for manual control)
	 */
	public ObjectDetectionSystemSim(String limelightTableName, BeamBreakSensor intakeBeamBreak) {
		super(limelightTableName, intakeBeamBreak);
	}

	/**
	 * Simulates detecting a target at the specified position.
	 * This sets the Limelight NetworkTables values directly for testing.
	 * 
	 * @param angleX
	 *            Horizontal angle to target (degrees, negative = left, positive =
	 *            right)
	 * @param angleY
	 *            Vertical angle to target (degrees)
	 * @param distance
	 *            Distance to target (meters, estimated)
	 * @param targetType
	 *            Target type string (e.g., "apriltag_1", "gamepiece")
	 * @param confidence
	 *            Confidence score (0.0 to 1.0)
	 */
	public void simulateTargetDetected(
			double angleX,
			double angleY,
			double distance,
			String targetType,
			double confidence) {
		NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(getLimelightTableName());

		// Set valid target flag
		limelightTable.getEntry("tv").setDouble(1.0);

		// Set angles
		limelightTable.getEntry("tx").setDouble(angleX);
		limelightTable.getEntry("ty").setDouble(angleY);

		// Estimate target area from distance (closer = larger area)
		// Simplified: assume 10% area at 1m, scales with 1/distance^2
		double estimatedArea = 10.0 / (distance * distance);
		limelightTable.getEntry("ta").setDouble(Math.min(100.0, estimatedArea));

		// Set target type
		if (targetType.startsWith("apriltag_")) {
			try {
				int tagId = Integer.parseInt(targetType.replace("apriltag_", ""));
				limelightTable.getEntry("tid").setDouble(tagId);
			} catch (NumberFormatException e) {
				limelightTable.getEntry("tid").setDouble(-1);
			}
			limelightTable.getEntry("tclass").setString(targetType);
		} else {
			limelightTable.getEntry("tid").setDouble(-1);
			limelightTable.getEntry("tclass").setString(targetType);
		}
	}

	/**
	 * Simulates no target detected.
	 */
	public void simulateNoTarget() {
		NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(getLimelightTableName());
		limelightTable.getEntry("tv").setDouble(0.0);
		limelightTable.getEntry("tx").setDouble(0.0);
		limelightTable.getEntry("ty").setDouble(0.0);
		limelightTable.getEntry("ta").setDouble(0.0);
		limelightTable.getEntry("tid").setDouble(-1);
		limelightTable.getEntry("tclass").setString("none");
	}

	/**
	 * Gets the Limelight table name (for accessing NetworkTables directly if
	 * needed).
	 * 
	 * @return Table name
	 */
	private String getLimelightTableName() {
		// Access the protected limelightTable field
		// The table name is the last part of the path
		return limelightTable.getPath().replaceFirst("^/", ""); // Remove leading slash if present
	}
}
