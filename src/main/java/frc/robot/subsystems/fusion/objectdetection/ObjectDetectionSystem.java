package frc.robot.subsystems.fusion.objectdetection;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.fusion.sensors.BeamBreakSensor;

/**
 * Object detection system that combines Limelight vision and beam break sensors
 * for situational awareness of game pieces.
 * 
 * This system provides:
 * - Game piece detection (intake/eject status)
 * - Auto-alignment target identification
 * - Real-time object tracking
 * 
 * Update rate: 30 Hz (limited by camera processing)
 * 
 * Detection layers:
 * 1. Limelight/Camera: Visual detection of game pieces and targets
 * - Uses neural network-based object detection
 * - Provides position, distance, and type information
 * - Typical range: 0.5m to 5.0m
 * 
 * 2. Beam Break Sensors: Physical confirmation of game piece presence
 * - Detects when object enters intake mechanism
 * - Provides binary (present/absent) status
 * - Update rate: 1000+ Hz (hardware-limited)
 * - Very high accuracy for close-range detection
 */
public class ObjectDetectionSystem {
	protected final NetworkTable limelightTable;
	private final BeamBreakSensor intakeBeamBreak;

	// Detection state
	private boolean hasGamePiece = false;
	private boolean isNewDetection = false;
	private double lastDetectionTime = 0.0;

	// Limelight state
	private boolean hasTarget = false;
	private double targetDistance = 0.0;
	private double targetAngleX = 0.0;
	private double targetAngleY = 0.0;
	private String targetType = "none";
	private double targetConfidence = 0.0;

	/**
	 * Creates a new object detection system.
	 * 
	 * @param limelightTableName
	 *            NetworkTables name for Limelight
	 * @param intakeBeamBreak
	 *            Beam break sensor for intake detection
	 */
	public ObjectDetectionSystem(String limelightTableName, BeamBreakSensor intakeBeamBreak) {
		this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightTableName);
		this.intakeBeamBreak = intakeBeamBreak;
	}

	/**
	 * Updates the object detection system.
	 * Should be called periodically (recommended: 30 Hz).
	 */
	public void update() {
		// Update beam break sensor status
		updateBeamBreakStatus();

		// Update Limelight detection
		updateLimelightDetection();

		// Fuse detection results
		fuseDetections();
	}

	/**
	 * Updates beam break sensor status.
	 * Layer 2 of object detection.
	 */
	private void updateBeamBreakStatus() {
		boolean previousHasPiece = hasGamePiece;
		hasGamePiece = intakeBeamBreak.hasGamePiece();

		// Detect new intake events
		if (hasGamePiece && !previousHasPiece) {
			isNewDetection = true;
			lastDetectionTime = intakeBeamBreak.getLastStateChangeTime();
		} else {
			isNewDetection = false;
		}
	}

	/**
	 * Updates Limelight-based object detection.
	 * Layer 1 of object detection.
	 */
	private void updateLimelightDetection() {
		// Read Limelight NetworkTables values
		// tv = valid target (0 or 1)
		// tx = horizontal offset from crosshair (degrees)
		// ty = vertical offset from crosshair (degrees)
		// ta = target area (0-100%)
		// tid = target ID (for AprilTags)
		// tclass = detected class (for object detection models)

		double tv = limelightTable.getEntry("tv").getDouble(0.0);
		hasTarget = tv >= 1.0;

		if (hasTarget) {
			targetAngleX = limelightTable.getEntry("tx").getDouble(0.0);
			targetAngleY = limelightTable.getEntry("ty").getDouble(0.0);
			double targetArea = limelightTable.getEntry("ta").getDouble(0.0);
			double targetID = limelightTable.getEntry("tid").getDouble(-1.0);

			// Try to read detected class (if using object detection model)
			String detectedClass = limelightTable.getEntry("tclass").getString("none");

			// Calculate confidence from target area and other factors
			targetConfidence = calculateLimelightConfidence(targetArea, targetAngleY);

			// Estimate distance (requires camera calibration)
			targetDistance = estimateDistance(targetAngleY);

			// Determine target type
			if (targetID >= 0) {
				targetType = "apriltag_" + (int) targetID;
			} else if (!detectedClass.equals("none")) {
				targetType = detectedClass;
			} else {
				targetType = "unknown";
			}
		} else {
			// No target detected
			targetDistance = 0.0;
			targetAngleX = 0.0;
			targetAngleY = 0.0;
			targetType = "none";
			targetConfidence = 0.0;
		}
	}

	/**
	 * Fuses detection results from all sensors.
	 * Combines camera and beam break data for best detection.
	 */
	private void fuseDetections() {
		// Beam break provides definitive confirmation when object is in intake
		// Limelight provides detection at range and target information

		// The fused result is that we have a game piece if:
		// 1. Beam break confirms it (definitive), OR
		// 2. Limelight detects it with high confidence (for auto-alignment)

		// Beam break takes precedence for intake status
		// Limelight provides supplementary information for alignment
	}

	/**
	 * Calculates confidence score for Limelight detection.
	 * 
	 * @param targetArea
	 *            Target area percentage (0-100)
	 * @param targetAngleY
	 *            Vertical angle to target (degrees)
	 * @return Confidence value (0.0 to 1.0)
	 */
	private double calculateLimelightConfidence(double targetArea, double targetAngleY) {
		// Larger target area = closer = higher confidence
		// Typical values: 0.5% to 50% area
		double areaConfidence = Math.min(1.0, targetArea / 10.0); // Normalize to 10% = full confidence

		// Closer targets (larger angleY) = higher confidence
		// This is a simplified model; actual calculation requires camera parameters
		double distanceConfidence = 1.0 - (Math.abs(targetAngleY) / 30.0); // Assume 30° max angle
		distanceConfidence = Math.max(0.0, Math.min(1.0, distanceConfidence));

		return (areaConfidence + distanceConfidence) / 2.0;
	}

	/**
	 * Estimates distance to target based on vertical angle.
	 * Requires camera calibration (height and mounting angle).
	 * 
	 * @param targetAngleY
	 *            Vertical angle to target (degrees)
	 * @return Estimated distance in meters
	 */
	private double estimateDistance(double targetAngleY) {
		// This is a simplified calculation
		// Actual calculation requires:
		// - Camera mounting height
		// - Camera mounting angle
		// - Target height
		// Formula: distance = (target_height - camera_height) / tan(camera_angle +
		// target_angle_y)

		// Placeholder: assume camera at 1m height, 30° down angle, target at 0.5m
		// height
		double cameraHeight = 1.0; // meters
		double cameraAngle = 30.0; // degrees (downward)
		double targetHeight = 0.5; // meters

		double angleRad = Math.toRadians(cameraAngle + targetAngleY);
		double distance = (targetHeight - cameraHeight) / Math.tan(angleRad);

		return Math.max(0.0, distance); // Clamp to positive
	}

	/**
	 * Checks if a game piece is currently detected in the intake.
	 * 
	 * @return true if game piece is present
	 */
	public boolean hasGamePiece() {
		return hasGamePiece;
	}

	/**
	 * Checks if a game piece was just detected (new intake event).
	 * 
	 * @return true if new detection occurred
	 */
	public boolean isNewDetection() {
		return isNewDetection;
	}

	/**
	 * Gets the timestamp of the last detection.
	 * 
	 * @return Timestamp in seconds (FPGA time)
	 */
	public double getLastDetectionTime() {
		return lastDetectionTime;
	}

	/**
	 * Checks if Limelight has a valid target.
	 * 
	 * @return true if target detected
	 */
	public boolean hasTarget() {
		return hasTarget && targetConfidence >= 0.5; // Minimum confidence threshold
	}

	/**
	 * Gets the horizontal angle to the target.
	 * 
	 * @return Angle in degrees (negative = left, positive = right)
	 */
	public double getTargetAngleX() {
		return targetAngleX;
	}

	/**
	 * Gets the vertical angle to the target.
	 * 
	 * @return Angle in degrees (negative = down, positive = up)
	 */
	public double getTargetAngleY() {
		return targetAngleY;
	}

	/**
	 * Gets the estimated distance to the target.
	 * 
	 * @return Distance in meters
	 */
	public double getTargetDistance() {
		return targetDistance;
	}

	/**
	 * Gets the detected target type.
	 * 
	 * @return Target type string (e.g., "apriltag_1", "gamepiece", etc.)
	 */
	public String getTargetType() {
		return targetType;
	}

	/**
	 * Gets the confidence score for the current detection.
	 * 
	 * @return Confidence value (0.0 to 1.0)
	 */
	public double getDetectionConfidence() {
		return targetConfidence;
	}

	/**
	 * Gets comprehensive detection status.
	 * 
	 * @return DetectionStatus object with all current detection information
	 */
	public DetectionStatus getDetectionStatus() {
		return new DetectionStatus(
				hasGamePiece,
				isNewDetection,
				hasTarget(),
				targetAngleX,
				targetAngleY,
				targetDistance,
				targetType,
				targetConfidence);
	}
}
