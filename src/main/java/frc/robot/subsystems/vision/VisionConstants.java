package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {

	public static record CameraParams(
			String name,
			Transform3d transforms) {
	};

	// ===================================================================
	// CAMERA CONFIGURATION (ROBOT-SPECIFIC - NOT GAME-SPECIFIC)
	// ===================================================================
	// These are where YOUR cameras are mounted on YOUR robot.
	// Only update these if you change camera positions on the robot.
	// These stay the same across different games (unlike GameConstants.java)
	// ===================================================================

	/** Camera names - must match PhotonVision camera names exactly */
	public static final String[] cameraNames = {
			"FL",
			"FR", // Uncomment when you add second camera
			"BL",
			"BR"
	};

	/**
	 * Camera mounting positions on robot (robot-to-camera transforms).
	 * Format: Translation3d(x, y, z), Rotation3d(roll, pitch, yaw)
	 * 
	 * Coordinate system:
	 * - X: Forward (positive = front of robot)
	 * - Y: Left (positive = left side of robot)
	 * - Z: Up (positive = above robot)
	 * 
	 * Current values are examples - UPDATE THESE to match YOUR robot's camera
	 * positions!
	 */
	public static final Transform3d[] vehicleToCameras = {
			new Transform3d(new Translation3d(0.263383, 0.275693, 0.259765),
					new Rotation3d(0, 0, Units.degreesToRadians(-45))), // FL
			// Uncomment when you add second camera:
			// new Transform3d(new Translation3d(0.263383, -0.275693, 0.259765),
			// new Rotation3d(0, 0, Units.degreesToRadians(42.5))), // FR
			// Add additional cameras here
	};

	public static final List<CameraParams> cameras = List.of(
			new CameraParams(cameraNames[0], vehicleToCameras[0])
	// Add more cameras here when you have them:
	// new CameraParams(cameraNames[1], vehicleToCameras[1])
	);

	// Field Layout for visual localization and map generation
	// Uses GameConstants.APRILTAG_LAYOUT (game-specific, see GameConstants.java)
	// Can be replaced with custom layout from TagFieldCalibrator
	public static AprilTagFieldLayout aprilTagLayout = frc.robot.util.GameConstants.APRILTAG_LAYOUT;

	// Custom field layout (set by TagFieldCalibrator after calibration)
	private static AprilTagFieldLayout customLayout = null;

	/**
	 * Sets a custom field layout (from calibration).
	 * 
	 * @param layout
	 *            Custom AprilTag field layout
	 */
	public static void setCustomLayout(AprilTagFieldLayout layout) {
		customLayout = layout;
	}

	/**
	 * Gets the active field layout (custom if available, otherwise standard).
	 * 
	 * @return Active field layout
	 */
	public static AprilTagFieldLayout getActiveLayout() {
		return customLayout != null ? customLayout : aprilTagLayout;
	}

	/**
	 * Resets to standard field layout.
	 */
	public static void resetToStandardLayout() {
		customLayout = null;
	}
}
