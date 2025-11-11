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

	public static final String[] cameraNames = {
			"FL",
			// "FR", // Uncomment when you add second camera
			// "BL",
			// "BR"
	};

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
	// Default: Standard 2025 FRC field
	// Can be replaced with custom layout from TagFieldCalibrator
	public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
			.loadField(AprilTagFields.k2025ReefscapeAndyMark);

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
