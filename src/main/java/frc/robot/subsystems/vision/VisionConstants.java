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
			"FR",
			"BL",
			"BR"
	};

	public static final Transform3d[] vehicleToCameras = {
			new Transform3d(new Translation3d(0.263383, 0.275693, 0.259765),
					new Rotation3d(0, 0, Units.degreesToRadians(-45))), // FL
			new Transform3d(new Translation3d(0.263383, -0.275693, 0.259765),
					new Rotation3d(0, 0, Units.degreesToRadians(42.5))), // FR
			new Transform3d(new Translation3d(-0.3459226, 0.3729482, 0.15875),
					new Rotation3d(0, 0, Units.degreesToRadians(-90 - 35))), // BL
			new Transform3d(new Translation3d(-0.3459226, -0.3729482, 0.15875),
					new Rotation3d(0, 0, Units.degreesToRadians(90 + 35))), // BR
			// Add additional cameras here
	};

	public static final List<CameraParams> cameras = List.of(
			new CameraParams(cameraNames[0], vehicleToCameras[0]),
			new CameraParams(cameraNames[1], vehicleToCameras[1]));

	// Field Layout for visual localization and map generation
	public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
			.loadField(AprilTagFields.k2025ReefscapeWelded);
}
