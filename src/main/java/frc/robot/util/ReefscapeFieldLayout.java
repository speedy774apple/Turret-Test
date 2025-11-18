package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * Custom AprilTag field layout for 2025 REEFSCAPE half-field (tags 11-22).
 * 
 * Uses WPILib 2025 alliance-agnostic coordinate system with origin at the
 * Red Alliance Driver Station corner.
 * 
 * Hardcoded tag positions for reef tags only (11, 17-22).
 * Note: Tags 12-16 are not included. Add them if needed.
 * 
 * USAGE EXAMPLE:
 * 
 * // In RobotContainer or VisionSubsystem:
 * Transform3d robotToCamera = new Transform3d(...); // Your camera position
 * PhotonPoseEstimator estimator =
 * ReefscapeFieldLayout.createPhotonPoseEstimator(robotToCamera);
 * PhotonCamera camera = ReefscapeFieldLayout.getCamera();
 * 
 * // In periodic():
 * Optional<EstimatedRobotPose> visionEstimate =
 * ReefscapeFieldLayout.getEstimatedPose(estimator, camera);
 * if (visionEstimate.isPresent() &&
 * !ReefscapeFieldLayout.shouldRejectPose(visionEstimate.get())) {
 * Pose2d pose2d = visionEstimate.get().estimatedPose.toPose2d();
 * var stdDevs = ReefscapeFieldLayout.getVisionStdDevs(visionEstimate.get());
 * drive.addVisionMeasurement(pose2d, visionEstimate.get().timestampSeconds,
 * stdDevs);
 * ReefscapeFieldLayout.logPoseToAdvantageScope(visionEstimate.get(), true);
 * }
 * 
 * See ReefscapeFieldLayoutUsageExample.java for complete integration examples.
 */
public final class ReefscapeFieldLayout {

	// Field dimensions from 2025 REEFSCAPE manual
	private static final double FIELD_LENGTH = Units.feetToMeters(57.0) + Units.inchesToMeters(6.875); // 17.545 m
	private static final double FIELD_WIDTH = Units.feetToMeters(26.0) + Units.inchesToMeters(5.0); // 8.051 m

	// Camera name
	private static final String CAMERA_NAME = "FrontCam";

	// Vision quality thresholds
	private static final double MAX_AMBIGUITY = 0.2; // Reject poses with ambiguity > 0.2
	private static final double MAX_TAG_DISTANCE = 8.0; // Reject poses with tags > 8m away
	private static final double MIN_TAG_COUNT = 1; // Minimum number of tags required

	/**
	 * Creates a custom AprilTagFieldLayout with hardcoded tag positions.
	 * Only includes tags 11, 17-22 (reef tags).
	 * 
	 * @return Custom field layout with hardcoded tags
	 */
	public static AprilTagFieldLayout createHalfFieldLayout() {
		List<AprilTag> tags = new ArrayList<>();

		// Hardcoded tag positions (EXACT values as provided)
		// Tag 11: Coral Station
		tags.add(new AprilTag(11, new Pose3d(
				4.073, 3.305, 0.174625,
				new Rotation3d(0, 0, Math.toRadians(120)))));

		// Reef tags 17-22
		tags.add(new AprilTag(17, new Pose3d(
				14.722, 4.026, 0.174625,
				new Rotation3d(0, 0, Math.toRadians(180)))));

		tags.add(new AprilTag(18, new Pose3d(
				14.306, 3.305, 0.174625,
				new Rotation3d(0, 0, Math.toRadians(240)))));

		tags.add(new AprilTag(19, new Pose3d(
				13.059, 3.305, 0.174625,
				new Rotation3d(0, 0, Math.toRadians(300)))));

		tags.add(new AprilTag(20, new Pose3d(
				13.059, 4.026, 0.174625,
				new Rotation3d(0, 0, Math.toRadians(0)))));

		tags.add(new AprilTag(21, new Pose3d(
				13.475, 4.747, 0.174625,
				new Rotation3d(0, 0, Math.toRadians(60)))));

		tags.add(new AprilTag(22, new Pose3d(
				14.306, 4.747, 0.174625,
				new Rotation3d(0, 0, Math.toRadians(120)))));

		// Create layout with hardcoded tags
		return new AprilTagFieldLayout(tags, FIELD_LENGTH, FIELD_WIDTH);
	}

	/**
	 * Creates and configures a PhotonPoseEstimator for the half-field layout.
	 * 
	 * @param robotToCamera
	 *            Transform from robot center to camera
	 * @return Configured PhotonPoseEstimator
	 */
	public static PhotonPoseEstimator createPhotonPoseEstimator(Transform3d robotToCamera) {
		// Get custom field layout
		AprilTagFieldLayout fieldLayout = createHalfFieldLayout();

		// Create PhotonPoseEstimator (2025 API: no camera in constructor)
		// Use LOWEST_AMBIGUITY as default - can be overridden per-update
		PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
				fieldLayout,
				PoseStrategy.LOWEST_AMBIGUITY, // Default strategy
				robotToCamera);

		return poseEstimator;
	}

	/**
	 * Gets the PhotonCamera instance for the FrontCam.
	 * 
	 * @return PhotonCamera instance
	 */
	public static PhotonCamera getCamera() {
		return new PhotonCamera(CAMERA_NAME);
	}

	/**
	 * Gets the latest estimated robot pose from PhotonPoseEstimator.
	 * 
	 * @param estimator
	 *            The PhotonPoseEstimator to query
	 * @param camera
	 *            The PhotonCamera to get results from
	 * @return Optional containing EstimatedRobotPose if available, empty otherwise
	 */
	public static Optional<EstimatedRobotPose> getEstimatedPose(PhotonPoseEstimator estimator, PhotonCamera camera) {
		// Get latest result from camera and update estimator
		var result = camera.getLatestResult();
		if (result.hasTargets()) {
			// Update estimator with latest camera result
			// Strategy is set in constructor, but can be overridden here if needed
			return estimator.update(result);
		}
		return Optional.empty();
	}

	/**
	 * Checks if an estimated pose should be rejected due to low quality or
	 * ambiguity.
	 * 
	 * @param estimatedPose
	 *            The estimated pose to validate
	 * @return true if pose should be rejected, false if acceptable
	 */
	public static boolean shouldRejectPose(EstimatedRobotPose estimatedPose) {
		// Check ambiguity - access from the pose estimate structure
		// Ambiguity is typically in the multitag result, check if available
		double ambiguity = 0.0;
		if (estimatedPose.targetsUsed.size() > 0) {
			// For single tags, use poseAmbiguity; for multitag, it's in the result
			// Default to 0 if not available (multitag is more reliable)
			ambiguity = estimatedPose.targetsUsed.get(0).poseAmbiguity;
		}
		if (ambiguity > MAX_AMBIGUITY) {
			return true;
		}

		// Check tag count
		if (estimatedPose.targetsUsed.size() < MIN_TAG_COUNT) {
			return true;
		}

		// Check average tag distance
		double avgDistance = estimatedPose.targetsUsed.stream()
				.mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
				.average()
				.orElse(Double.MAX_VALUE);

		if (avgDistance > MAX_TAG_DISTANCE) {
			return true;
		}

		// Check if pose is within field bounds (with margin)
		Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
		if (pose2d.getX() < -1.0 || pose2d.getX() > FIELD_LENGTH + 1.0 ||
				pose2d.getY() < -1.0 || pose2d.getY() > FIELD_WIDTH + 1.0) {
			return true;
		}

		// Check Z coordinate (robot should be on ground)
		if (Math.abs(estimatedPose.estimatedPose.getZ()) > 0.5) {
			return true;
		}

		return false;
	}

	/**
	 * Converts PhotonVision EstimatedRobotPose to SwerveDrivePoseEstimator format.
	 * 
	 * @param estimatedPose
	 *            The PhotonVision estimated pose
	 * @return Matrix containing standard deviations for x, y, and theta (in that
	 *         order)
	 */
	public static Matrix<N3, N1> getVisionStdDevs(EstimatedRobotPose estimatedPose) {
		int tagCount = estimatedPose.targetsUsed.size();
		double avgDistance = estimatedPose.targetsUsed.stream()
				.mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
				.average()
				.orElse(5.0); // Default to 5m if no targets

		// Calculate uncertainty based on tag count and distance
		// More tags and closer tags = lower uncertainty (more trust)
		double linearStdDev = 0.02 * Math.pow(avgDistance, 2) / tagCount;
		double angularStdDev = 0.06 * Math.pow(avgDistance, 2) / tagCount;

		// Increase uncertainty for single tags
		if (tagCount == 1) {
			linearStdDev *= 2.0;
			angularStdDev *= 2.5;
		}

		// Ensure minimum uncertainty (don't trust vision too much)
		linearStdDev = Math.max(linearStdDev, 0.08); // At least 8cm
		angularStdDev = Math.max(angularStdDev, 0.05); // At least ~3°

		// Cap maximum uncertainty
		linearStdDev = Math.min(linearStdDev, 0.5); // Max 50cm
		angularStdDev = Math.min(angularStdDev, 0.3); // Max ~17°

		return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
	}

	/**
	 * Logs the estimated pose to AdvantageScope for visualization.
	 * 
	 * @param estimatedPose
	 *            The estimated pose to log
	 * @param accepted
	 *            Whether the pose was accepted or rejected
	 */
	public static void logPoseToAdvantageScope(EstimatedRobotPose estimatedPose, boolean accepted) {
		Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();

		Logger.recordOutput("Vision/PhotonPoseEstimator/Pose", pose2d);
		Logger.recordOutput("Vision/PhotonPoseEstimator/Timestamp", estimatedPose.timestampSeconds);
		// Calculate ambiguity from targets
		double ambiguity = 0.0;
		if (estimatedPose.targetsUsed.size() > 0) {
			ambiguity = estimatedPose.targetsUsed.get(0).poseAmbiguity;
		}
		Logger.recordOutput("Vision/PhotonPoseEstimator/Ambiguity", ambiguity);
		Logger.recordOutput("Vision/PhotonPoseEstimator/TagCount", estimatedPose.targetsUsed.size());
		Logger.recordOutput("Vision/PhotonPoseEstimator/Accepted", accepted);

		// Log tag IDs seen
		int[] tagIds = estimatedPose.targetsUsed.stream()
				.mapToInt(target -> target.getFiducialId())
				.toArray();
		Logger.recordOutput("Vision/PhotonPoseEstimator/TagIDs", tagIds);

		// Log average tag distance
		double avgDistance = estimatedPose.targetsUsed.stream()
				.mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
				.average()
				.orElse(0.0);
		Logger.recordOutput("Vision/PhotonPoseEstimator/AvgTagDistance", avgDistance);
	}

	/**
	 * Gets the field layout (for use in other parts of code).
	 * 
	 * @return AprilTagFieldLayout with tags 11, 17-22
	 */
	public static AprilTagFieldLayout getFieldLayout() {
		return createHalfFieldLayout();
	}

	/**
	 * Gets the camera name.
	 * 
	 * @return Camera name ("FrontCam")
	 */
	public static String getCameraName() {
		return CAMERA_NAME;
	}
}
