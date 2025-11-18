package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import frc.robot.subsystems.drive.Drive;

/**
 * USAGE EXAMPLES for ReefscapeFieldLayout
 * 
 * This file shows how to integrate PhotonPoseEstimator into your robot code.
 * Copy the relevant sections into your actual implementation.
 */
public class ReefscapeFieldLayoutUsageExample {

	// ===================================================================
	// EXAMPLE 1: Initialization in RobotContainer
	// ===================================================================
	/**
	 * Example showing how to initialize PhotonPoseEstimator in RobotContainer.
	 * 
	 * Add this to your RobotContainer constructor:
	 */
	public static void exampleRobotContainerInit(Drive drive) {
		// Get camera-to-robot transform (UPDATE THESE VALUES!)
		// This is where your "FrontCam" is mounted on the robot
		Transform3d robotToCamera = new Transform3d(
				new Translation3d(0.26, 0.0, 0.26), // x, y, z in meters
				new Rotation3d(0, 0, Units.degreesToRadians(0)) // roll, pitch, yaw
		);

		// Create PhotonPoseEstimator with custom field layout
		PhotonPoseEstimator poseEstimator = ReefscapeFieldLayout.createPhotonPoseEstimator(robotToCamera);

		// Get camera instance
		PhotonCamera camera = ReefscapeFieldLayout.getCamera();

		// Store both for use in periodic methods
		// You would typically store these as fields in RobotContainer or a
		// VisionSubsystem
		// private PhotonPoseEstimator photonPoseEstimator;
		// private PhotonCamera camera;
	}

	// ===================================================================
	// EXAMPLE 2: Usage in VisionSubsystem or VisionLocalizer periodic()
	// ===================================================================
	/**
	 * Example showing how to use PhotonPoseEstimator in a periodic method
	 * to update the drive subsystem's pose estimator.
	 * 
	 * Add this to your VisionSubsystem or VisionLocalizer periodic() method:
	 */
	public static void exampleVisionPeriodic(PhotonPoseEstimator estimator, PhotonCamera camera, Drive drive) {
		// Get latest estimated pose from PhotonVision
		Optional<EstimatedRobotPose> visionEstimate = ReefscapeFieldLayout.getEstimatedPose(estimator, camera);

		if (visionEstimate.isPresent()) {
			EstimatedRobotPose estimatedPose = visionEstimate.get();

			// Check if pose should be rejected (low quality, ambiguous, etc.)
			if (!ReefscapeFieldLayout.shouldRejectPose(estimatedPose)) {
				// Convert to Pose2d for SwerveDrivePoseEstimator
				Pose2d visionPose2d = estimatedPose.estimatedPose.toPose2d();

				// Get standard deviations for pose estimator
				var visionStdDevs = ReefscapeFieldLayout.getVisionStdDevs(estimatedPose);

				// Add vision measurement to drive subsystem
				drive.addVisionMeasurement(
						visionPose2d,
						estimatedPose.timestampSeconds,
						visionStdDevs);

				// Log to AdvantageScope for visualization
				ReefscapeFieldLayout.logPoseToAdvantageScope(estimatedPose, true);
			} else {
				// Log rejected pose
				ReefscapeFieldLayout.logPoseToAdvantageScope(estimatedPose, false);
			}
		}
	}

	// ===================================================================
	// EXAMPLE 3: Complete VisionSubsystem Implementation
	// ===================================================================
	/**
	 * Example of a complete VisionSubsystem that uses PhotonPoseEstimator.
	 * 
	 * You can create a new VisionSubsystem class like this:
	 */
	/*
	 * public class VisionSubsystem extends SubsystemBase {
	 * private final PhotonPoseEstimator photonPoseEstimator;
	 * private final Drive drive;
	 * 
	 * public VisionSubsystem(Drive drive, Transform3d robotToCamera) {
	 * this.drive = drive;
	 * this.photonPoseEstimator =
	 * ReefscapeFieldLayout.createPhotonPoseEstimator(robotToCamera);
	 * }
	 * 
	 * @Override
	 * public void periodic() {
	 * // Get latest estimated pose
	 * Optional<EstimatedRobotPose> visionEstimate =
	 * ReefscapeFieldLayout.getEstimatedPose(photonPoseEstimator);
	 * 
	 * if (visionEstimate.isPresent()) {
	 * EstimatedRobotPose estimatedPose = visionEstimate.get();
	 * 
	 * // Validate and apply pose
	 * if (!ReefscapeFieldLayout.shouldRejectPose(estimatedPose)) {
	 * Pose2d visionPose2d = estimatedPose.estimatedPose.toPose2d();
	 * var visionStdDevs = ReefscapeFieldLayout.getVisionStdDevs(estimatedPose);
	 * 
	 * drive.addVisionMeasurement(
	 * visionPose2d,
	 * estimatedPose.timestampSeconds,
	 * visionStdDevs
	 * );
	 * 
	 * ReefscapeFieldLayout.logPoseToAdvantageScope(estimatedPose, true);
	 * } else {
	 * ReefscapeFieldLayout.logPoseToAdvantageScope(estimatedPose, false);
	 * }
	 * }
	 * }
	 * }
	 */
}
