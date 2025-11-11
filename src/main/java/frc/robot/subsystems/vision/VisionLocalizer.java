package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.vision.VisionIO.SingleTagObservation;
import frc.robot.subsystems.vision.VisionIOPhotonReal;
import frc.robot.subsystems.vision.VisionIOPhotonSim;

import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Localizes the robot using camera measurements. Periodically updates camera
 * data and allows for
 * custom handling of new measurements.
 */
public class VisionLocalizer extends SubsystemBase {
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;
	// avoid NullPointerExceptions by setting a default no-op
	private VisionConsumer consumer;
	private Drive drive;
	// private double[] cameraStdDevFactors;

	// Optional calibrator for custom field layouts
	private TagFieldCalibrator calibrator = null;

	/**
	 * Constructs a new VisionLocalizer instance
	 *
	 * @param consumer
	 *            functional interface responsible for
	 *            vision measurements to
	 *            drive pose
	 * @param aprilTagLayout
	 *            the field layout for current year
	 * @param cameraStdDevFactors
	 *            factors to multiply standard deviation. matches camera index
	 *            (camera 0 -> index 0 in factors)
	 * @param io
	 *            of each camera, using photon vision or sim
	 */
	public VisionLocalizer(
			VisionConsumer consumer, Drive drive,
			// double[] cameraStdDevFactors,
			VisionIO... io) {
		this.consumer = consumer;
		this.io = io;
		this.drive = drive;
		// this.cameraStdDevFactors = cameraStdDevFactors;

		// Use active layout (custom if available, otherwise standard)
		for (int i = 0; i < io.length; i++) {
			io[i].setAprilTagLayout(VisionConstants.getActiveLayout());
		}

		// Initialize inputs
		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			String cameraName = "Unknown";
			if (io[i] instanceof VisionIOPhotonReal) {
				cameraName = ((VisionIOPhotonReal) io[i]).name;
			} else if (io[i] instanceof VisionIOPhotonSim) {
				cameraName = ((VisionIOPhotonSim) io[i]).name;
			}
			disconnectedAlerts[i] = new Alert("Vision camera " + cameraName + " (index " + i + ") is disconnected.",
					AlertType.kWarning);
			System.out.println("Vision: Initialized camera " + i + " with name: " + cameraName);
		}
	}

	/**
	 * Returns the X angle to the best target, which can be used for simple servoing
	 * with vision.
	 *
	 * @param cameraIndex
	 *            The index of the camera to use.
	 */
	public Rotation2d getTargetX(int cameraIndex) {
		return inputs[cameraIndex].latestTargetObservation.tx();
	}

	/**
	 * calculates the strafing and forward / reverse required for drive to be in
	 * line with a
	 * specific tag + offset
	 *
	 * @param tagId
	 *            desired tag to align to
	 * @param desiredCameraIndex
	 *            camera to use for measurements
	 * @param crossTrackOffsetMeters
	 *            how much to offset horizontal distance by
	 * @param alongTrackOffsetMeters
	 *            how much to offset along track distance by (if camera is pushed
	 *            into robot, not aligned with bumper)
	 * @return a distance to tag with validity
	 */
	public DistanceToTag getDistanceErrorToTag(
			int tagId,
			int desiredCameraIndex,
			double crossTrackOffsetMeters,
			double alongTrackOffsetMeters) {
		// camera not in vision
		if (desiredCameraIndex >= inputs.length) {
			return new DistanceToTag(0, 0, false);
		}

		SingleTagObservation tagObserved = inputs[desiredCameraIndex].latestSingleTagObservation;

		// if tag id doesn't match, we assume we don't have that tag in view
		// therefore, no distance can be observed
		if (tagObserved.tagId() != tagId) {
			return new DistanceToTag(0, 0, false);
		}

		// get part of 3d distance lying on xy plane
		double distanceXYPlane = tagObserved.distance3D() * Math.cos(tagObserved.ty().getRadians());

		// calculate strafe and forward distances required to get to tag
		double crossTrackDistance = distanceXYPlane * Math.sin(tagObserved.tx().minus(new Rotation2d()).getRadians())
				+ crossTrackOffsetMeters;
		double alongTrackDistance = distanceXYPlane * Math.cos(tagObserved.tx().minus(new Rotation2d()).getRadians())
				+ alongTrackOffsetMeters;

		return new DistanceToTag(crossTrackDistance, alongTrackDistance, true);
	}

	/** Periodically updates the camera data and processes new measurements. */
	@Override
	public void periodic() {
		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + i, inputs[i]);
		}

		// Initialize logging values
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();

		// Track overall status
		boolean anyCameraSeesTags = false;
		int totalTagsSeen = 0;

		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			// Get camera name for better debugging
			String cameraName = "Unknown";
			if (io[cameraIndex] instanceof VisionIOPhotonReal) {
				cameraName = ((VisionIOPhotonReal) io[cameraIndex]).name;
			} else if (io[cameraIndex] instanceof VisionIOPhotonSim) {
				cameraName = ((VisionIOPhotonSim) io[cameraIndex]).name;
			}

			// ORGANIZED LOGGING - Simple status per camera with NAME
			boolean seesTags = inputs[cameraIndex].tagIds.length > 0;
			int numTagsSeen = inputs[cameraIndex].tagIds.length;
			boolean isConnected = inputs[cameraIndex].connected;

			// Get single tag observation (used for both logging and calibration)
			VisionIO.SingleTagObservation singleTag = inputs[cameraIndex].latestSingleTagObservation;

			// Main status logs - USE CAMERA NAME so you know which is which!
			Logger.recordOutput("Vision/" + cameraName + "/Connected", isConnected);
			Logger.recordOutput("Vision/" + cameraName + "/SeesTags", seesTags);
			Logger.recordOutput("Vision/Camera" + cameraIndex + "/Name", cameraName); // Also keep index for
																						// compatibility

			if (seesTags) {
				anyCameraSeesTags = true;
				totalTagsSeen += numTagsSeen;

				// Only log details when tags are actually seen (reduces clutter)
				Logger.recordOutput("Vision/" + cameraName + "/TagIDs", inputs[cameraIndex].tagIds);
				Logger.recordOutput("Vision/Camera" + cameraIndex + "/TagIDs", inputs[cameraIndex].tagIds); // Keep for
																											// compatibility

				// Single tag details (if applicable)
				if (singleTag.tagId() > 0) {
					Logger.recordOutput("Vision/" + cameraName + "/TagID", singleTag.tagId());
					Logger.recordOutput("Vision/" + cameraName + "/TagDistance", singleTag.distance3D());
				}
			} else if (!isConnected) {
				// Log when camera is disconnected so you know why it's not seeing tags
				Logger.recordOutput("Vision/" + cameraName + "/Status", "DISCONNECTED");
			}

			// Initialize logging values
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			// Collect tag observations for calibration
			// During calibration, we use robot odometry pose (trusted) + camera
			// measurements
			// to calculate absolute tag positions
			if (calibrator != null && calibrator.isCalibrating()) {
				// Get robot pose from odometry (more reliable than vision during calibration)
				Pose3d robotPoseOdometry = new Pose3d(drive.getPose());

				// Get camera-to-robot transform for this camera
				Transform3d robotToCamera = VisionConstants.vehicleToCameras[cameraIndex];

				// Process single tag observations for calibration
				// (singleTag already declared above for logging)
				if (singleTag.tagId() > 0 && singleTag.distance3D() > 0) {
					// Calculate camera-to-tag transform from observation
					// We have: distance, angles (tx, ty)
					// Build transform: distance in direction of angles
					double distance = singleTag.distance3D();
					double tx = singleTag.tx().getRadians();
					double ty = singleTag.ty().getRadians();

					// Calculate tag position relative to camera
					// Using spherical to cartesian conversion
					double x = distance * Math.cos(ty) * Math.cos(tx);
					double y = distance * Math.cos(ty) * Math.sin(tx);
					double z = distance * Math.sin(ty);

					Transform3d cameraToTag = new Transform3d(
							new Translation3d(x, y, z),
							new Rotation3d(0, ty, tx) // Simplified rotation
					);

					// Calculate absolute tag position:
					// tag_absolute = robot_absolute + robot_to_camera + camera_to_tag
					Pose3d tagPose = robotPoseOdometry.transformBy(robotToCamera).transformBy(cameraToTag);

					// Record observation
					calibrator.recordObservation(singleTag.tagId(), robotPoseOdometry, tagPose);
				}
			}

			// Process all pose observations from this camera
			// PhotonVision processes the FULL camera feed to detect tags and calculate pose
			// When multiple tags are visible, it uses all of them together for better
			// accuracy
			for (VisionIO.PoseObservation observation : inputs[cameraIndex].poseObservations) {
				robotPoses.add(observation.pose());

				// Enhanced field understanding:
				// - Multitag results use the full camera view and multiple tags for better
				// accuracy
				// - Single tag results use one tag but still process the full camera feed
				// - The camera feed is continuously analyzed for tag detection

				if (shouldRejectPose(observation)) {
					robotPosesRejected.add(observation.pose());
					continue;
				}

				robotPosesAccepted.add(observation.pose());

				// Send pose estimate to drive subsystem
				// This uses the full camera feed analysis from PhotonVision:
				// - Distance measurements from camera to tags
				// - Angle measurements (tx, ty) from camera view
				// - Multiple tag fusion when available (multitag)
				// - All calculated from the full camera feed processing
				consumer.accept(
						observation.pose().toPose2d(),
						observation.timestamp(),
						getLatestVariance(observation, cameraIndex));
			}
			logCameraData(cameraIndex, robotPoses, robotPosesAccepted, robotPosesRejected);

			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

		logSummaryData(allRobotPoses, allRobotPosesAccepted, allRobotPosesRejected);

		// ORGANIZED SUMMARY - One simple log to check
		Logger.recordOutput("Vision/SeesTags", anyCameraSeesTags);
		Logger.recordOutput("Vision/TotalTagsSeen", totalTagsSeen);
	}

	/** sets a VisionConsumer for the vision to send estimates to */
	public void setVisionConsumer(VisionConsumer consumer) {
		this.consumer = consumer;
	}

	/**
	 * Sets the tag field calibrator for custom field layouts.
	 * 
	 * @param calibrator
	 *            Calibrator instance (can be null)
	 */
	public void setCalibrator(TagFieldCalibrator calibrator) {
		this.calibrator = calibrator;
	}

	/**
	 * Updates the field layout (called after calibration completes).
	 */
	public void updateFieldLayout() {
		// Update all cameras with new layout
		for (int i = 0; i < io.length; i++) {
			io[i].setAprilTagLayout(VisionConstants.getActiveLayout());
		}
	}

	/***
	 * checks if a pose measurement should be consumed
	 *
	 * @param observation
	 *            a single observation from a camera
	 * @return true if pose should be rejected due to low tags, high distance, or
	 *         out of field
	 */
	private boolean shouldRejectPose(VisionIO.PoseObservation observation) {
		return observation.tagCount() == 0 // Must have at least one tag
				// single tag
				|| Math.abs(observation.pose().getZ()) > 1.0 // Must have realistic Z coordinate
				|| observation.averageTagDistance() > 10
				|| observation.ambiguity() > 0.16
				// Must be within the field boundaries
				|| observation.pose().getX() < 0.0
				|| observation.pose().getX() > VisionConstants.aprilTagLayout.getFieldLength()
				|| observation.pose().getY() < 0.0
				|| observation.pose().getY() > VisionConstants.aprilTagLayout.getFieldWidth();
		// || Math.abs(drive.getHeading().getDegrees()
		// - Units.radiansToDegrees(observation.pose().getRotation().getAngle())) > 2;
	}

	/**
	 * calculates how much we should rely on this pose when sending it to vision
	 * consumer
	 *
	 * @param observation
	 *            a pose estimate from a camera
	 * @param cameraIndex
	 *            the index of camera providing observation
	 * @return a matrix representing the standard deviation factors
	 */
	private Matrix<N3, N1> getLatestVariance(
			VisionIO.PoseObservation observation, int cameraIndex) {
		double avgDistanceFromTarget = observation.averageTagDistance();
		int numTags = observation.tagCount();

		// Base uncertainty calculation
		double linearStdDev = 0.02
				* Math.pow(avgDistanceFromTarget, 2)
				/ numTags;
		double angularStdDev = 0.06
				* Math.pow(avgDistanceFromTarget, 2)
				/ numTags;

		// Enhanced field understanding from full camera feed:
		// - PhotonVision processes the ENTIRE camera feed frame-by-frame
		// - It detects all visible AprilTags and calculates distances/angles
		// - Multitag results use multiple tags together for better accuracy
		// - This gives the robot full field understanding through camera analysis

		// Increase uncertainty for single tags (they're less reliable)
		// Single tags can have ambiguity issues, so we trust them less
		if (numTags == 1) {
			linearStdDev *= 2.0; // Double uncertainty for single tags
			angularStdDev *= 2.5; // Even more uncertainty for angle with single tags
		} else {
			// Multitag results are more accurate because:
			// - Uses full camera feed with multiple tags visible
			// - PhotonVision fuses all tag positions from the camera view
			// - Better distance/angle measurements from analyzing entire frame
			// - More reliable pose estimation from field understanding
			linearStdDev *= 0.9; // 10% better accuracy with multitag
			angularStdDev *= 0.85; // 15% better angle accuracy with multitag
		}

		// Ensure minimum uncertainty (don't trust vision too much)
		// Increased to prevent aggressive corrections that cause driving issues
		linearStdDev = Math.max(linearStdDev, 0.15); // At least 15cm uncertainty (was 5cm)
		angularStdDev = Math.max(angularStdDev, 0.1); // At least 0.1 rad (~6°) uncertainty (was 3°)

		// Cap maximum uncertainty (don't make it useless)
		linearStdDev = Math.min(linearStdDev, 0.5); // Max 50cm uncertainty
		angularStdDev = Math.min(angularStdDev, 0.3); // Max 0.3 rad (~17°) uncertainty

		return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
	}

	/**
	 * logs individual camera data to advantage kit realOutputs under
	 * Vision/camera/index
	 *
	 * @param cameraIndex
	 *            index of camera to liog
	 * @param robotPoses
	 *            list of all poses found by camera
	 * @param robotPosesAccepted
	 *            list of poses NOT REJECTED by shouldRejectPose
	 * @param robotPosesRejected
	 *            list of poses REJECTED by shouldRejectPose
	 */
	private void logCameraData(
			int cameraIndex,
			List<Pose3d> robotPoses,
			List<Pose3d> robotPosesAccepted,
			List<Pose3d> robotPosesRejected) {
		// Log camera datadata
		Logger.recordOutput(
				"Vision/Camera" + cameraIndex + "/RobotPoses",
				robotPoses.toArray(new Pose3d[robotPoses.size()]));
		Logger.recordOutput(
				"Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
				robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
		Logger.recordOutput(
				"Vision/Camera" + cameraIndex + "/RobotPosesRejected",
				robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
	}

	/**
	 * logs summary data to realOutputs via Vision/Summary/
	 *
	 * @param allRobotPoses
	 *            list of all poses found by all cameras
	 * @param allRobotPosesAccepted
	 *            list of poses NOT REJECTED by shouldRejectPose
	 * @param allRobotPosesRejected
	 *            list of poses REJECTED by shouldRejectPose
	 */
	private void logSummaryData(
			List<Pose3d> allRobotPoses,
			List<Pose3d> allRobotPosesAccepted,
			List<Pose3d> allRobotPosesRejected) {
		Logger.recordOutput(
				"Vision/Summary/RobotPoses",
				allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesAccepted",
				allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesRejected",
				allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
	}

	@FunctionalInterface
	public static interface VisionConsumer {
		public void accept(
				Pose2d visionRobotPoseMeters,
				double timestampSeconds,
				Matrix<N3, N1> visionMeasurementStdDevs);
	}

	public static record DistanceToTag(
			double crossTrackDistance, double alongTrackDistance, boolean isValid) {
	};
}