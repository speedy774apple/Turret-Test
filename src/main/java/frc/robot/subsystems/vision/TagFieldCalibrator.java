package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.SingleTagObservation;

import org.littletonrobotics.junction.Logger;

/**
 * Calibrates a custom AprilTag field layout by observing tags during
 * calibration mode.
 * 
 * How it works:
 * 1. During calibration, robot drives around and cameras see multiple tags
 * 2. System collects observations: robot pose + tag positions relative to robot
 * 3. Calculates absolute tag positions using: tag_absolute = robot_absolute +
 * tag_relative
 * 4. Builds up a map of all tag positions over time
 * 5. Creates custom AprilTagFieldLayout from collected data
 * 6. Uses that layout for localization instead of standard field
 * 
 * This allows the robot to adapt to ANY field configuration with tags placed
 * anywhere!
 */
public class TagFieldCalibrator extends SubsystemBase {
	private final Drive drive;
	private final VisionLocalizer vision;

	// Calibration state
	private boolean isCalibrating = false;
	private Map<Integer, List<TagObservation>> tagObservations = new HashMap<>();

	// Minimum observations needed per tag for reliable calibration
	private static final int MIN_OBSERVATIONS_PER_TAG = 3;

	// Custom field layout (null until calibration complete)
	private AprilTagFieldLayout customLayout = null;

	/**
	 * Creates a new tag field calibrator.
	 * 
	 * @param drive
	 *            Drive subsystem (for robot pose during calibration)
	 * @param vision
	 *            Vision localizer (for tag observations)
	 */
	public TagFieldCalibrator(Drive drive, VisionLocalizer vision) {
		this.drive = drive;
		this.vision = vision;
	}

	/**
	 * Starts calibration mode.
	 * Clears previous observations and begins collecting new data.
	 */
	public void startCalibration() {
		isCalibrating = true;
		tagObservations.clear();
		customLayout = null;
		Logger.recordOutput("Calibration/Active", true);
		Logger.recordOutput("Calibration/TagsFound", 0);
		Logger.recordOutput("Calibration/TotalObservations", 0);
	}

	/**
	 * Stops calibration and builds the custom field layout.
	 * 
	 * @return true if calibration was successful (enough data collected)
	 */
	public boolean finishCalibration() {
		isCalibrating = false;
		Logger.recordOutput("Calibration/Active", false);

		// Build custom layout from observations
		customLayout = buildCustomLayout();

		if (customLayout != null) {
			Logger.recordOutput("Calibration/Success", true);
			Logger.recordOutput("Calibration/TagsCalibrated", customLayout.getTags().size());
			return true;
		} else {
			Logger.recordOutput("Calibration/Success", false);
			Logger.recordOutput("Calibration/Error", "Not enough observations collected");
			return false;
		}
	}

	/**
	 * Periodic update - collects tag observations during calibration.
	 * This is called by VisionLocalizer when it processes vision data.
	 */
	@Override
	public void periodic() {
		if (!isCalibrating) {
			return;
		}

		// Get current robot pose from odometry (more reliable than vision during
		// calibration)
		Pose3d robotPose3d = new Pose3d(drive.getPose());

		// Count total observations
		int totalObservations = 0;
		for (List<TagObservation> observations : tagObservations.values()) {
			totalObservations += observations.size();
		}

		Logger.recordOutput("Calibration/TagsFound", tagObservations.size());
		Logger.recordOutput("Calibration/TotalObservations", totalObservations);
		Logger.recordOutput("Calibration/RobotPose", robotPose3d.toPose2d());
	}

	/**
	 * Records a tag observation from vision system.
	 * Called by VisionLocalizer when it detects tags during calibration.
	 * 
	 * @param tagId
	 *            AprilTag ID
	 * @param robotPose
	 *            Current robot pose (from odometry)
	 * @param tagPose
	 *            Estimated tag pose (from vision)
	 */
	public void recordObservation(int tagId, Pose3d robotPose, Pose3d tagPose) {
		if (!isCalibrating) {
			return;
		}

		// Store observation
		tagObservations.computeIfAbsent(tagId, k -> new ArrayList<>())
				.add(new TagObservation(tagPose, robotPose));

		Logger.recordOutput("Calibration/Tag" + tagId + "/Observations",
				tagObservations.get(tagId).size());
		Logger.recordOutput("Calibration/Tag" + tagId + "/LatestPose", tagPose);
	}

	/**
	 * Builds a custom AprilTagFieldLayout from collected observations.
	 * Uses average position of each tag from multiple observations.
	 * 
	 * @return Custom field layout, or null if not enough data
	 */
	private AprilTagFieldLayout buildCustomLayout() {
		List<AprilTag> tags = new ArrayList<>();

		for (Map.Entry<Integer, List<TagObservation>> entry : tagObservations.entrySet()) {
			int tagId = entry.getKey();
			List<TagObservation> observations = entry.getValue();

			if (observations.size() < MIN_OBSERVATIONS_PER_TAG) {
				Logger.recordOutput("Calibration/Tag" + tagId + "/Warning",
						"Not enough observations: " + observations.size());
				continue; // Skip tags with insufficient data
			}

			// Calculate average position from all observations
			double avgX = 0, avgY = 0, avgZ = 0;
			double avgRoll = 0, avgPitch = 0, avgYaw = 0;

			for (TagObservation obs : observations) {
				Pose3d tagPose = obs.tagPose();
				avgX += tagPose.getX();
				avgY += tagPose.getY();
				avgZ += tagPose.getZ();

				Rotation3d rot = tagPose.getRotation();
				avgRoll += rot.getX();
				avgPitch += rot.getY();
				avgYaw += rot.getZ();
			}

			int count = observations.size();
			Pose3d averagePose = new Pose3d(
					avgX / count,
					avgY / count,
					avgZ / count,
					new Rotation3d(avgRoll / count, avgPitch / count, avgYaw / count));

			// Create AprilTag with average pose
			tags.add(new AprilTag(tagId, averagePose));

			Logger.recordOutput("Calibration/Tag" + tagId + "/FinalPose", averagePose);
		}

		if (tags.isEmpty()) {
			return null;
		}

		// Create field layout with game-specific field dimensions
		// Uses GameConstants (game-specific, see GameConstants.java)
		double fieldLength = frc.robot.util.GameConstants.FIELD_LENGTH;
		double fieldWidth = frc.robot.util.GameConstants.FIELD_WIDTH;

		return new AprilTagFieldLayout(tags, fieldLength, fieldWidth);
	}

	/**
	 * Gets the custom field layout if calibration is complete.
	 * 
	 * @return Custom layout, or null if not calibrated yet
	 */
	public Optional<AprilTagFieldLayout> getCustomLayout() {
		return Optional.ofNullable(customLayout);
	}

	/**
	 * Checks if calibration is currently active.
	 * 
	 * @return true if calibrating
	 */
	public boolean isCalibrating() {
		return isCalibrating;
	}

	/**
	 * Checks if a custom layout is available.
	 * 
	 * @return true if custom layout exists
	 */
	public boolean hasCustomLayout() {
		return customLayout != null;
	}

	/**
	 * Resets calibration (clears custom layout).
	 */
	public void reset() {
		isCalibrating = false;
		tagObservations.clear();
		customLayout = null;
		Logger.recordOutput("Calibration/Active", false);
		Logger.recordOutput("Calibration/Success", false);
	}

	/**
	 * Record for storing tag observations during calibration.
	 */
	private static record TagObservation(Pose3d tagPose, Pose3d robotPose) {
	}
}
