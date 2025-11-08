package frc.robot.subsystems.fusion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.fusion.objectdetection.DetectionStatus;
import frc.robot.subsystems.fusion.objectdetection.ObjectDetectionSystem;
import frc.robot.subsystems.fusion.sensors.BeamBreakSensor;
import frc.robot.subsystems.vision.VisionLocalizer;

import org.littletonrobotics.junction.Logger;

/**
 * High-level sensor fusion manager that integrates all sensor layers.
 * 
 * This is the main interface for the complete sensor fusion system.
 * It coordinates:
 * - Enhanced pose estimation with Kalman filtering
 * - Vision localization integration
 * - Object detection (game pieces and targets)
 * - Real-time pose updates for autonomous and teleop
 * 
 * All processing is done offline - no internet connection required.
 * 
 * SENSOR FUSION LAYERS:
 * 
 * Layer 1: Wheel Odometry (encoders + gyro)
 * - Update rate: 250 Hz
 * - Weight: 0.7 (primary sensor)
 * - Input: Swerve module encoders + gyro angle
 * - Output: Continuous pose estimate from wheel movement
 * 
 * Layer 2: Vision Localization (AprilTags)
 * - Update rate: 20 Hz
 * - Weight: 0.4 (correction sensor)
 * - Input: Camera AprilTag detections
 * - Output: Global position corrections
 * 
 * Layer 3: Swerve Module Absolute Encoders
 * - Update rate: 250 Hz
 * - Weight: Integrated into Layer 1
 * - Input: CANcoder absolute positions
 * - Output: Accurate module angle measurements
 * 
 * Layer 4: Gyro + Accelerometer (AHRS)
 * - Update rate: 200 Hz
 * - Weight: 0.9 for heading
 * - Input: Pigeon 2 IMU data
 * - Output: Heading and angular velocity
 * 
 * Layer 5: Velocity & Motion Model Prediction (Kalman Filter)
 * - Update rate: 50 Hz
 * - Weight: 0.2 (smoothing)
 * - Input: Chassis velocities
 * - Output: Velocity-based pose predictions and smoothing
 * 
 * Layer 6: High-Level Fusion (Statistical/Probabilistic Fusion)
 * - Update rate: 50 Hz
 * - Weighting: Dynamic based on measurement uncertainty (covariance matrices)
 * - Method: Kalman filter-based fusion using proper statistical weighting
 * - Input: All above layers (already fused by SwerveDrivePoseEstimator)
 * - Output: Best-estimate fused pose
 * 
 * NOTE: "AI Fusion" refers to intelligent/smart fusion using statistical
 * methods
 * (Kalman filtering), not machine learning. The fusion intelligently weights
 * sensors
 * based on their uncertainty (covariance), adapting to sensor quality
 * dynamically.
 * All processing is fully offline with no external dependencies.
 * 
 * OBJECT DETECTION LAYERS:
 * 
 * Layer 1: Limelight/Camera Vision
 * - Update rate: 30 Hz
 * - Input: Camera images (processed by Limelight on-robot, offline)
 * - Output: Game piece and target detection
 * 
 * Layer 2: Beam Break Sensors
 * - Update rate: Polled at fusion loop rate (~50 Hz)
 * - Note: Hardware responds quickly, but polling at 1000+ Hz is unnecessary
 * - Input: Digital sensor readings (polled, not interrupt-driven)
 * - Output: Intake confirmation
 */
public class SensorFusionManager extends SubsystemBase {
	private final Drive drive;
	private final VisionLocalizer vision;
	private final EnhancedPoseEstimator poseEstimator;
	private final ObjectDetectionSystem objectDetection;

	// Update timing
	private double lastFusionUpdateTime = 0.0;
	private double lastObjectDetectionUpdateTime = 0.0;
	private double lastDetailedLogTime = 0.0;

	// Current fused outputs
	private Pose2d fusedPose = new Pose2d();
	private DetectionStatus detectionStatus;

	/**
	 * Creates a new sensor fusion manager.
	 * 
	 * @param drive
	 *            Drive subsystem (provides odometry and module data)
	 * @param vision
	 *            Vision localizer (provides AprilTag corrections)
	 * @param poseEstimator
	 *            Enhanced pose estimator with Kalman filtering
	 * @param objectDetection
	 *            Object detection system
	 */
	public SensorFusionManager(
			Drive drive,
			VisionLocalizer vision,
			EnhancedPoseEstimator poseEstimator,
			ObjectDetectionSystem objectDetection) {
		this.drive = drive;
		this.vision = vision;
		this.poseEstimator = poseEstimator;
		this.objectDetection = objectDetection;
	}

	/**
	 * Periodic update method.
	 * Should be called every robot loop cycle (50 Hz).
	 * 
	 * This method handles timing to ensure each sensor layer updates at its
	 * appropriate rate, with proper interpolation/extrapolation for slower sensors.
	 */
	@Override
	public void periodic() {
		double currentTime = Timer.getFPGATimestamp();

		// Update fusion at target rate (50 Hz)
		// The Drive subsystem already handles high-frequency odometry updates
		// internally.
		// We sync vision measurements (20 Hz) and other sensors to this rate.
		double fusionDeltaTime = currentTime - lastFusionUpdateTime;
		if (fusionDeltaTime >= (1.0 / SensorFusionConstants.FUSION_UPDATE_RATE)) {
			updateFusion();
			lastFusionUpdateTime = currentTime;
		}

		// Update object detection at target rate (30 Hz)
		// Beam break sensor is polled here, not at 1000+ Hz
		double detectionDeltaTime = currentTime - lastObjectDetectionUpdateTime;
		if (detectionDeltaTime >= (1.0 / SensorFusionConstants.OBJECT_DETECTION_UPDATE_RATE)) {
			updateObjectDetection();
			lastObjectDetectionUpdateTime = currentTime;
		}

		// Log fusion outputs (rate-limited to avoid network congestion)
		logFusionOutputs(currentTime);
	}

	/**
	 * Updates the high-level fusion (Layer 6).
	 * Integrates all sensor layers and produces best-estimate pose.
	 * 
	 * TIMING HANDLING:
	 * - Wheel odometry updates at 250 Hz internally in Drive.periodic()
	 * - Vision measurements arrive asynchronously at ~20 Hz
	 * - SwerveDrivePoseEstimator internally handles interpolation/extrapolation
	 * of vision measurements to match its high update rate
	 * - This fusion loop runs at 50 Hz and reads the already-fused result
	 * 
	 * FUSION METHOD:
	 * - Uses SwerveDrivePoseEstimator (Kalman filter-based)
	 * - Properly handles covariances (not arbitrary weights)
	 * - Statistically optimal fusion based on measurement uncertainty
	 */
	private void updateFusion() {
		// Layer 1: Wheel Odometry (already updated in Drive.periodic() at 250 Hz)
		// The Drive subsystem's internal SwerveDrivePoseEstimator is updated
		// continuously.
		// We track the gyro angle for our enhanced estimator confidence calculation.
		Rotation2d gyroAngle = drive.getHeading();

		// Update enhanced estimator tracking (monitors Layer 1 for confidence)
		// Note: Module positions are accessed internally by Drive. We pass empty array
		// since the actual tracking happens in Drive.periodic(), but this allows
		// confidence calculation.
		SwerveModulePosition[] modulePositions = getModulePositions();
		poseEstimator.updateWheelOdometry(gyroAngle, modulePositions);

		// Layer 2: Vision Localization
		// Vision measurements are added asynchronously by VisionLocalizer
		// via drive::addVisionMeasurement. The SwerveDrivePoseEstimator internally
		// handles timestamp-based interpolation/extrapolation, so vision measurements
		// at 20 Hz are properly synchronized with the 250 Hz odometry updates.
		// Our validateVisionMeasurement() method ensures only good measurements are
		// used.

		// Layer 3: Absolute Encoders
		// Already integrated into Layer 1 via Drive's module position tracking
		// (CANcoder absolute encoders are used in SwerveModulePosition calculations)

		// Layer 4: Gyro + Accelerometer (AHRS)
		// Already integrated into Layer 1 via Drive's gyro integration
		// (gyroAngle comes from Pigeon 2 IMU, which internally fuses gyro + accel)
		// Note: Pigeon 2 uses internal AHRS fusion algorithms for stable heading

		// Layer 5: Motion Model Prediction
		// Updated via velocity tracking for motion model smoothing
		ChassisSpeeds chassisSpeeds = drive.getChassisSpeeds();
		poseEstimator.updateVelocity(chassisSpeeds);

		// Layer 6: Get final fused pose from Drive subsystem
		// The Drive's SwerveDrivePoseEstimator already performs optimal Kalman filter
		// fusion of all layers. It uses proper covariance-based weighting (not
		// arbitrary
		// weights), ensuring statistically optimal pose estimation.
		// The enhanced estimator wraps this to add confidence tracking and validation.
		fusedPose = poseEstimator.getEstimatedPose();
	}

	/**
	 * Updates object detection system.
	 */
	private void updateObjectDetection() {
		if (objectDetection != null) {
			objectDetection.update();
			detectionStatus = objectDetection.getDetectionStatus();
		}
	}

	/**
	 * Gets module positions from drive subsystem.
	 * Note: The Drive subsystem's pose estimator already tracks module positions
	 * internally, so we access the current pose estimate directly.
	 * 
	 * @return Array of swerve module positions (placeholder - not used for updates)
	 */
	private SwerveModulePosition[] getModulePositions() {
		// The Drive subsystem's pose estimator is already updated with module positions
		// in Drive.periodic(). We don't need to extract them here since the enhanced
		// estimator wraps the Drive's internal estimator. For reset operations, we'll
		// use the Drive's internal state.
		return new SwerveModulePosition[] {
				new SwerveModulePosition(),
				new SwerveModulePosition(),
				new SwerveModulePosition(),
				new SwerveModulePosition()
		};
	}

	/**
	 * Logs fusion outputs for debugging and telemetry.
	 * Uses rate limiting to prevent excessive network traffic.
	 * 
	 * @param currentTime
	 *            Current timestamp for rate limiting
	 */
	private void logFusionOutputs(double currentTime) {
		// Always log pose (critical data, but at limited rate)
		Logger.recordOutput("Fusion/FusedPose", fusedPose);

		// Detailed logs at reduced rate (every 100ms instead of every 20ms)
		boolean shouldLogDetails = (currentTime - lastDetailedLogTime) >= SensorFusionConstants.DETAILED_LOG_INTERVAL;

		if (shouldLogDetails) {
			// Log sensor confidences (for monitoring)
			double[] confidences = poseEstimator.getSensorConfidences();
			Logger.recordOutput("Fusion/WheelOdometryConfidence", confidences[0]);
			Logger.recordOutput("Fusion/VisionConfidence", confidences[1]);
			Logger.recordOutput("Fusion/GyroConfidence", confidences[2]);

			// Log fusion statistics
			var stats = poseEstimator.getStatistics();
			Logger.recordOutput("Fusion/TotalUpdates", stats.totalUpdates());
			Logger.recordOutput("Fusion/VisionMeasurements", stats.visionMeasurements());
			Logger.recordOutput("Fusion/RejectedVisionMeasurements", stats.rejectedVisionMeasurements());
			Logger.recordOutput("Fusion/HasRecentVision", stats.hasRecentVisionMeasurement());

			// Log object detection status
			if (detectionStatus != null) {
				Logger.recordOutput("Detection/HasGamePiece", detectionStatus.hasGamePiece());
				Logger.recordOutput("Detection/HasTarget", detectionStatus.hasTarget());
				Logger.recordOutput("Detection/TargetDistance", detectionStatus.targetDistance());
				Logger.recordOutput("Detection/Confidence", detectionStatus.confidence());
			}

			lastDetailedLogTime = currentTime;
		}
	}

	/**
	 * Gets the current best-estimate fused pose.
	 * This is the primary output of the sensor fusion system.
	 * 
	 * @return Fused pose estimate
	 */
	public Pose2d getFusedPose() {
		return fusedPose;
	}

	/**
	 * Gets the current object detection status.
	 * 
	 * @return DetectionStatus object with all detection information (null if not
	 *         available)
	 */
	public DetectionStatus getDetectionStatus() {
		return detectionStatus;
	}

	/**
	 * Resets the pose estimator to a new pose.
	 * Useful for field initialization or manual resets.
	 * 
	 * @param pose
	 *            New pose
	 */
	public void resetPose(Pose2d pose) {
		// Reset the drive subsystem's pose (which contains the base estimator)
		drive.setPose(pose);

		// Reset enhanced estimator
		Rotation2d gyroAngle = drive.getHeading();
		SwerveModulePosition[] modulePositions = getModulePositions();
		poseEstimator.resetPosition(pose, gyroAngle, modulePositions);

		fusedPose = pose;
	}

	/**
	 * Gets fusion statistics for monitoring and tuning.
	 * 
	 * @return FusionStatistics object
	 */
	public EnhancedPoseEstimator.FusionStatistics getFusionStatistics() {
		return poseEstimator.getStatistics();
	}
}
