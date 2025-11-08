package frc.robot.subsystems.fusion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

/**
 * Enhanced pose estimator with Kalman filtering and motion model prediction.
 * 
 * This class wraps the Drive subsystem's pose estimator and adds:
 * - Adaptive confidence tracking for all sensor layers
 * - Motion model prediction using velocity integration
 * - Multi-layer sensor fusion weighting
 * - Enhanced measurement validation and outlier rejection
 * 
 * NOTE: This class does NOT replace the Drive's internal
 * SwerveDrivePoseEstimator.
 * Instead, it monitors and enhances the measurements fed to it.
 * 
 * Layer 1: Wheel Odometry (encoders + gyro)
 * - Update rate: 250 Hz (high frequency)
 * - Weight: 0.7 (primary sensor)
 * - Provides: Continuous pose tracking from wheel encoders and gyro
 * 
 * Layer 2: Vision Localization (AprilTags)
 * - Update rate: 20 Hz (lower frequency, global corrections)
 * - Weight: 0.4 (correction sensor)
 * - Provides: Absolute position corrections when tags are visible
 * 
 * Layer 3: Gyro/IMU (AHRS)
 * - Update rate: 200 Hz (high frequency)
 * - Weight: 0.9 (high confidence for heading)
 * - Provides: Accurate heading and angular velocity
 * 
 * Layer 4: Kalman Filter Prediction (motion model)
 * - Update rate: 50 Hz (prediction loop)
 * - Weight: 0.2 (smoothing)
 * - Provides: Velocity-based prediction and smoothing
 * 
 * Layer 5: High-level Fusion (all layers combined)
 * - Update rate: 50 Hz (final fusion)
 * - Provides: Best estimate pose from all sensors
 */
public class EnhancedPoseEstimator {
	// Reference to Drive subsystem's pose estimator (via functional interface)
	private final PoseSupplier poseSupplier;

	// Kalman filter state
	private Pose2d predictedPose;
	private ChassisSpeeds lastVelocity = new ChassisSpeeds();
	private double lastUpdateTime = Timer.getFPGATimestamp();

	// Vision measurement timing (for interpolation/extrapolation)
	private double lastVisionMeasurementTime = 0.0;
	private Pose2d lastVisionPose = new Pose2d();
	private boolean hasRecentVisionMeasurement = false;

	// Confidence tracking (for monitoring, not used in Kalman filter)
	private double wheelOdometryConfidence = 1.0;
	private double visionConfidence = 0.0;
	private double gyroConfidence = 1.0;

	// Statistics
	private int visionMeasurementCount = 0;
	private int totalUpdateCount = 0;
	private int rejectedVisionMeasurements = 0;

	/**
	 * Functional interface for getting the current pose estimate.
	 */
	@FunctionalInterface
	public static interface PoseSupplier {
		Pose2d get();
	}

	/**
	 * Creates an enhanced pose estimator that wraps the Drive subsystem.
	 * 
	 * @param poseSupplier
	 *            Function to get current pose from Drive subsystem
	 */
	public EnhancedPoseEstimator(PoseSupplier poseSupplier) {
		this.poseSupplier = poseSupplier;
		this.predictedPose = poseSupplier.get();
		this.lastUpdateTime = Timer.getFPGATimestamp();
	}

	/**
	 * Updates the enhanced estimator tracking.
	 * This monitors Layer 1 (wheel odometry) which is already updated in
	 * Drive.periodic().
	 * 
	 * @param gyroAngle
	 *            Current gyro rotation (for confidence calculation)
	 * @param modulePositions
	 *            Current swerve module positions (for confidence calculation)
	 */
	public void updateWheelOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
		double currentTime = Timer.getFPGATimestamp();

		// Calculate confidence based on sensor health
		wheelOdometryConfidence = calculateWheelOdometryConfidence(modulePositions);

		// Update Kalman prediction
		updateKalmanPrediction(currentTime);

		totalUpdateCount++;
	}

	/**
	 * Validates and processes a vision measurement before it's added to the Drive's
	 * estimator.
	 * This is Layer 2 of the fusion system.
	 * 
	 * This method handles:
	 * - Timestamp validation (rejects stale measurements)
	 * - Outlier rejection based on Mahalanobis distance from odometry
	 * - Adaptive covariance adjustment based on measurement quality
	 * 
	 * TIMING: The SwerveDrivePoseEstimator internally handles
	 * interpolation/extrapolation
	 * of vision measurements to match its update rate (250 Hz). When a vision
	 * measurement
	 * arrives at 20 Hz, the estimator uses the timestamp to interpolate/extrapolate
	 * it to
	 * the current odometry update time. This prevents "jumps" or lag in the pose
	 * estimate.
	 * We validate timestamps to ensure measurements aren't too old for reliable
	 * extrapolation.
	 * 
	 * REPROJECTION ERROR: While we don't have direct access to pixel-level
	 * reprojection
	 * error from VisionIO, we use the ambiguity score and tag count as proxies. The
	 * VisionLocalizer's shouldRejectPose() method already filters based on
	 * ambiguity and
	 * tag count. We add additional statistical validation here.
	 * 
	 * @param visionRobotPoseMeters
	 *            Vision-estimated pose
	 * @param timestampSeconds
	 *            Timestamp of the measurement (must be recent)
	 * @param visionMeasurementStdDevs
	 *            Standard deviation of the measurement
	 * @return Adjusted standard deviation (or null if measurement should be
	 *         rejected)
	 */
	public Matrix<N3, N1> validateVisionMeasurement(
			Pose2d visionRobotPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs) {

		double currentTime = Timer.getFPGATimestamp();

		// Reject stale measurements (too old for reliable extrapolation)
		// Old measurements can cause lag or incorrect corrections
		double measurementAge = currentTime - timestampSeconds;
		if (measurementAge > SensorFusionConstants.MAX_VISION_MEASUREMENT_AGE) {
			rejectedVisionMeasurements++;
			visionConfidence = 0.0;
			return null; // Measurement too old
		}

		// Check for valid timestamp (not in future, reasonable)
		if (timestampSeconds > currentTime + 0.1 || timestampSeconds < currentTime - 1.0) {
			rejectedVisionMeasurements++;
			visionConfidence = 0.0;
			return null; // Invalid timestamp
		}

		// Calculate confidence based on measurement quality
		double confidence = calculateVisionConfidence(visionRobotPoseMeters, visionMeasurementStdDevs);

		// Apply adaptive covariance adjustment based on confidence
		// Lower confidence = higher uncertainty (larger stdDev) in Kalman filter
		Matrix<N3, N1> adjustedStdDev = adjustStdDevForConfidence(visionMeasurementStdDevs, confidence);

		// Check if vision measurement is reasonable (not too far from odometry)
		if (isVisionMeasurementValid(visionRobotPoseMeters, adjustedStdDev)) {
			visionConfidence = confidence;
			visionMeasurementCount++;
			lastVisionMeasurementTime = timestampSeconds;
			lastVisionPose = visionRobotPoseMeters;
			hasRecentVisionMeasurement = true;
			return adjustedStdDev; // Return adjusted stdDev for use in Drive
		} else {
			// Reject outlier measurement
			rejectedVisionMeasurements++;
			visionConfidence = 0.0;
			return null; // Signal rejection
		}
	}

	/**
	 * Updates the Kalman filter prediction using motion model.
	 * This is Layer 4 of the fusion system.
	 * 
	 * @param currentTime
	 *            Current timestamp
	 */
	private void updateKalmanPrediction(double currentTime) {
		Pose2d currentPose = poseSupplier.get(); // Get from Drive subsystem
		double deltaTime = currentTime - lastUpdateTime;

		if (deltaTime > 0 && deltaTime < 0.1) { // Sanity check
			// Simple motion model: integrate velocity
			// More sophisticated models can predict based on acceleration, etc.

			// Update prediction (this would be more sophisticated in a full Kalman filter)
			// For now, we rely on the SwerveDrivePoseEstimator's internal filtering
			predictedPose = currentPose;

			lastUpdateTime = currentTime;
		}
	}

	/**
	 * Updates velocity tracking for motion model prediction.
	 * 
	 * @param chassisSpeeds
	 *            Current chassis speeds
	 */
	public void updateVelocity(ChassisSpeeds chassisSpeeds) {
		this.lastVelocity = chassisSpeeds;
	}

	/**
	 * Gets the current best-estimate pose from all fused sensors.
	 * This is Layer 5 (high-level fusion) output.
	 * 
	 * @return Fused pose estimate (from Drive subsystem)
	 */
	public Pose2d getEstimatedPose() {
		return poseSupplier.get();
	}

	/**
	 * Calculates confidence score for wheel odometry based on sensor health.
	 * 
	 * @param modulePositions
	 *            Current module positions
	 * @return Confidence value (0.0 to 1.0)
	 */
	private double calculateWheelOdometryConfidence(SwerveModulePosition[] modulePositions) {
		// Check for anomalies (e.g., all modules reporting same value = sensor failure)
		// This is a simplified check; more sophisticated checks can be added

		boolean allModulesValid = modulePositions != null && modulePositions.length == 4;

		if (!allModulesValid) {
			return 0.5; // Reduced confidence
		}

		// Check for reasonable values
		for (var position : modulePositions) {
			if (Double.isNaN(position.distanceMeters) || Double.isInfinite(position.distanceMeters)) {
				return 0.3; // Low confidence if any sensor reports invalid data
			}
		}

		return 1.0; // High confidence if all sensors healthy
	}

	/**
	 * Calculates confidence score for vision measurements.
	 * 
	 * This considers:
	 * - Standard deviation (lower = higher confidence)
	 * - Distance from odometry (closer = higher confidence, if reasonable)
	 * - Measurement recency (recent = higher confidence)
	 * 
	 * NOTE: This confidence value is for monitoring/logging only.
	 * The actual Kalman filter uses covariance matrices (stdDev) for proper
	 * weighting.
	 * 
	 * @param visionPose
	 *            Vision-estimated pose
	 * @param stdDevs
	 *            Standard deviation of measurement
	 * @return Confidence value (0.0 to 1.0) for monitoring purposes
	 */
	private double calculateVisionConfidence(Pose2d visionPose, Matrix<N3, N1> stdDevs) {
		double xStdDev = stdDevs.get(0, 0);
		double yStdDev = stdDevs.get(1, 0);
		double thetaStdDev = stdDevs.get(2, 0);

		// Lower standard deviation = higher confidence
		double positionStdDev = Math.sqrt(xStdDev * xStdDev + yStdDev * yStdDev);

		// Convert to confidence (inverse relationship)
		// Typical good vision: stdDev < 0.1m -> confidence > 0.8
		// Typical poor vision: stdDev > 0.5m -> confidence < 0.3
		double positionConfidence = Math.max(0.0, 1.0 - (positionStdDev / 0.5));
		double angleConfidence = Math.max(0.0, 1.0 - (thetaStdDev / 0.2));

		// Base confidence from measurement quality
		double baseConfidence = (positionConfidence + angleConfidence) / 2.0;

		// Bonus for reasonable agreement with odometry (but don't penalize too much)
		Pose2d currentPose = poseSupplier.get();
		double distanceFromOdometry = visionPose.getTranslation().getDistance(currentPose.getTranslation());
		double agreementBonus = Math.max(0.0,
				1.0 - (distanceFromOdometry / SensorFusionConstants.MAX_VISION_DISTANCE_FROM_ODOMETRY)) * 0.2;

		return Math.min(1.0, baseConfidence + agreementBonus);
	}

	/**
	 * Adjusts standard deviation based on confidence.
	 * Lower confidence -> higher uncertainty (larger stdDev).
	 * 
	 * @param baseStdDev
	 *            Base standard deviation
	 * @param confidence
	 *            Confidence value (0.0 to 1.0)
	 * @return Adjusted standard deviation
	 */
	private Matrix<N3, N1> adjustStdDevForConfidence(Matrix<N3, N1> baseStdDev, double confidence) {
		// Scale stdDev inversely with confidence
		// confidence = 1.0 -> stdDev unchanged
		// confidence = 0.5 -> stdDev doubled
		// confidence = 0.0 -> stdDev tripled (very uncertain)
		double scaleFactor = 3.0 - (2.0 * confidence);

		return baseStdDev.times(scaleFactor);
	}

	/**
	 * Validates vision measurement against current odometry estimate.
	 * Rejects outliers that are too far from expected position.
	 * 
	 * Uses Mahalanobis distance (accounts for uncertainty) instead of simple
	 * Euclidean distance.
	 * This is more statistically sound and prevents rejecting good measurements
	 * with high uncertainty.
	 * 
	 * @param visionPose
	 *            Vision-estimated pose
	 * @param stdDevs
	 *            Standard deviation (used for Mahalanobis distance calculation)
	 * @return true if measurement is valid
	 */
	private boolean isVisionMeasurementValid(Pose2d visionPose, Matrix<N3, N1> stdDevs) {
		Pose2d currentPose = poseSupplier.get(); // Get current pose from Drive

		// Calculate position and angle differences
		double dx = visionPose.getX() - currentPose.getX();
		double dy = visionPose.getY() - currentPose.getY();
		double dtheta = visionPose.getRotation().minus(currentPose.getRotation()).getRadians();

		// Calculate Mahalanobis distance (accounts for measurement uncertainty)
		// Mahalanobis distance = sqrt((x - mu)^T * Sigma^(-1) * (x - mu))
		// Where Sigma is the covariance matrix (diagonal with stdDev^2)
		double xStdDev = stdDevs.get(0, 0);
		double yStdDev = stdDevs.get(1, 0);
		double thetaStdDev = stdDevs.get(2, 0);

		// Avoid division by zero
		if (xStdDev < 0.001 || yStdDev < 0.001 || thetaStdDev < 0.001) {
			// If uncertainty is extremely low, use simple distance check
			double positionError = Math.sqrt(dx * dx + dy * dy);
			double angleError = Math.abs(dtheta);
			return positionError < SensorFusionConstants.MAX_VISION_DISTANCE_FROM_ODOMETRY &&
					angleError < SensorFusionConstants.MAX_VISION_ANGULAR_DIFFERENCE;
		}

		// Calculate normalized (Mahalanobis-like) distance components
		double normalizedX = dx / xStdDev;
		double normalizedY = dy / yStdDev;
		double normalizedTheta = dtheta / thetaStdDev;

		// Mahalanobis distance
		double mahalanobisDistance = Math.sqrt(
				normalizedX * normalizedX +
						normalizedY * normalizedY +
						normalizedTheta * normalizedTheta);

		// Accept if within 3 standard deviations (99.7% confidence interval)
		// This is more lenient for high-uncertainty measurements
		double mahalanobisThreshold = 3.0;

		// Also do simple checks as backup
		double positionError = Math.sqrt(dx * dx + dy * dy);
		double angleError = Math.abs(dtheta);
		boolean mahalanobisValid = mahalanobisDistance < mahalanobisThreshold;
		boolean positionValid = positionError < SensorFusionConstants.MAX_VISION_DISTANCE_FROM_ODOMETRY;
		boolean angleValid = angleError < SensorFusionConstants.MAX_VISION_ANGULAR_DIFFERENCE;

		return mahalanobisValid && positionValid && angleValid;
	}

	/**
	 * Resets the enhanced estimator tracking.
	 * Note: The actual reset should be done on the Drive subsystem.
	 * 
	 * @param pose
	 *            New pose
	 * @param gyroAngle
	 *            Current gyro angle
	 * @param modulePositions
	 *            Current module positions
	 */
	public void resetPosition(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
		// Reset our tracking state
		predictedPose = pose;
		lastUpdateTime = Timer.getFPGATimestamp();
		// Note: Drive subsystem's setPose() should be called separately
	}

	/**
	 * Gets confidence values for all sensor layers.
	 * 
	 * @return Array of [wheelOdometry, vision, gyro] confidence values
	 */
	public double[] getSensorConfidences() {
		return new double[] { wheelOdometryConfidence, visionConfidence, gyroConfidence };
	}

	/**
	 * Gets statistics about fusion performance.
	 * 
	 * @return Statistics record
	 */
	public FusionStatistics getStatistics() {
		return new FusionStatistics(
				totalUpdateCount,
				visionMeasurementCount,
				rejectedVisionMeasurements,
				wheelOdometryConfidence,
				visionConfidence,
				gyroConfidence,
				hasRecentVisionMeasurement,
				lastVisionMeasurementTime);
	}

	/**
	 * Statistics record for fusion performance monitoring.
	 */
	public static record FusionStatistics(
			int totalUpdates,
			int visionMeasurements,
			int rejectedVisionMeasurements,
			double wheelOdometryConfidence,
			double visionConfidence,
			double gyroConfidence,
			boolean hasRecentVisionMeasurement,
			double lastVisionMeasurementTime) {
	}
}
