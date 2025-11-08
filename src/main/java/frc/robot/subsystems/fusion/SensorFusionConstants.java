package frc.robot.subsystems.fusion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Constants for the sensor fusion system.
 * 
 * This class defines update rates, weighting factors, and standard deviations
 * for all sensor fusion layers.
 */
public final class SensorFusionConstants {

	// ============================================
	// UPDATE RATES (Hz)
	// ============================================
	/**
	 * Update rate for wheel odometry layer (typically 50-250 Hz)
	 * This is the rate at which Drive.periodic() updates odometry internally.
	 * Actual rate may be lower on non-CAN-FD buses (100 Hz) or higher with CAN-FD
	 * (250 Hz).
	 */
	public static final double WHEEL_ODOMETRY_UPDATE_RATE = 250.0;

	/**
	 * Update rate for vision localization layer (typically 10-30 Hz)
	 * Vision measurements arrive asynchronously from cameras.
	 * The fusion system interpolates/extrapolates these to match the fusion loop
	 * rate.
	 */
	public static final double VISION_UPDATE_RATE = 20.0;

	/**
	 * Update rate for gyro/IMU layer (typically 50-200 Hz)
	 * Gyro data is read continuously and integrated into wheel odometry.
	 */
	public static final double GYRO_UPDATE_RATE = 200.0;

	/**
	 * Update rate for Kalman filter predictions (typically 50 Hz)
	 * This matches the main fusion loop rate for consistent predictions.
	 */
	public static final double KALMAN_UPDATE_RATE = 50.0;

	/**
	 * Update rate for object detection (typically 10-30 Hz)
	 * Limited by camera processing speed. Higher rates possible with faster
	 * cameras.
	 */
	public static final double OBJECT_DETECTION_UPDATE_RATE = 30.0;

	/**
	 * Main fusion loop update rate (typically 50 Hz)
	 * All sensor data is synchronized to this rate using
	 * interpolation/extrapolation.
	 * Slower sensors (like vision at 20 Hz) are interpolated to this rate.
	 */
	public static final double FUSION_UPDATE_RATE = 50.0;

	/**
	 * Maximum age for vision measurements before rejection (seconds)
	 * Older measurements are rejected to prevent using stale data.
	 */
	public static final double MAX_VISION_MEASUREMENT_AGE = 0.1; // 100ms max age

	// ============================================
	// SENSOR CONFIDENCE FACTORS (for monitoring only)
	// ============================================
	/**
	 * NOTE: These are NOT used directly in Kalman filter fusion!
	 * Kalman filters use covariance matrices (standard deviations) for weighting,
	 * not arbitrary weight values. These values are for documentation/monitoring
	 * only.
	 * 
	 * Relative importance indicators (conceptual):
	 * - Wheel Odometry: Primary sensor (0.7 = high importance)
	 * - Vision: Correction sensor (0.4 = moderate importance)
	 * - Gyro: Heading reference (0.9 = very high for heading)
	 * 
	 * Actual fusion uses covariance matrices defined below, which properly handle
	 * uncertainty propagation according to Kalman filter theory.
	 */
	public static final double WHEEL_ODOMETRY_CONFIDENCE = 0.7; // Conceptual only
	public static final double VISION_CONFIDENCE = 0.4; // Conceptual only
	public static final double GYRO_CONFIDENCE = 0.9; // Conceptual only

	// ============================================
	// STANDARD DEVIATIONS FOR MEASUREMENT NOISE
	// ============================================
	/**
	 * Standard deviation for wheel odometry measurements [x, y, theta] (meters,
	 * meters, radians)
	 * Lower values = higher confidence
	 * Typical: [0.01-0.05, 0.01-0.05, 0.01-0.05] for well-calibrated swerve
	 */
	public static final Matrix<N3, N1> WHEEL_ODOMETRY_STD_DEV = VecBuilder.fill(0.03, 0.03, 0.01);

	/**
	 * Standard deviation for vision measurements [x, y, theta] (meters, meters,
	 * radians)
	 * Distance-dependent: closer tags = lower uncertainty
	 * Typical base: [0.1-0.5, 0.1-0.5, 0.05-0.2] at 1 meter distance
	 */
	public static final Matrix<N3, N1> VISION_BASE_STD_DEV = VecBuilder.fill(0.2, 0.2, 0.1);

	/**
	 * Standard deviation for gyro measurements [x, y, theta] (meters, meters,
	 * radians)
	 * Gyro only provides theta, so x/y are infinite (ignored)
	 * Typical: [∞, ∞, 0.01-0.02] for high-quality IMU
	 */
	public static final Matrix<N3, N1> GYRO_STD_DEV = VecBuilder.fill(Double.POSITIVE_INFINITY,
			Double.POSITIVE_INFINITY, 0.015);

	/**
	 * Process noise for Kalman filter [x, y, theta] (meters, meters, radians)
	 * Represents uncertainty in motion model
	 * Typical: [0.01-0.05, 0.01-0.05, 0.005-0.02]
	 */
	public static final Matrix<N3, N1> KALMAN_PROCESS_NOISE = VecBuilder.fill(0.02, 0.02, 0.01);

	// ============================================
	// MOTION MODEL PARAMETERS
	// ============================================
	/** Maximum expected acceleration (m/s²) for motion model */
	public static final double MAX_ACCELERATION = 5.0;

	/** Maximum expected angular acceleration (rad/s²) for motion model */
	public static final double MAX_ANGULAR_ACCELERATION = 10.0;

	/** Time constant for velocity decay in motion model (seconds) */
	public static final double VELOCITY_DECAY_CONSTANT = 0.1;

	// ============================================
	// VISION FILTERING PARAMETERS
	// ============================================
	/** Maximum distance from odometry for vision measurement acceptance (meters) */
	public static final double MAX_VISION_DISTANCE_FROM_ODOMETRY = 2.0;

	/** Maximum angular difference from odometry for vision acceptance (radians) */
	public static final double MAX_VISION_ANGULAR_DIFFERENCE = 0.5; // ~30 degrees

	/** Minimum number of tags required for high-confidence vision measurement */
	public static final int MIN_TAGS_FOR_HIGH_CONFIDENCE = 2;

	/** Maximum tag distance for reliable measurements (meters) */
	public static final double MAX_RELIABLE_TAG_DISTANCE = 6.0;

	/**
	 * Maximum reprojection error for vision measurement acceptance (pixels)
	 * Higher reprojection error indicates poor tag detection (occlusion, blur,
	 * etc.)
	 * Typical good detections: < 0.5 pixels
	 * Typical acceptable: < 1.0 pixels
	 * Reject if > 2.0 pixels
	 */
	public static final double MAX_REPROJECTION_ERROR_PIXELS = 2.0;

	/**
	 * Maximum tag area variance for multi-tag detections
	 * Large variance in tag sizes indicates inconsistent detection quality
	 */
	public static final double MAX_TAG_AREA_VARIANCE = 0.3;

	// ============================================
	// BEAM BREAK SENSOR PARAMETERS
	// ============================================
	/** Digital I/O port for intake beam break sensor */
	public static final int INTAKE_BEAM_BREAK_PORT = 0;

	/**
	 * Debounce time for beam break sensor (seconds)
	 * Note: Actual sensor polling happens at the fusion loop rate (50 Hz), not
	 * 1000+ Hz.
	 * The sensor hardware may respond at high rates, but we poll it at ~50 Hz to
	 * avoid
	 * unnecessary CPU load. Digital input reads are very fast (~microseconds), but
	 * processing and logging at 1000 Hz would be wasteful.
	 */
	public static final double BEAM_BREAK_DEBOUNCE_TIME = 0.05;

	/**
	 * Actual polling rate for beam break sensor (Hz)
	 * Limited by fusion loop rate to avoid excessive CPU usage.
	 */
	public static final double BEAM_BREAK_POLLING_RATE = FUSION_UPDATE_RATE; // ~50 Hz

	// ============================================
	// OBJECT DETECTION PARAMETERS
	// ============================================
	/** Limelight network table name */
	public static final String LIMELIGHT_TABLE_NAME = "limelight";

	/** Minimum confidence threshold for object detection (0.0 to 1.0) */
	public static final double MIN_DETECTION_CONFIDENCE = 0.5;

	/** Maximum detection range (meters) */
	public static final double MAX_DETECTION_RANGE = 5.0;

	// ============================================
	// LOGGING RATE LIMITING
	// ============================================
	/**
	 * Maximum logging rate for high-frequency sensors (Hz)
	 * Prevents excessive network traffic and CPU usage.
	 * Even if sensors update at 250 Hz, we only log at this rate.
	 */
	public static final double MAX_LOGGING_RATE = 50.0;

	/**
	 * Logging interval for detailed sensor data (seconds)
	 * Detailed logs are emitted at this interval, not every loop cycle.
	 */
	public static final double DETAILED_LOG_INTERVAL = 0.1; // Every 100ms
}
