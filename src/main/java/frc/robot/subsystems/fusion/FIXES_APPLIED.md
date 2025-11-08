# Fixes Applied to Sensor Fusion System

This document summarizes the fixes applied to address the identified issues.

## ✅ Fixed Issues

### 1. Update Rates vs Fusion Timing
**Problem:** Vision measurements at 20 Hz could cause "jumps" when fused with 250 Hz odometry.

**Fix:**
- Documented that `SwerveDrivePoseEstimator` internally handles interpolation/extrapolation using timestamps
- Added timestamp validation to reject stale measurements (>100ms old)
- Added clear documentation explaining how slower sensors are synchronized

### 2. Weighting vs Covariance
**Problem:** Arbitrary weights (0.7, 0.4, etc.) don't match Kalman filter theory which uses covariance matrices.

**Fix:**
- Clarified that "weights" are conceptual/monitoring only, not used in fusion
- Documented that actual fusion uses covariance matrices (standard deviations) properly
- Added extensive comments explaining Kalman filter uses covariances, not arbitrary weights
- Ensured all fusion uses proper statistical weighting through SwerveDrivePoseEstimator

### 3. Beam Break Frequency
**Problem:** Documented as 1000+ Hz polling rate, which is unrealistic for WPILib/Java.

**Fix:**
- Changed polling rate to fusion loop rate (~50 Hz)
- Added documentation explaining hardware responds quickly but polling is limited to 50 Hz
- Clarified that 20ms latency is acceptable for FRC control loops
- Noted that interrupt-driven reading is possible but not necessary for most use cases

### 4. Vision Measurement Validation
**Problem:** Needed better reprojection error handling and outlier rejection.

**Fix:**
- Added Mahalanobis distance validation (accounts for measurement uncertainty)
- Added timestamp validation (rejects stale measurements)
- Improved outlier rejection using statistical methods
- Added documentation explaining VisionLocalizer already filters by ambiguity/tag count
- Enhanced confidence calculation based on measurement quality

### 5. Logging Rate Limiting
**Problem:** Logging at 250 Hz + beam break at 1000 Hz would cause network congestion.

**Fix:**
- Added rate limiting for detailed logs (every 100ms instead of every 20ms)
- Pose logging still happens every cycle (critical data) but at 50 Hz max
- Detailed statistics and confidence values log at reduced rate
- Added `MAX_LOGGING_RATE` and `DETAILED_LOG_INTERVAL` constants

### 6. AI Fusion Layer Clarification
**Problem:** "AI Fusion" was ambiguous - could imply machine learning with latency.

**Fix:**
- Clarified that "AI" means intelligent/statistical fusion, not machine learning
- Documented that fusion uses Kalman filtering (proper statistical methods)
- Explained that all processing is offline with no external dependencies
- Made it clear that SwerveDrivePoseEstimator performs the actual fusion

### 7. Accelerometer/AHRS Fusion
**Problem:** Needed to clarify how accelerometer data is fused with gyro.

**Fix:**
- Documented that Pigeon 2 IMU internally fuses gyro + accelerometer using AHRS algorithms
- Noted that this happens at the hardware/firmware level
- Clarified that we receive already-fused heading from the IMU

## 📊 Updated Constants

### Timing
- `MAX_VISION_MEASUREMENT_AGE`: 100ms (rejects stale measurements)
- `BEAM_BREAK_POLLING_RATE`: 50 Hz (realistic polling rate)
- `MAX_LOGGING_RATE`: 50 Hz (prevents network congestion)
- `DETAILED_LOG_INTERVAL`: 100ms (reduced logging frequency)

### Validation
- `MAX_REPROJECTION_ERROR_PIXELS`: 2.0 pixels (documented, used conceptually)
- `MAX_TAG_AREA_VARIANCE`: 0.3 (for multi-tag consistency checking)

### Clarifications
- Renamed "weights" to "confidence factors" with clear documentation that they're monitoring only
- Added extensive comments explaining Kalman filter uses covariances

## 🔍 Key Technical Details

### Interpolation/Extrapolation
The `SwerveDrivePoseEstimator` handles timing synchronization internally:
- Vision measurements arrive with timestamps
- Estimator interpolates/extrapolates to current time using motion model
- Prevents jumps/lag by proper temporal alignment

### Statistical Fusion
- Uses proper Kalman filter covariance matrices (not arbitrary weights)
- Mahalanobis distance for outlier rejection (accounts for uncertainty)
- Adaptive covariance adjustment based on measurement quality

### Offline Operation
- All processing happens on-robot
- No cloud services or external dependencies
- Limelight processes images on-robot
- Statistical fusion algorithms run locally

## 📝 Documentation Improvements

1. Added extensive inline documentation explaining each layer
2. Clarified update rates and timing handling
3. Explained statistical vs conceptual "weights"
4. Documented polling vs hardware response rates
5. Added comments on offline operation guarantee

