# Testing Sensor Fusion in Simulation

## Quick Start

1. **Run in Simulation Mode**
   - The code automatically detects simulation mode
   - Vision simulation is already configured in `RobotContainer.java`
   - Just run the robot code and it will use simulated cameras

2. **What Gets Simulated**
   - ✅ Layer 1 (Odometry): Wheel encoders + gyro - fully simulated
   - ✅ Layer 2 (Vision): PhotonVision simulation with AprilTags - fully simulated
   - ✅ Sensor Fusion: Confidence filtering and timer-based updates - fully functional

## How to Test

### Option 1: Use AdvantageKit Logging (Recommended)

1. **Run the simulation** (F5 or deploy in simulation mode)
2. **Open AdvantageKit** and connect to the robot
3. **Key Logs to Monitor:**
   - `Odometry/Robot` - Final fused pose (Layer 1 + Layer 2)
   - `Odometry/Layer1_GyroAngle` - Odometry-only pose
   - `Odometry/VisionApplied` - Whether vision measurements are being applied
   - `Odometry/VisionUpdateType` - HIGH_CONFIDENCE, GOOD_CONFIDENCE_APPLIED, or REJECTED
   - `Odometry/VisionConfidence` - Confidence score (0.0 to 1.0)
   - `Odometry/VisionReprojectionError` - Reprojection error estimate
   - `Odometry/VisionPoseDifference` - Difference from odometry
   - `Odometry/VisionUpdateTimer` - Timer for window-based updates
   - `Odometry/HighConfidenceUpdates` - Count of high-confidence updates
   - `Odometry/GoodConfidenceUpdates` - Count of good-confidence updates
   - `Odometry/VisionMeasurementsRejected` - Count of rejected measurements

4. **What to Look For:**
   - Vision measurements should appear when robot is near AprilTags
   - High-confidence updates (multiple tags) should apply immediately
   - Good-confidence updates (single tag) should apply after timer window
   - Rejected measurements should not affect pose
   - Fused pose should be more accurate than odometry alone

### Option 2: Use Field Visualization

1. **Run simulation** and open the field visualization
2. **Drive the robot around** using joystick controls
3. **Watch the robot pose:**
   - Robot should track position accurately when near AprilTags
   - Pose should correct when vision sees tags
   - Pose should remain stable when vision doesn't see tags (odometry only)

### Option 3: Add Test Commands

You can add test commands to verify specific scenarios:

```java
// In RobotContainer.java, add test commands:
public Command testVisionFusion() {
    return Commands.sequence(
        // Drive to a position with multiple tags visible
        new PathPlannerCommand(...),
        // Wait and verify vision updates
        Commands.waitSeconds(5.0),
        // Check logs to verify fusion worked
        Commands.print("Check AdvantageKit logs for vision fusion")
    );
}
```

## Testing Scenarios

### Scenario 1: High-Confidence Vision (Multiple Tags)
- **Setup:** Drive robot to position where 2+ AprilTags are visible
- **Expected:** Vision measurement applied immediately, timer resets
- **Verify:** `Odometry/VisionUpdateType` = "HIGH_CONFIDENCE"

### Scenario 2: Good-Confidence Vision (Single Tag)
- **Setup:** Drive robot to position where only 1 AprilTag is visible
- **Expected:** Vision measurement stored, applied after timer window (15 seconds)
- **Verify:** `Odometry/VisionUpdateType` = "GOOD_CONFIDENCE_APPLIED" after timer expires

### Scenario 3: Low-Confidence Vision (Rejected)
- **Setup:** Drive robot to position with poor tag visibility (far away, high ambiguity)
- **Expected:** Vision measurement rejected, odometry continues unchanged
- **Verify:** `Odometry/VisionUpdateType` = "REJECTED"

### Scenario 4: Odometry Drift Correction
- **Setup:** 
  1. Start robot at known position
  2. Drive around (odometry will drift)
  3. Return to position with AprilTags visible
- **Expected:** Vision corrects odometry drift when tags are seen
- **Verify:** `Odometry/VisionCorrection` shows correction magnitude

### Scenario 5: Timer-Based Updates
- **Setup:** 
  1. Drive to position with single tag (good confidence)
  2. Wait 15+ seconds without high-confidence updates
- **Expected:** Best good measurement applied at end of timer window
- **Verify:** `Odometry/VisionUpdateTimer` resets, good measurement applied

## Configuration Parameters

You can adjust these in `Drive.java`:

```java
private static final double REPROJECTION_ERROR_THRESHOLD = 0.1; // meters
private static final double POSE_DIFFERENCE_THRESHOLD = 3.0; // meters
private static final double VISION_UPDATE_WINDOW_SECONDS = 15.0; // seconds
```

## Troubleshooting

### Vision Not Appearing
- Check `Vision/SeesTags` - should be true when near tags
- Check `Vision/Connected` - cameras should be connected
- Verify AprilTag field layout is loaded correctly

### Vision Always Rejected
- Check `Odometry/VisionRejectionReason` for why measurements are rejected
- Adjust thresholds if needed (reprojection error, pose difference)
- Verify tag count is sufficient (need 2+ for high confidence)

### Fusion Not Working
- Check `Odometry/SensorFusionActive` - should be true
- Check `Odometry/Layer1_OdometryActive` - should be true
- Check `Odometry/Layer2_VisionActive` - should be true when vision is active
- Verify `Drive.processVisionMeasurement()` is being called

## Advantages of Simulation Testing

✅ **No Hardware Required** - Test anywhere, anytime
✅ **Repeatable** - Same conditions every time
✅ **Fast Iteration** - No need to wait for field setup
✅ **Safe** - Can't break anything
✅ **Full Logging** - All data available in AdvantageKit
✅ **Visualization** - See robot and field in real-time

## Next Steps After Simulation

Once simulation tests pass:
1. Test on real robot with cameras
2. Verify thresholds work in real conditions
3. Tune parameters based on real-world performance
4. Monitor logs during matches

