# Simulation Testing Guide for Sensor Fusion System

This guide shows you how to easily test the sensor fusion system using WPILib simulation and AdvantageKit.

## Quick Start

### Step 1: Add Sensor Fusion to RobotContainer

Add this to your `RobotContainer.java` constructor in the `SIM` case:

```java
case SIM:
    // ... existing drive, vision setup ...
    
    // Create sensor fusion for simulation
    BeamBreakSensorSim beamBreakSim = new BeamBreakSensorSim(
        SensorFusionConstants.INTAKE_BEAM_BREAK_PORT,
        SensorFusionConstants.BEAM_BREAK_DEBOUNCE_TIME
    );
    
    ObjectDetectionSystemSim objectDetectionSim = new ObjectDetectionSystemSim(
        SensorFusionConstants.LIMELIGHT_TABLE_NAME,
        beamBreakSim
    );
    
    sensorFusion = SensorFusionFactory.createFusionSystem(
        drive,
        vision,
        SensorFusionConstants.LIMELIGHT_TABLE_NAME,
        -1  // Don't create real beam break, use simulated one
    );
    
    // Replace with simulated components (if factory doesn't handle it)
    // You may need to modify SensorFusionFactory to accept pre-created components
    
    // Create simulation helper for easy testing
    SensorFusionSimHelper simHelper = new SensorFusionSimHelper(
        sensorFusion,
        beamBreakSim,
        objectDetectionSim
    );
    break;
```

### Step 2: Run Simulation

1. **Start WPILib Simulation:**
   - In VS Code, click the "Start Simulation" button (robot icon)
   - Or use: `./gradlew simulateJava`

2. **Enable Field Visualization:**
   - The simulation will show the robot on the field
   - Vision simulation automatically detects AprilTags
   - You can see the robot pose estimate update in real-time

### Step 3: Test Sensor Fusion

#### Test Vision Localization

The vision system automatically simulates AprilTag detection based on robot position:

1. **Drive the robot** using teleop controls
2. **Watch the pose estimate** in AdvantageKit logs or NetworkTables
3. **Check vision corrections** - when robot is near AprilTags, pose should correct

#### Test Object Detection

Use the simulation helper in test code or commands:

```java
// In a test command or test mode:
SensorFusionSimHelper simHelper = ...; // Get from RobotContainer

// Simulate game piece detected
simHelper.simulateGamePieceDetected();

// Check detection status
DetectionStatus status = simHelper.getDetectionStatus();
System.out.println("Has game piece: " + status.hasGamePiece());

// Simulate target visible
simHelper.simulateTargetDetected(
    5.0,   // 5 degrees right
    -10.0, // 10 degrees down
    2.5,   // 2.5 meters away
    "apriltag_1",
    0.8    // 80% confidence
);

// Simulate no target
simHelper.simulateNoTarget();
```

## Viewing Fusion Data

### AdvantageKit Logging

The fusion system automatically logs data to AdvantageKit:

- **Fusion/FusedPose** - Current best-estimate pose
- **Fusion/VisionConfidence** - Vision sensor confidence
- **Fusion/WheelOdometryConfidence** - Odometry confidence
- **Detection/HasGamePiece** - Game piece detection status
- **Detection/HasTarget** - Target detection status

View logs in AdvantageKit's log viewer after running simulation.

### NetworkTables

You can also view data in NetworkTables:

```java
NetworkTableInstance.getDefault().getTable("AdvantageKit")
    .getEntry("Fusion/FusedPose")
    .getValue();  // Returns Pose2d
```

### Shuffleboard

Add widgets to Shuffleboard to visualize:

1. **Pose Widget** - Shows robot position on field
2. **Number Widget** - Shows confidence values
3. **Boolean Widget** - Shows detection status

## Testing Scenarios

### Scenario 1: Vision Correction

**Test:** Verify vision corrects odometry drift

1. Drive robot forward 5 meters
2. Move to a position near an AprilTag
3. Check that pose estimate corrects to tag position
4. Verify confidence increases when tag is visible

### Scenario 2: Outlier Rejection

**Test:** Verify bad vision measurements are rejected

1. Manually inject a bad vision measurement (far from odometry)
2. Check that it's rejected (rejectedVisionMeasurements increases)
3. Verify pose doesn't jump incorrectly

### Scenario 3: Game Piece Detection

**Test:** Verify beam break sensor works

1. Simulate game piece detected: `simHelper.simulateGamePieceDetected()`
2. Check detection status: `status.hasGamePiece()` should be true
3. Simulate ejection: `simHelper.simulateGamePieceEjected()`
4. Verify status updates correctly

### Scenario 4: Multi-Sensor Fusion

**Test:** Verify all sensors contribute to pose estimate

1. Drive robot while vision and odometry are both active
2. Check confidence values for each sensor
3. Verify fused pose is reasonable blend of all sensors
4. Disable one sensor (e.g., simulate vision off) and verify system degrades gracefully

## Debugging Tips

### Check Update Rates

Monitor the fusion statistics:

```java
var stats = sensorFusion.getFusionStatistics();
System.out.println("Total updates: " + stats.totalUpdates());
System.out.println("Vision measurements: " + stats.visionMeasurements());
System.out.println("Rejected: " + stats.rejectedVisionMeasurements());
```

### Verify Timing

Check that vision measurements aren't stale:

```java
System.out.println("Last vision time: " + stats.lastVisionMeasurementTime());
System.out.println("Has recent vision: " + stats.hasRecentVisionMeasurement());
```

### Monitor Confidence

Watch confidence values to see sensor health:

- Low vision confidence = camera issues or bad lighting
- Low odometry confidence = encoder problems
- All low = system not working properly

## Common Issues

### Vision Not Updating

- **Check:** VisionLocalizer is updating periodically
- **Check:** Camera simulation is enabled in PhotonVision
- **Check:** AprilTags are in the field layout

### Pose Jumps Around

- **Check:** Vision measurements are being validated (check rejected count)
- **Check:** Timestamps are correct
- **Check:** Standard deviations are reasonable

### Beam Break Not Working

- **Check:** Sensor is created with correct channel
- **Check:** Simulation helper is calling setBeamBroken()
- **Check:** Detection system is updating periodically

## Advanced Testing

### Replay Logs

Use AdvantageKit's replay feature:

1. Record a simulation run
2. Replay the log
3. Verify fusion behavior matches expectations

### Custom Test Commands

Create commands for automated testing:

```java
public class TestFusionCommand extends Command {
    private final SensorFusionSimHelper simHelper;
    private int step = 0;
    
    @Override
    public void execute() {
        switch (step) {
            case 0:
                simHelper.simulateGamePieceDetected();
                break;
            case 1:
                simHelper.simulateTargetDetected(...);
                break;
            // etc.
        }
        step++;
    }
}
```

## Integration with RobotContainer

Here's a complete example of integrating sensor fusion in simulation:

```java
// In RobotContainer.java
private SensorFusionManager sensorFusion;
private SensorFusionSimHelper simHelper;  // For testing only

public RobotContainer() {
    switch (Constants.currentMode) {
        case SIM:
            // ... create drive, vision ...
            
            // Create simulated sensors
            BeamBreakSensorSim beamBreakSim = new BeamBreakSensorSim(
                SensorFusionConstants.INTAKE_BEAM_BREAK_PORT,
                SensorFusionConstants.BEAM_BREAK_DEBOUNCE_TIME
            );
            
            ObjectDetectionSystemSim objectDetectionSim = new ObjectDetectionSystemSim(
                SensorFusionConstants.LIMELIGHT_TABLE_NAME,
                beamBreakSim
            );
            
            // Create fusion (you may need to modify factory to accept pre-created components)
            sensorFusion = SensorFusionFactory.createMinimalFusionSystem(drive, vision);
            
            // Create helper for testing
            simHelper = new SensorFusionSimHelper(
                sensorFusion,
                beamBreakSim,
                objectDetectionSim
            );
            
            // Make helper accessible for testing
            // (e.g., store in a field or provide getter)
            break;
    }
}

// For testing/debugging
public SensorFusionSimHelper getSimHelper() {
    return simHelper;
}
```

Now you can easily test the sensor fusion system in simulation!

