# Sensor Fusion System Documentation

## Overview

This sensor fusion system provides a comprehensive, multi-layer approach to robot pose estimation and object detection. It integrates all sensor inputs to produce the best possible robot pose estimate and situational awareness.

## Architecture

### Pose Estimation Layers

1. **Wheel Odometry (Layer 1)**
   - Update rate: 250 Hz
   - Weight: 0.7
   - Input: Swerve module encoders + gyro
   - Output: Continuous pose tracking

2. **Vision Localization (Layer 2)**
   - Update rate: 20 Hz
   - Weight: 0.4
   - Input: AprilTag detections
   - Output: Global position corrections

3. **Absolute Encoders (Layer 3)**
   - Update rate: 250 Hz
   - Weight: Integrated into Layer 1
   - Input: CANcoder absolute positions
   - Output: Accurate module angles

4. **Gyro + Accelerometer (Layer 4)**
   - Update rate: 200 Hz
   - Weight: 0.9 (heading only)
   - Input: Pigeon 2 IMU
   - Output: Heading and angular velocity

5. **Motion Model Prediction (Layer 5)**
   - Update rate: 50 Hz
   - Weight: 0.2
   - Input: Chassis velocities
   - Output: Velocity-based predictions

6. **High-Level Fusion (Layer 6)**
   - Update rate: 50 Hz
   - Weight: Dynamic
   - Input: All above layers
   - Output: Best-estimate fused pose

### Object Detection Layers

1. **Limelight/Camera (Layer 1)**
   - Update rate: 30 Hz
   - Input: Camera images
   - Output: Game piece and target detection

2. **Beam Break Sensors (Layer 2)**
   - Update rate: 1000+ Hz
   - Input: Digital sensor readings
   - Output: Intake confirmation

## Usage

### Basic Setup in RobotContainer

```java
// In RobotContainer constructor
private SensorFusionManager sensorFusion;

public RobotContainer() {
    // ... initialize drive, vision, etc. ...
    
    // Create sensor fusion system
    sensorFusion = SensorFusionFactory.createFusionSystem(
        drive,                          // Drive subsystem
        vision,                         // Vision localizer
        "limelight",                    // Limelight table name (or null)
        SensorFusionConstants.INTAKE_BEAM_BREAK_PORT  // Beam break channel (or -1)
    );
    
    // Register sensor fusion as a subsystem
    // (it extends SubsystemBase and will update automatically)
}

// Use fused pose in commands/autonomous
public Pose2d getRobotPose() {
    return sensorFusion.getFusedPose();
}

// Check for game pieces
public boolean hasGamePiece() {
    DetectionStatus status = sensorFusion.getDetectionStatus();
    return status != null && status.hasGamePiece();
}
```

### Minimal Setup (Pose Only, No Object Detection)

```java
sensorFusion = SensorFusionFactory.createMinimalFusionSystem(drive, vision);
```

## Configuration

All constants are in `SensorFusionConstants.java`:

- **Update rates**: Adjust based on processing capability
- **Weights**: Tune based on sensor reliability
- **Standard deviations**: Adjust for your robot's sensor precision
- **Thresholds**: Tune for your field and camera setup

## Tuning Guide

### Vision Weight Adjustment
- Increase if camera setup is reliable
- Decrease if experiencing false detections

### Wheel Odometry Weight
- Increase for well-calibrated encoders
- Decrease if experiencing encoder drift

### Standard Deviations
- Lower = higher confidence in that sensor
- Tune based on actual sensor performance

## Offline Operation

All processing is done locally - no internet required. The system uses:
- Local sensor readings
- Local camera processing (Limelight on-robot)
- Local fusion algorithms
- No cloud services or network dependencies

