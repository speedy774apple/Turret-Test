package frc.robot.subsystems.fusion.objectdetection;

/**
* Detection status record containing all detection information.
* 
* This record holds the complete state of the object detection system,
* including game piece detection, target detection, and confidence scores.
*/
public record DetectionStatus(
boolean hasGamePiece,
boolean isNewDetection,
boolean hasTarget,
double targetAngleX,
double targetAngleY,
double targetDistance,
String targetType,
double confidence) {
}
