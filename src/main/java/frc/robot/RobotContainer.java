
package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;

// NOTE: Fusion subsystem is intentionally commented out/not included
// All fusion code is in frc.robot.subsystems.fusion package but not used here
// To enable fusion, uncomment and add fusion imports/initialization below

public class RobotContainer {
	private final Drive drive;
	private final VisionLocalizer vision;
	private final TagFieldCalibrator fieldCalibrator;

	private final CommandXboxController controller = new CommandXboxController(0);
	private final LoggedDashboardChooser<Command> autoChooser;

	public RobotContainer() {
		switch (Constants.currentMode) {
			case REAL:
				drive = new Drive(
						new GyroIOPigeon2(),
						new ModuleIOTalonFX(TunerConstants.FrontLeft),
						new ModuleIOTalonFX(TunerConstants.FrontRight),
						new ModuleIOTalonFX(TunerConstants.BackLeft),
						new ModuleIOTalonFX(TunerConstants.BackRight));
				// Vision automatically uses AprilTag field layout - no manual dimensions
				// needed!
				// Cameras detect tags and calculate robot pose automatically
				vision = new VisionLocalizer(drive::addVisionMeasurement, drive,
						new VisionIOPhotonReal(VisionConstants.cameraNames[0],
								VisionConstants.vehicleToCameras[0]),
						new VisionIOPhotonReal(VisionConstants.cameraNames[1],
								VisionConstants.vehicleToCameras[1]));
				break;

			case SIM:
				drive = new Drive(
						new GyroIO() {
						},
						new ModuleIOSim(TunerConstants.FrontLeft),
						new ModuleIOSim(TunerConstants.FrontRight),
						new ModuleIOSim(TunerConstants.BackLeft),
						new ModuleIOSim(TunerConstants.BackRight));
				// Vision automatically uses AprilTag field layout - no manual dimensions
				// needed!
				// Cameras detect tags and calculate robot pose automatically
				vision = new VisionLocalizer(
						drive::addVisionMeasurement,
						drive,
						new VisionIOPhotonSim(VisionConstants.cameraNames[0],
								VisionConstants.vehicleToCameras[0],
								drive::getPose),
						new VisionIOPhotonSim(VisionConstants.cameraNames[1],
								VisionConstants.vehicleToCameras[1], drive::getPose));
				break;

			default:
				drive = new Drive(
						new GyroIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						});
				// Vision automatically uses AprilTag field layout - no manual dimensions
				// needed!
				vision = new VisionLocalizer(drive::addVisionMeasurement, drive, new VisionIO() {
				});
		}

		// Vision automatically sends pose corrections to drive subsystem
		vision.setVisionConsumer(drive::addVisionMeasurement);

		// Create field calibrator for custom field layouts
		// This allows the robot to adapt to ANY field configuration!
		fieldCalibrator = new TagFieldCalibrator(drive, vision);
		vision.setCalibrator(fieldCalibrator);

		// ===================================================================
		// HOW ROBOT POSITION TRACKING WORKS (Automatic AprilTag Localization)
		// ===================================================================
		//
		// The robot finds its position through multiple sensor layers:
		//
		// LAYER 1: WHEEL ODOMETRY (Active)
		// - How it works: Integrates wheel encoder positions + gyro angle
		// - Update rate: 250 Hz (very fast, continuous)
		// - Accuracy: Good for short distances, drifts over time
		// - Purpose: Provides smooth, continuous tracking between vision updates
		// - View in AdvantageKit: Check "Odometry/Layer1_WheelOdometry"
		//
		// LAYER 2: VISION LOCALIZATION (Active - Automatic!)
		// - How it works: Cameras detect AprilTags and automatically calculate pose
		// - NO MANUAL DIMENSIONS NEEDED! Uses pre-loaded 2025 field layout
		// - The system knows where all AprilTags are on the field automatically
		// - When cameras see tags, they calculate robot position using:
		// * Known tag positions (from AprilTagFieldLayout)
		// * Camera-to-tag distance and angles
		// * Camera mounting positions on robot
		// - Update rate: ~20 Hz (when tags are visible)
		// - Accuracy: Very accurate, corrects Layer 1 drift automatically
		// - View in AdvantageKit: Check "Odometry/Layer2_VisionPose"
		//
		// HOW IT WORKS AUTOMATICALLY:
		// 1. VisionConstants.aprilTagLayout loads the 2025 field with all tag positions
		// 2. Cameras detect AprilTags and identify them (tag ID)
		// 3. System looks up tag position from field layout
		// 4. Calculates robot pose using camera-to-tag geometry
		// 5. Sends corrections to drive subsystem automatically
		// - NO manual typing of dimensions needed!
		//
		// CURRENT SETUP:
		// - Layer 1 (wheel odometry) + Layer 2 (vision) both active
		// - Robot starts at (3.181, 3.960) - manually set initial position
		// - Once vision sees tags, it will automatically correct any drift
		// - The two cameras (FL and FR) look over the field for tags
		//
		// ===================================================================

		// Initialize robot pose at starting position
		// Comment out this line to start at blue alliance origin (0, 0)
		drive.setPose(new Pose2d(3.181, 3.960, new Rotation2d()));

		// ... NamedCommands registration (unchanged)

		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		autoChooser.addOption("Drive Wheel Radius Characterization",
				AkitDriveCommands.wheelRadiusCharacterization(drive));
		autoChooser.addOption("Drive Simple FF Characterization", AkitDriveCommands.feedforwardCharacterization(drive));

		configureButtonBindings();
	}

	private void configureButtonBindings() {
		drive.setDefaultCommand(
				AkitDriveCommands.joystickDrive(
						drive,
						() -> -controller.getLeftY() * 0.5, // Reduced speed to 50%
						() -> -controller.getLeftX() * 0.5, // Reduced speed to 50%
						() -> -controller.getRightX() * 0.5)); // Reduced speed to 50%

		// Drive pose reset
		controller.rightBumper().onTrue(
				Commands.runOnce(
						() -> drive.setPose(
								new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
						drive)
						.ignoringDisable(true));

		// Field calibration controls
		// Y button - Start calibration (drive around to collect tag positions)
		controller.y().onTrue(Commands.runOnce(() -> {
			fieldCalibrator.startCalibration();
			System.out.println("Field calibration started! Drive around to collect tag positions.");
		}));

		// B button - Finish calibration (builds custom field layout)
		controller.b().onTrue(Commands.runOnce(() -> {
			boolean success = fieldCalibrator.finishCalibration();
			if (success) {
				fieldCalibrator.getCustomLayout().ifPresent(layout -> {
					VisionConstants.setCustomLayout(layout);
					vision.updateFieldLayout();
					System.out.println("Field calibration complete! Custom layout is now active.");
				});
			} else {
				System.out.println("Calibration failed - not enough observations collected.");
			}
		}));
	}

	public void teleopInit() {
		// Zero heading so forward on stick maps to straight field-forward
		drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
	}

	public Command getAutonomousCommand() {
		return autoChooser.get();
	}
}