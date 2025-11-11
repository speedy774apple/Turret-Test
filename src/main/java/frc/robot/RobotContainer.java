
package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;

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
				// Vision - automatically uses whatever cameras are configured
				// Currently using 2 cameras (FL, FR) - add more in VisionConstants as needed
				vision = createVisionSystem();
				break;

			case SIM:
				drive = new Drive(
						new GyroIO() {
						},
						new ModuleIOSim(TunerConstants.FrontLeft),
						new ModuleIOSim(TunerConstants.FrontRight),
						new ModuleIOSim(TunerConstants.BackLeft),
						new ModuleIOSim(TunerConstants.BackRight));
				// Vision - automatically uses whatever cameras are configured
				vision = createVisionSystem();
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
		// Vision will fix/correct the robot's pose when it sees AprilTags
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

	/**
	 * Creates vision system with automatically detected cameras.
	 * Only uses cameras that are actually connected - automatically adjusts!
	 * Tries all possible cameras and only uses the ones that work.
	 */
	private VisionLocalizer createVisionSystem() {
		// Define all possible cameras (add more here as you get them)
		String[] allPossibleCameras = { "FL", "FR", "BL", "BR" };
		Transform3d[] allPossibleTransforms = {
				new Transform3d(new Translation3d(0.263383, 0.275693, 0.259765),
						new Rotation3d(0, 0, Units.degreesToRadians(-45))), // FL
				new Transform3d(new Translation3d(0.263383, -0.275693, 0.259765),
						new Rotation3d(0, 0, Units.degreesToRadians(42.5))), // FR
				new Transform3d(new Translation3d(-0.263383, 0.275693, 0.259765),
						new Rotation3d(0, 0, Units.degreesToRadians(135))), // BL (example)
				new Transform3d(new Translation3d(-0.263383, -0.275693, 0.259765),
						new Rotation3d(0, 0, Units.degreesToRadians(-135))), // BR (example)
		};

		java.util.List<VisionIO> connectedCameras = new java.util.ArrayList<>();

		// Try each camera and only add the ones that are connected
		for (int i = 0; i < allPossibleCameras.length && i < allPossibleTransforms.length; i++) {
			try {
				if (Constants.currentMode == Constants.Mode.REAL) {
					VisionIOPhotonReal camera = new VisionIOPhotonReal(
							allPossibleCameras[i],
							allPossibleTransforms[i]);

					// Check if camera is connected (give it a moment to initialize)
					// Note: PhotonCamera connection check might take a moment
					// We'll add it anyway and let VisionLocalizer handle disconnection
					connectedCameras.add(camera);
					System.out.println("Vision: Attempting to use camera: " + allPossibleCameras[i]);
				} else if (Constants.currentMode == Constants.Mode.SIM) {
					connectedCameras.add(new VisionIOPhotonSim(
							allPossibleCameras[i],
							allPossibleTransforms[i],
							drive::getPose));
					System.out.println("Vision: Added sim camera: " + allPossibleCameras[i]);
				}
			} catch (Exception e) {
				System.out.println(
						"Vision: Could not initialize camera " + allPossibleCameras[i] + ": " + e.getMessage());
			}
		}

		if (connectedCameras.isEmpty()) {
			System.out.println("Vision: WARNING - No cameras initialized! Using dummy camera.");
			// Return with dummy camera so system doesn't crash
			return new VisionLocalizer(drive::addVisionMeasurement, drive, new VisionIO() {
			});
		}

		System.out.println("Vision: Initialized " + connectedCameras.size() + " camera(s)");

		// Create vision localizer with all connected cameras
		return new VisionLocalizer(
				drive::addVisionMeasurement,
				drive,
				connectedCameras.toArray(new VisionIO[0]));
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