package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * GAME-SPECIFIC CONSTANTS
 * 
 * WARNING: THIS FILE IS GAME-SPECIFIC - DELETE AND REPLACE FOR NEW GAMES!
 * 
 * Contains all game-specific information:
 * - Field dimensions
 * - AprilTag layout
 * - Game piece dimensions
 * - Structure locations (REEF, BARGE, CORAL STATIONS, PROCESSOR)
 * - Scoring locations
 * 
 * For 2025 REEFSCAPE game.
 * When new game comes out: DELETE THIS FILE and create new one with new game
 * specs.
 */
public final class GameConstants {

	// ===================================================================
	// FIELD DIMENSIONS
	// ===================================================================

	/** Field length: 57 ft 6⅞ in = 17.545 meters */
	public static final double FIELD_LENGTH = Units.feetToMeters(57.0) + Units.inchesToMeters(6.875);

	/** Field width: 26 ft 5 in = 8.051 meters */
	public static final double FIELD_WIDTH = Units.feetToMeters(26.0) + Units.inchesToMeters(5.0);

	// ===================================================================
	// APRILTAG FIELD LAYOUT
	// ===================================================================

	/**
	 * Official 2025 REEFSCAPE AprilTag field layout.
	 * Contains all 22 tags with correct positions and orientations.
	 * 
	 * Tag breakdown:
	 * - CORAL STATION Tags (IDs 1, 2, 12, 13): Height 4 ft 5¼ in
	 * - PROCESSOR Tags (IDs 3, 16): Height 3 ft 9⅞ in
	 * - REEF Tags (IDs 6-11, 17-22): Height 6⅞ in (very low!)
	 * - BARGE Tags (IDs 4, 5, 14, 15): Height 5 ft 9 in, angled 30° from vertical
	 * 
	 * All tags mounted on 10½ in square polycarbonate plates.
	 * 
	 * TESTING MODE: Currently filtered to only tags 12-22 (tags 1-11 removed)
	 * To use all tags, change back to:
	 * AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
	 */
	public static final AprilTagFieldLayout APRILTAG_LAYOUT = createFilteredLayout(12, 22);

	/**
	 * Creates a filtered AprilTag layout with only tags in the specified range.
	 * Used for testing with specific tag sets.
	 */
	private static AprilTagFieldLayout createFilteredLayout(int minTagId, int maxTagId) {
		AprilTagFieldLayout fullLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
		java.util.List<edu.wpi.first.apriltag.AprilTag> filteredTags = new java.util.ArrayList<>();

		for (edu.wpi.first.apriltag.AprilTag tag : fullLayout.getTags()) {
			if (tag.ID >= minTagId && tag.ID <= maxTagId) {
				filteredTags.add(tag);
			}
		}

		return new AprilTagFieldLayout(filteredTags, FIELD_LENGTH, FIELD_WIDTH);
	}

	// ===================================================================
	// ALLIANCE AREAS
	// ===================================================================

	/** Alliance area: 30 ft × 13 ft 8⅜ in */
	public static final double ALLIANCE_AREA_LENGTH = Units.feetToMeters(30.0);
	public static final double ALLIANCE_AREA_WIDTH = Units.feetToMeters(13.0) + Units.inchesToMeters(8.375);

	// ===================================================================
	// REEF STRUCTURE
	// ===================================================================

	/** REEF hexagon width across flats: 5 ft 5½ in */
	public static final double REEF_WIDTH = Units.feetToMeters(5.0) + Units.inchesToMeters(5.5);

	/** REEF centered 12 ft from alliance wall */
	public static final double REEF_DISTANCE_FROM_WALL = Units.feetToMeters(12.0);

	/** REEF zone hexagon: 7 ft 9½ in across flats */
	public static final double REEF_ZONE_WIDTH = Units.feetToMeters(7.0) + Units.inchesToMeters(9.5);

	/** Robot starting line: 7 ft 4 in from REEF */
	public static final double START_LINE_DISTANCE_FROM_REEF = Units.feetToMeters(7.0) + Units.inchesToMeters(4.0);

	// REEF scoring levels
	/** Level 1 (trough): 18 in front edge */
	public static final double REEF_L1_HEIGHT = Units.inchesToMeters(18.0);

	/** Level 2 (angled branch): 2 ft 7⅞ in, 35° upward */
	public static final double REEF_L2_HEIGHT = Units.feetToMeters(2.0) + Units.inchesToMeters(7.875);
	public static final double REEF_L2_ANGLE = Units.degreesToRadians(35.0);

	/** Level 3 (angled branch): 3 ft 11 5/8 in, 35 degrees upward */
	public static final double REEF_L3_HEIGHT = Units.feetToMeters(3.0) + Units.inchesToMeters(11.625);
	public static final double REEF_L3_ANGLE = Units.degreesToRadians(35.0);

	/** Level 4 (vertical branch): 6 ft straight up */
	public static final double REEF_L4_HEIGHT = Units.feetToMeters(6.0);

	/** Branch spacing: 1 ft 1 in center-to-center */
	public static final double REEF_BRANCH_SPACING = Units.feetToMeters(1.0) + Units.inchesToMeters(1.0);

	// ===================================================================
	// BARGE & CAGES
	// ===================================================================

	/** BARGE overall: 29 ft 2 in wide × 3 ft 8 in deep × 8 ft 5 in tall */
	public static final double BARGE_WIDTH = Units.feetToMeters(29.0) + Units.inchesToMeters(2.0);
	public static final double BARGE_DEPTH = Units.feetToMeters(3.0) + Units.inchesToMeters(8.0);
	public static final double BARGE_HEIGHT = Units.feetToMeters(8.0) + Units.inchesToMeters(5.0);

	/** BARGE zone: 3 ft 10 in × 12 ft 2½ in */
	public static final double BARGE_ZONE_WIDTH = Units.feetToMeters(3.0) + Units.inchesToMeters(10.0);
	public static final double BARGE_ZONE_LENGTH = Units.feetToMeters(12.0) + Units.inchesToMeters(2.5);

	/** Each CAGE: 2 ft tall × 7⅜ in wide */
	public static final double CAGE_HEIGHT = Units.feetToMeters(2.0);
	public static final double CAGE_WIDTH = Units.inchesToMeters(7.375);

	// ===================================================================
	// CORAL STATIONS
	// ===================================================================

	/** CORAL STATION area: 5 ft 10⅞ in × 13 ft 8⅜ in */
	public static final double CORAL_STATION_WIDTH = Units.feetToMeters(5.0) + Units.inchesToMeters(10.875);
	public static final double CORAL_STATION_LENGTH = Units.feetToMeters(13.0) + Units.inchesToMeters(8.375);

	/** CORAL STATION opening: 6 ft 4 in wide × 7 in tall */
	public static final double CORAL_STATION_OPENING_WIDTH = Units.feetToMeters(6.0) + Units.inchesToMeters(4.0);
	public static final double CORAL_STATION_OPENING_HEIGHT = Units.inchesToMeters(7.0);

	/** CORAL STATION bottom height: 3 ft 1½ in from carpet */
	public static final double CORAL_STATION_BOTTOM_HEIGHT = Units.feetToMeters(3.0) + Units.inchesToMeters(1.5);

	/** CORAL STATION chute angle: 55° */
	public static final double CORAL_STATION_CHUTE_ANGLE = Units.degreesToRadians(55.0);

	// ===================================================================
	// PROCESSOR
	// ===================================================================

	/** PROCESSOR area: 3 ft 7⅜ in × 7 ft 6 in */
	public static final double PROCESSOR_WIDTH = Units.feetToMeters(3.0) + Units.inchesToMeters(7.375);
	public static final double PROCESSOR_LENGTH = Units.feetToMeters(7.0) + Units.inchesToMeters(6.0);

	/** PROCESSOR opening AprilTag center height: 3 ft 9⅞ in */
	public static final double PROCESSOR_TAG_HEIGHT = Units.feetToMeters(3.0) + Units.inchesToMeters(9.875);

	// ===================================================================
	// APRILTAG HEIGHTS (for reference)
	// ===================================================================

	/** CORAL STATION Tags (IDs 1, 2, 12, 13): Height 4 ft 5¼ in */
	public static final double TAG_CORAL_STATION_HEIGHT = Units.feetToMeters(4.0) + Units.inchesToMeters(5.25);

	/** PROCESSOR Tags (IDs 3, 16): Height 3 ft 9⅞ in */
	public static final double TAG_PROCESSOR_HEIGHT = Units.feetToMeters(3.0) + Units.inchesToMeters(9.875);

	/** REEF Tags (IDs 6-11, 17-22): Height 6⅞ in (very low!) */
	public static final double TAG_REEF_HEIGHT = Units.inchesToMeters(6.875);

	/** BARGE Tags (IDs 4, 5, 14, 15): Height 5 ft 9 in, angled 30° from vertical */
	public static final double TAG_BARGE_HEIGHT = Units.feetToMeters(5.0) + Units.inchesToMeters(9.0);
	public static final double TAG_BARGE_ANGLE = Units.degreesToRadians(30.0);

	/** All tags mounted on 10½ in square polycarbonate plates */
	public static final double TAG_PLATE_SIZE = Units.inchesToMeters(10.5);

	// ===================================================================
	// GAME PIECES
	// ===================================================================

	/** CORAL (Game Piece 1) - Length: 11⅞ in */
	public static final double CORAL_LENGTH = Units.inchesToMeters(11.875);

	/** CORAL outer diameter: 4½ in */
	public static final double CORAL_OD = Units.inchesToMeters(4.5);

	/** CORAL inner diameter: 4 in */
	public static final double CORAL_ID = Units.inchesToMeters(4.0);

	/** CORAL weight: 1.1-1.8 lbs */
	public static final double CORAL_WEIGHT_MIN = 1.1; // lbs
	public static final double CORAL_WEIGHT_MAX = 1.8; // lbs

	/** ALGAE (Game Piece 2) - Diameter: 16.25 in ± 0.25 in */
	public static final double ALGAE_DIAMETER = Units.inchesToMeters(16.25);
	public static final double ALGAE_DIAMETER_TOLERANCE = Units.inchesToMeters(0.25);

	// ===================================================================
	// COORDINATE SYSTEM REFERENCE
	// ===================================================================

	/**
	 * Blue alliance origin is at (0, 0) with 0° rotation pointing downfield.
	 * Red alliance origin is at (FIELD_LENGTH, FIELD_WIDTH) with 180° rotation.
	 * 
	 * AprilTag positions are automatically loaded from
	 * AprilTagFields.k2025ReefscapeAndyMark
	 * which contains all 22 tags with correct positions and orientations.
	 */

	/** Blue alliance origin pose */
	public static final Pose2d BLUE_ORIGIN = new Pose2d(0, 0, new Rotation2d());

	/**
	 * Red alliance origin pose (approximate - use actual tag positions for precise)
	 */
	public static final Pose2d RED_ORIGIN = new Pose2d(FIELD_LENGTH, FIELD_WIDTH, Rotation2d.fromDegrees(180));
}
