// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Newman_Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean debugMode = true;
    public static final boolean kEliminationRound = (DriverStation.getMatchType() == DriverStation.MatchType.Elimination);

    /**
     * CAN
     */
    public final static int CAN_LeftFrontSteer = 2;
    public final static int CAN_LeftFrontDrive = 3;
    public final static int CAN_RightFrontSteer = 4;
    public final static int CAN_RightFrontDrive = 5;

    public final static int CAN_LeftBackSteer = 6;
    public final static int CAN_LeftBackDrive = 7;
    public final static int CAN_RightBackSteer = 8;
    public final static int CAN_RightBackDrive = 9;

    public final static int CANCoderTopRight = 10;
    public final static int CANCoderBottomRight = 11;
    public final static int CANCoderTopLeft = 12;
    public final static int CANCoderBottomLeft = 13;

    public final static int CAN_RotaryArm = 14;
    public final static int CAN_ExtensionArm = 15;

    public final static int CAN_hopperright = 16;
    public final static int CAN_hopperleft = 17;

    public final static int CAN_PNM = 19;

    /**
     * PNM ID's
     */
    public static final int PNM_Intake = 2;
    public final static int PNM_Grabber = 0;
    public static final int PNM_Brake = 1;

    /**
     * Digital inputs
     */
    public static final int PUNCHY_LIMIT_SWITCH = 0;
    public static final int ROTARY_ARM_LIMIT_SWITCH = 1;

    // Order should match side
    public static final int[] turningId = new int[] {CAN_LeftFrontSteer, CAN_LeftBackSteer, CAN_RightFrontSteer, CAN_RightBackSteer};
    public static final int[] spinningId = new int[] {CAN_LeftFrontDrive, CAN_LeftBackDrive, CAN_RightFrontDrive, CAN_RightBackDrive};
    public final static int[] CANCoders = new int[] {CANCoderTopLeft, CANCoderBottomLeft, CANCoderTopRight, CANCoderBottomRight};

    /**
     * Encoder offsets
     */
    public static final double kTopLeftOffset = Math.toRadians(268.682);
    public static final double kBottomLeftOffset = Math.toRadians(281.426);
    public static final double kTopRightOffset = Math.toRadians(129.3);
    public static final double kBottomRightOffset = Math.toRadians(0);
    public static final double[] kCanCoderOffsets = new double[] {kTopLeftOffset, kBottomLeftOffset, kTopRightOffset, kBottomRightOffset};

    /**
     * Gear ratio and ticks per rev
     */
    public final static double kDriveGearRatio = 6.75; // checked 1/19
    public final static double kSteerGearRatio = 150d/7d; // checked 1/19
    public static final double kEncoderResolution = 2048;

    public static final double kWheelDiameter = Units.inchesToMeters(3.86);
    public static final double SteerTicksToRads = 1/(kEncoderResolution * kSteerGearRatio) * Math.PI * 2; // multiply by position
    public static final double SteerTicksToRadsPerSecond = SteerTicksToRads * 10; // multiply by velocity
    public final static double DriveTicksToMeters = kWheelDiameter * Math.PI * 1/(kEncoderResolution * kDriveGearRatio); // multiply by
    public static final double DriveTicksToMetersPerSecond = DriveTicksToMeters * 10; // multiply by velocity


    public final static double kMaxSteerVoltage = 5d;
    public final static double kMaxDriveVoltage = 8d;
    public final static double kMaxRotaryArmVoltage = 9d;
    public final static double kMaxExtensionArmVoltage = 10.5d;
    public final static double kMaxVoltageHopper = 9d;

    /** Chassis auton */
    public static final double kPXController = 3;
    public static final double kIXController = 0.5;
    public static final double kDXController = 0;
    public static final double kPYController = 3;
    public static final double kIYController = 0.5;
    public static final double kDYController = 0;
    public static final double kPThetaController = 7;
    public static final double kIThetaController = 0;

    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI; // max spiny acceleration
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // max spiny velocity
    // real max spiny speed (multiply by some number for safety)
    public static final double kMaxAngularSpeedRadiansPerSecond =  kPhysicalMaxAngularSpeedRadiansPerSecond;
    // spiny PID constraints
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

    /**
     * Length and width as measured as distances between center of wheels
     */
    // the left-to-right distance between the drivetrain wheels, should be measured from center to center
	public static final double trackWidth_m = 0.61;
	// the front-to-back distance between the drivetrain wheels, should be measured from center to center
	public static final double wheelBase_m = 0.61;


    /**
     * For swerve drive
     * translations for the distance to each wheel from the center of the bot.
     * Remember that forward (0 radians) is positive X
     * Check:
     *  right half the bot up half the bot      (0.5, 0.5)
     *  right half the bot down half the bot    (-0.5, 0.5)
     *  left half the bot up half the bot       (0.5, -0.5)
     *  left half the bot down half the bot     (-0.5, -0.5)
     * These look like coordinates to each wheel with the order being:
     *  top right,
     *  bottom right,
     *  top left,
     *  bottom left,
     */
	public static final Translation2d[] moduleTranslations = {
		new Translation2d(wheelBase_m / 2.0, trackWidth_m / 2.0),
		new Translation2d(-wheelBase_m / 2.0, trackWidth_m / 2.0),
		new Translation2d(wheelBase_m / 2.0, -trackWidth_m / 2.0),
		new Translation2d(-wheelBase_m / 2.0, -trackWidth_m / 2.0)
	};

    public static final boolean kNavxReversed = true;


    public final static double SwerveKpFrontRight = 1.35;
    public final static double SwerveKiFrontRight = 0.05;
    public final static double SwerveKdFrontRight = 0;
    public final static double SwerveKfFrontRight = 0;

    public final static double SwerveKpFrontLeft = 1.55;
    public final static double SwerveKiFrontLeft = 0.05;
    public final static double SwerveKdFrontLeft = 0.015;
    public final static double SwerveKfFrontLeft = 0;


    public final static double SwerveKpBackLeft = 1.6;
    public final static double SwerveKiBackLeft = 0.01;
    public final static double SwerveKdBackLeft = 0.015;
    public final static double SwerveKfBackLeft = 0;

    public final static double SwerveKpBackRight = 1.2;
    public final static double SwerveKiBackRight = 0.05;
    public final static double SwerveKdBackRight = 0;
    public final static double SwerveKfBackRight = 0;

    public final static double[] SwerveKp = new double[] {SwerveKpFrontLeft, SwerveKpBackLeft, SwerveKpFrontRight, SwerveKpBackRight};
    public final static double[] SwerveKi = new double[] {SwerveKiFrontLeft, SwerveKiBackLeft, SwerveKiFrontRight, SwerveKiBackRight};
    public final static double[] SwerveKd = new double[] {SwerveKdFrontLeft, SwerveKdBackLeft, SwerveKdFrontRight, SwerveKdBackRight};
    public final static double[] SwerveKf = new double[] {SwerveKfFrontLeft, SwerveKfBackLeft, SwerveKfFrontRight, SwerveKfBackRight};


    //Balancing PID values
    public final static double BalanceKp = 2.25; //do this first
    public final static double BalanceKi = 0; //this isn't real
    public final static double BalanceKd = 0; //then this
    public final static double BalanceKf = 0.4; //idk what to do about this

    public final static double kPhysicalMaxSpeedMetersPerSecond = 4.788; // 3.54 with voltage compensation

    public final static double kDeadband = 0.075;

    public final static double kMaxAccelerationDrive = 7;
    public final static double kMaxAccelerationAngularDrive = Math.PI;

    public final static double kResetTime = 0.75;

    //Rotary Arm
    public final static double kRotaryPlacementArmGearRatio = 0.0119008879; // experimentally found gear ratio
    public final static double kTicksToRadiansRotaryPlacementArm = (1/kEncoderResolution) * 2 * Math.PI * kRotaryPlacementArmGearRatio;
    public final static double kRadiansToTicksRotaryPlacementArm = 1/kTicksToRadiansRotaryPlacementArm;
    public final static double kMaxVelocityRotaryPlacementArm =  0.75 * Math.PI;
    public final static double kMaxAccelerationRotaryPlacementArm = Math.PI;

    public final static double kRotaryArmP = 0.6;
    public final static double kRotaryArmI = 0;
    public final static double kRotaryArmD = 0.075;

    public static final double lowPosition = Math.PI / 4;
    public static final double midPosition = Math.toRadians(90);
    public static final double highPosition = Math.toRadians(120);
    public static final double offGroundAngleCone = Math.toRadians(20);
    public static final boolean listener = true;
    public static final double midPositionCones = Math.toRadians(100);

    public static class Extension {
        public static final double offGroundPosition = 82065;
        public static final double kMaxExtensionLength = 190000;
        public static final double kPositionWithinBot = 14000;

        public static final double intermediatePosition = 0;

        public final static double kExtensionArmP = 1;
        public final static double kExtensionArmI = 0;
        public final static double kExtensionArmD = 0.05;

        public final static double kExtensionArmLengthExtendedMeters = Units.inchesToMeters(40);
        public static final double kExtensionArmLengthRetractedMeters = Units.inchesToMeters(25);

        public final static double kPercentOutputToHoldAtMaxExtension = 0.09;

        public final static double kRotaryStaticGain = kPercentOutputToHoldAtMaxExtension / (kMaxExtensionLength); // magic number that turns torque into motor output
        public final static double kExtensionArmGearRatio = 0.25;
        public final static double kTicksToRadiansExtensionPlacement = 1/(kEncoderResolution) * 2 * Math.PI * kExtensionArmGearRatio;
        public final static double kExtensionHexShaftRadius = Units.inchesToMeters(0.25);
        public final static double kTicksToMetersExtension = kTicksToRadiansExtensionPlacement * kExtensionHexShaftRadius;
        // radians to distance is just radians * radius
        public final static double kRadiansToTicksExtensionPlacement = 1 / kTicksToRadiansExtensionPlacement;
        // public final static double kTicksToMetersExtensionPlacement = kTicksToRadiansExtensionPlacement * kExtensionShaftRadius;
        public final static double kMaxVelocityPlacementExtensionArm = 125000;
        public final static double kMaxAccelerationPlacementExtensionArm = 150000;

        // should be the length of the extension arm when it is retracted in meters
            /*kTicksToMetersExtension * (kMaxExtensionLength - (kExtensionArmLengthExtendedMeters * (1/kTicksToMetersExtension)));*/ // should e 24 inches
    }

    public static class Field {

        public static final double[] yPositionsForRowBounds = new double[]{
                //0       1       2       3       4       5       6       7       8
                0, 0.7875, 1.3465, 1.9055, 2.4645, 3.0235, 3.5825, 4.1415, 4.7005, 5.5
        };

        public static final double xPositionForGridBlue = 1.35;
        public static final double xPositionForGridRed = 14.7;

        public static final double xPositionForRedHumanPlayerStation = 0.75;
        public static final double yPositionForRedHumanPlayerStation = 7.5;
        public static final double rotationForRedHumanPlayerStation = (Math.PI) / 2;

        public static final double xPositionForBlueHumanPlayerStation = 15.77;
        public static final double yPositionForBlueHumanPlayerStation = 7.5;
        public static final double rotationForBlueHumanPlayerStation = (3*Math.PI) / 2;

        //TODO: get placement height above ground
        public static final Translation3d kPlacementAxleHeightAboveGround = new Translation3d(0, 0, Units.inchesToMeters(27));

        public static final double heightOfConeSpireOne = 0.864;
        public static final double heightOfConeSpireTwo = 1.168;
        public static final double depthOfConeSpireOneSmall = 0.41;
        public static final double depthOfConeSpireOneBig = 0.795;
        public static final double depthOfConeSpireTwoSmall = 0.795;
        public static final double depthOfConeSpireTwoBig = 1.43;

        public static final double heightOfCubeSpotOne = 0.597;
        public static final double heightOfCubeSpotTwo = 0.902;
        public static final double depthOfCubeSpotOneSmall = 0.36;
        public static final double depthOfCubeSpotOneBig = 0.7526586;
        public static final double depthOfCubeSpotTwoSmall = 0.7526586;
        public static final double depthOfCubeSpotTwoBig = 1.43;

        // true for cone spot, false for cube spot. ignore bottom row
        public static final boolean[] coneSpots = new boolean[] {
                true, false, true, true, false, true, true, false, true,
                true, false, true, true, false, true, true, false, true,
        };
    }


    public static class Side {
         public static final int LEFT_FRONT = 0;
         public static final int LEFT_BACK = 1;
         public static final int RIGHT_FRONT = 2;
         public static final int RIGHT_BACK = 3;
    }

    /**
     * Camera constants
     */
    public static class Camera {
        // The position and orientation of the camera in meters
        public static final double xPos = Units.inchesToMeters(0);
        public static final double yPos = Units.inchesToMeters(12.5);
        public static final double zPos = Units.inchesToMeters(-38);

        // TODO: Find these values
        public static final double pitch = -15;
        public static final double yaw = 0;
        public static final double roll = 0;

        public static double confidenceN1 = 0; // I'm guessing x component confidence
        public static double confidenceN2 = 0; // I'm guessing y component confidence
        public static double confidenceN3 = 0; // I'm guessing theta component confidence

        public final static int kMedianFilterWindowSize = 5;

        public static double kCameraFOV = 0; // TODO: Find real value
    }

    // error gain for the KugelMediaFilter
    public static final double kKugelMedianFilterP = 7/(Math.PI);

    public static class Buttons {
        /**
     * Gamepad Button List
     */
    public static final int LST_BTN_A = 1;
    public static final int LST_BTN_B = 2;
    public static final int LST_BTN_X = 3;
    public static final int LST_BTN_Y = 4;
    public static final int LST_BTN_LBUMPER = 5;
    public static final int LST_BTN_RBUMPER = 6;
    public static final int LST_BTN_WINDOW = 7;
    public static final int LST_BTN_MENU = 8;
    public static final int LST_BTN_LJOYSTICKPRESS = 9;
    public static final int LST_BTN_RJOYSTICKPRESS = 10;

    /**
     * Gamepad POV List
     */
    public static final int LST_POV_UNPRESSED = -1;
    public static final int LST_POV_N = 0;
    public static final int LST_POV_NE = 45;
    public static final int LST_POV_E = 90;
    public static final int LST_POV_SE = 135;
    public static final int LST_POV_S = 180;
    public static final int LST_POV_SW = 225;
    public static final int LST_POV_W = 270;
    public static final int LST_POV_NW = 315;

    /**
     * Gamepad Axis List
     */
    public static final int LST_AXS_LJOYSTICKX = 0;
    public static final int LST_AXS_LJOYSTICKY = 1;
    public static final int LST_AXS_LTRIGGER = 2;
    public static final int LST_AXS_RTRIGGER = 3;
    public static final int LST_AXS_RJOYSTICKX = 4;
    public static final int LST_AXS_RJOYSTICKY = 5;
    }
}
