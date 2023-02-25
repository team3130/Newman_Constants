// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Newman_Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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

    public final static int CAN_SpinnyBar = 18;

    public final static int CAN_PNM = 19;

    /**
     * PNM ID's
     */
    public static final int PNM_LargeSolenoid = 2;
    public static final int PNM_SmallSolenoid = 1;
    public final static int PNM_Grabber = 4;

    /**
     * Digital inputs
     */
    public static final int PUNCHY_LIMIT_SWITCH = 0;

    // Order should match side
    public static final int[] turningId = new int[] {CAN_LeftFrontSteer, CAN_LeftBackSteer, CAN_RightFrontSteer, CAN_RightBackSteer};
    public static final int[] spinningId = new int[] {CAN_LeftFrontDrive, CAN_LeftBackDrive, CAN_RightFrontDrive, CAN_RightBackDrive};
    public final static int[] CANCoders = new int[] {CANCoderTopLeft, CANCoderBottomLeft, CANCoderTopRight, CANCoderBottomRight};

    /**
     * Encoder offsets
     */
    public static final double kTopLeftOffset = Math.toRadians(264.90);
    public static final double kBottomLeftOffset = Math.toRadians(275.27);
    public static final double kTopRightOffset = Math.toRadians(129.3);
    public static final double kBottomRightOffset = Math.toRadians(357.71484);
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
    public final static double kMaxDriveVoltage = 9d;

    /** Chassis auton */
    public static final double kPXController = 3;
    public static final double kIXController = 0.5;
    public static final double kDXController = 0;
    public static final double kPYController = 3;
    public static final double kIYController = 0.5;
    public static final double kDYController = 0;
    public static final double kPThetaController = 5;
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
     * Timer Settings
     */
    public final static double timetoIntake = 0.25;

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


    public final static double SwerveKp = 0.55;
    public final static double SwerveKi = 0;
    public final static double SwerveKd = 0.01;
    public final static double SwerveKf = 0;

    //Balancing PID values
    public final static double BalanceKp = 2; //do this first
    public final static double BalanceKi = 0; //then this
    public final static double BalanceKd = 2.5; //then this
    public final static double BalanceKf = 0; //idk what to do about this

    //TODO: Find a good value for this idk
    public final static double BalanceConstrain = 300;


    public final static double openLoopRampRate = 0.7;

    public final static double kPhysicalMaxSpeedMetersPerSecond = 3.6;

    public final static double kDeadband = 0.075;

    public final static double kMaxAccelerationDrive = 7;
    public final static double kMaxAccelerationAngularDrive = 3;

    public final static double kResetTime = 1.5;

        //Rotary Arm
    public final static double kPlacementRotaryArmGearInRatio = 16d/61d;
    public final static double kPlacementRotaryArmGearboxRatio = 12d/60d;
    public final static double kTicksToRadiansRotaryPlacementArm = kEncoderResolution * 2 * Math.PI * kPlacementRotaryArmGearInRatio * kPlacementRotaryArmGearboxRatio;
    public final static double kRadiansToTicksRotaryPlacementArm = 1/kTicksToRadiansRotaryPlacementArm;
    public final static double kMaxVelocityRotaryPlacementArm = Math.PI/4;
    public final static double kMaxAccelerationRotaryPlacementArm = Math.PI/8;

    //Extension Arm
    public static double placemenExtensionShaftRadius = Units.inchesToMeters(.25);
    public static double placementExtensionArmGearboxRatio = 16d / 61d;
    public static double placementExtensionArmGearInRatio = 12d / 60d;
    public static double ticksToRadiansExtensionPlacement = kEncoderResolution * 2 * Math.PI * placementExtensionArmGearboxRatio * placementExtensionArmGearInRatio;
    public static double ticksToMetersExtensionPlacement = ticksToRadiansExtensionPlacement * placemenExtensionShaftRadius;
    public static double radiansToTicksExtensionPlacement = 1 / ticksToRadiansExtensionPlacement;
    public static double maxVelocityPlacementExtensionArm = Math.PI / 4;
    public static double maxAccelerationPlacementExtensionArm = Math.PI / 8;
    public int ExtensionCAN_ID = CAN_ExtensionArm;

    public final static double kExtensionArmSpringXPosition = 1; // TODO: Find real value
    public final static double kExtensionArmSpringYPosition = 1; // TODO: Find real value

    public final static double kExtensionArmSpringConstant = 1; // TODO: Find real value

    public final static double kPercentOutputToHoldAtMaxExtension = 0.1; //TODO: Find real value
    public final static double kTorqueToPercentOutScalar = kPercentOutputToHoldAtMaxExtension / (kExtensionArmLength * kMassOfExtensionArm * kAccelerationDueToGravity); // magic number that turns torque into motor output
    public final static double kExtensionArmGearRatio = 1;
    public final static double kTicksToRadiansExtensionPlacement = kEncoderResolution * 2 * Math.PI * kExtensionArmGearRatio;
    // radians to distance is just radians * radius
    public final static double kRadiansToTicksExtensionPlacement = 1 / kTicksToRadiansExtensionPlacement;
    public final static double kTicksToMetersExtensionPlacement = kTicksToRadiansExtensionPlacement * kExtensionShaftRadius;
    public final static double kMaxVelocityPlacementExtensionArm = 0.2;
    public final static double kMaxAccelerationPlacementExtensionArm = 0.2;
    // Distance from the center axle of the springs mount position on the rotary arm
    public static final double kExtensionArmSpringMountPosition = 0.23; // TODO: Find real value

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
        public static final double xPos = Units.inchesToMeters(12);
        public static final double yPos = Units.inchesToMeters(0);
        public static final double zPos = Units.inchesToMeters(4.5);

        // TODO: Find these values
        public static final double pitch = 0;
        public static final double yaw = 0;
        public static final double roll = 0;

        public static double confidenceN1 = 0; // I'm guessing x component confidence
        public static double confidenceN2 = 0; // I'm guessing y component confidence
        public static double confidenceN3 = 0; // I'm guessing theta component confidence

        public final static int kMedianFilterWindowSize = 5;
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
