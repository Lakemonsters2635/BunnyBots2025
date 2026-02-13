package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {
  // SWERVE
  public static final double kPModuleTurningController = 0.5; // 0.5
  public static final double kPModuleDriveController = 0; // added random value for test

  public static final double kMaxSpeedMetersPerSecond = 6.0; // 4.0;
  // TODO: 'EncoderDistancePerPulse' should be calculated based on the gearing and wheel diameter
  public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5); // 3.5 inch wheels
  public static final double kDriveEncoderDistancePerPulse = 0.0001 / 0.002706682950506;

  public static final double maxModuleLinearSpeed = 1.75; // Irrelevant used for createPath
  public static final double maxModuleLinearAccelaration = 8;
  public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI;
  public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 12 * Math.PI;

  public static final double DRIVETRAIN_WHEELBASE_WIDTH = Units.inchesToMeters(22); // 26.625
  public static final double DRIVETRAIN_WHEELBASE_LENGTH = Units.inchesToMeters(26); // 19.625

  public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 5;
  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 6;
  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 2;

  public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 7;
  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 8;
  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 0;

  public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 3; // TODO: set correct ports 8
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 4; // 7
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 1;

  public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 1; // TODO: set correct ports 2
  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 2; // 1
  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 3;

  // SWERVE MODULE STATES
  public static final int FRONT_LEFT_MODULE_STATE_INDEX = 0;
  public static final int FRONT_RIGHT_MODULE_STATE_INDEX = 1;
  public static final int BACK_LEFT_MODULE_STATE_INDEX = 2;
  public static final int BACK_RIGHT_MODULE_STATE_INDEX = 3;

  // ANGLE OFFSETS
  public static final double FRONT_LEFT_ANGLE_OFFSET =
      Math.toRadians(-13 - 90 - 2.5 + 180); // TODO: set correct values
  public static final double FRONT_RIGHT_ANGLE_OFFSET = Math.toRadians(-53 + 90 - 229 + 45);
  public static final double BACK_LEFT_ANGLE_OFFSET = Math.toRadians(-14 + 90 - 40 + 45 + 180);
  public static final double BACK_RIGHT_ANGLE_OFFSET = Math.toRadians(75 - 90 + 3 + 180);

  // HAT CONSTANTS
  public static final double HAT_POWER_MOVE = 0.1;
  public static final double HAT_POWER_ROTATE = 0.3;

  public static final int HAT_POV_MOVE_LEFT = 270;
  public static final int HAT_POV_MOVE_RIGHT = 90;
  public static final int HAT_POV_MOVE_FORWARD = 0;
  public static final int HAT_POV_MOVE_BACK = 180;
  public static final int HAT_POV_0 = 0; // Left hat up
  public static final int HAT_POV_180 = 180; // Left hat down
  public static final int HAT_POV_ROTATE_LEFT = 270;
  public static final int HAT_POV_ROTATE_RIGHT = 90;

  // ELEVATOR CONSTANTS
  public static final int ELEVATOR_MOTOR_ID = 11;
  public static final int ELEVATOR_ENCODER_CLICK =
      0; // number of clicks in a full rotation of the elevator motor
  public static final double ELEVATOR_VOLTAGE = 1.5;
  public static final double ELEVATOR_UP_VOLTAGE = 0.5;
  public static final double ELEVATOR_HOLD_VOLTAGE = 0.7;

  // Outtake
  public static final int OUTTAKE_VOLTAGE = 1;
  public static final int INTAKE_VOLTAGE = 1;
  public static final double SHOOTER_TARGET_DELTA_ANGLE = 1.4 + .26;
  public static final double SHOOTER_P = 20;
  public static final double SHOOTER_I = 0;
  public static final double SHOOTER_D = .2;

  // JOYSTICKS
  public static final int LEFT_JOYSTICK_PORT = 0;
  public static final int RIGHT_JOYSTICK_PORT = 1;

  // BUTTONS
  public static final int SHOOTER_BUTTON = 1; // TODO: set correct buttons
  // VISION AUTO COMMAND
  // TODO: VisionAutoData assumes only a single camera
  public static final int CAM_X_OFFSET = 0;
  public static final int CAM_Y_OFFSET = 0;
  public static final int CAM_ANGLE_OFFSET =
      0; // Rad // TODO: Code currently assumes the camera is facing forward
  // public static final Pose2d[] APRIL_TAG_POSITIONS = {
  //   null,
  //   null,
  //   null,
  //   null,
  //   null,
  //   null,
  //   null,
  //   null,
  //   new Pose2d(3.715, 1.364, new Rotation2d(Math.toRadians(180))),
  //   new Pose2d(2.417, 2.988, new Rotation2d(Math.toRadians(270))),
  //   null,
  //   null,
  //   null,
  //   null,
  //   new Pose2d(3.125, 0, new Rotation2d(Math.toRadians(180)))
  // };
  // This is the real april tag postition

  public static final Matrix<N3, N1> ODOMETRY_STD =
      VecBuilder.fill(
          0.002, 0.002, 0.0035); // These are placeholder numbers calibrate later: x, y, z

  public static final Matrix<N3, N1> VISION_STD =
      VecBuilder.fill(0.3, 0.3, 0.7); // N3 and N1 matrix dimensions

  public static final Pose2d[] APRIL_TAG_POSITIONS = {
    null, // index 0 unused
    new Pose2d(467.64 * 0.0254, 292.31 * 0.0254, Rotation2d.fromDegrees(180)), // 1
    new Pose2d(469.11 * 0.0254, 182.60 * 0.0254, Rotation2d.fromDegrees(90)), // 2
    new Pose2d(445.35 * 0.0254, 172.84 * 0.0254, Rotation2d.fromDegrees(180)), // 3
    new Pose2d(445.35 * 0.0254, 158.84 * 0.0254, Rotation2d.fromDegrees(180)), // 4
    new Pose2d(469.11 * 0.0254, 135.09 * 0.0254, Rotation2d.fromDegrees(270)), // 5
    new Pose2d(467.64 * 0.0254, 25.37 * 0.0254, Rotation2d.fromDegrees(180)), // 6
    new Pose2d(470.59 * 0.0254, 25.37 * 0.0254, Rotation2d.fromDegrees(0)), // 7
    new Pose2d(483.11 * 0.0254, 135.09 * 0.0254, Rotation2d.fromDegrees(270)), // 8
    new Pose2d(492.88 * 0.0254, 144.84 * 0.0254, Rotation2d.fromDegrees(0)), // 9
    new Pose2d(492.88 * 0.0254, 158.84 * 0.0254, Rotation2d.fromDegrees(0)), // 10
    new Pose2d(483.11 * 0.0254, 182.60 * 0.0254, Rotation2d.fromDegrees(90)), // 11
    new Pose2d(470.59 * 0.0254, 292.31 * 0.0254, Rotation2d.fromDegrees(0)), // 12
    new Pose2d(650.92 * 0.0254, 291.47 * 0.0254, Rotation2d.fromDegrees(180)), // 13
    new Pose2d(650.92 * 0.0254, 274.47 * 0.0254, Rotation2d.fromDegrees(180)), // 14
    new Pose2d(650.90 * 0.0254, 170.22 * 0.0254, Rotation2d.fromDegrees(180)), // 15
    new Pose2d(650.90 * 0.0254, 153.22 * 0.0254, Rotation2d.fromDegrees(180)), // 16
    new Pose2d(183.59 * 0.0254, 25.37 * 0.0254, Rotation2d.fromDegrees(0)), // 17
    new Pose2d(182.11 * 0.0254, 135.09 * 0.0254, Rotation2d.fromDegrees(270)), // 18
    new Pose2d(205.87 * 0.0254, 144.84 * 0.0254, Rotation2d.fromDegrees(0)), // 19
    new Pose2d(205.87 * 0.0254, 158.84 * 0.0254, Rotation2d.fromDegrees(0)), // 20
    new Pose2d(182.11 * 0.0254, 182.60 * 0.0254, Rotation2d.fromDegrees(90)), // 21
    new Pose2d(183.59 * 0.0254, 292.31 * 0.0254, Rotation2d.fromDegrees(0)), // 22
    new Pose2d(180.64 * 0.0254, 292.31 * 0.0254, Rotation2d.fromDegrees(180)), // 23
    new Pose2d(168.11 * 0.0254, 182.60 * 0.0254, Rotation2d.fromDegrees(90)), // 24
    new Pose2d(158.34 * 0.0254, 172.84 * 0.0254, Rotation2d.fromDegrees(180)), // 25
    new Pose2d(158.34 * 0.0254, 158.84 * 0.0254, Rotation2d.fromDegrees(180)), // 26
    new Pose2d(168.11 * 0.0254, 135.09 * 0.0254, Rotation2d.fromDegrees(270)), // 27
    new Pose2d(180.64 * 0.0254, 25.37 * 0.0254, Rotation2d.fromDegrees(180)), // 28
    new Pose2d(0.30 * 0.0254, 26.22 * 0.0254, Rotation2d.fromDegrees(0)), // 29
    new Pose2d(0.30 * 0.0254, 43.22 * 0.0254, Rotation2d.fromDegrees(0)), // 30
    new Pose2d(0.32 * 0.0254, 147.47 * 0.0254, Rotation2d.fromDegrees(0)), // 31
    new Pose2d(0.32 * 0.0254, 164.47 * 0.0254, Rotation2d.fromDegrees(0)) // 32
  };

  // OBJECT TRACKER SUBSYSTEM
  public static final double CAMERA_TILT = 30; // previous was 37, 30 is for testing post bunnybots
  public static final double[] CAMERA_OFFSET = {3, 13}; // offset = [x, y], In inches

  // INTAKE CONSTANTS
  public static final int INTAKE_MOTOR_TOP_ID = 9;
  public static final int INTAKE_MOTOR_BOTTOM_ID = 10;
  public static final double INTAKE_TOP_VOLTAGE = -2.5;
  public static final double INTAKE_BOT_VOLTAGE = -6;

  // INDEX SUBSYSTEM CONSTANTS
  public static final int INDEX_SUBSYSTEM_ID = 12;
  public static final double INDEX_SUBSYSTEM_VOLTAGE = -3; // negative is the correct direction
  public static final double INDEX_EXPECTED_POSITION = -90; // In degrees

  // BUTTONS
  public static final int INTAKE_BUTTON = 1; // TODO: change ID when given robot
  public static final int ELEVATOR_UP_BUTTON = 5;
  public static final int ELEVATOR_DOWN_BUTTON = 3;
}
