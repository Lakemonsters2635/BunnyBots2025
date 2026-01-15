package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final Pose2d[] APRIL_TAG_POSITIONS = {
    null,
    null,
    null,
    null,
    null,
    null,
    null,
    null,
    null,
    new Pose2d(2.417, 2.988, new Rotation2d(Math.toRadians(270))),
    null,
    null,
    null,
    null,
    new Pose2d(3.125, 0, new Rotation2d(Math.toRadians(180)))

  };
  //This is the real april tag postition
  /* 
  public static final Pose2d[] APRIL_TAG_POSITIONS = {
    null,
    new Pose2d(11.878056, 7.424674, new Rotation2d(Math.toRadians(180))),
    new Pose2d(11.915394, 4.638040, new Rotation2d(Math.toRadians(90))),
    new Pose2d(11.311890, 4.390136, new Rotation2d(Math.toRadians(180))),
    new Pose2d(11.311890, 4.034536, new Rotation2d(Math.toRadians(180))),
    new Pose2d(11.915394, 3.431286, new Rotation2d(Math.toRadians(270))),
    new Pose2d(11.878056, 0.644398, new Rotation2d(Math.toRadians(180))),
    new Pose2d(11.952986, 0.644398, new Rotation2d(Math.toRadians(0))),
    new Pose2d(12.270994, 3.431286, new Rotation2d(Math.toRadians(270))),
    new Pose2d(4.021836, 3.678936, new Rotation2d(Math.toRadians(0))),
    new Pose2d(12.519152, 4.034536, new Rotation2d(Math.toRadians(0))),
    new Pose2d(12.270994, 4.638040, new Rotation2d(Math.toRadians(90))),
    new Pose2d(11.952986, 7.424674, new Rotation2d(Math.toRadians(0))),
    new Pose2d(16.533368, 7.403338, new Rotation2d(Math.toRadians(180))),
    new Pose2d(16.533368, 6.971538, new Rotation2d(Math.toRadians(180))),
    new Pose2d(16.532860, 4.323588, new Rotation2d(Math.toRadians(180))),
    new Pose2d(16.532860, 3.891788, new Rotation2d(Math.toRadians(180))),
    new Pose2d(4.663186, 0.644398, new Rotation2d(Math.toRadians(0))),
    new Pose2d(4.625594, 3.431286, new Rotation2d(Math.toRadians(270))),
    new Pose2d(5.229098, 3.678936, new Rotation2d(Math.toRadians(0))),
    new Pose2d(5.229098, 4.034536, new Rotation2d(Math.toRadians(0))),
    new Pose2d(4.625594, 4.638040, new Rotation2d(Math.toRadians(90))),
    new Pose2d(4.663186, 7.424674, new Rotation2d(Math.toRadians(0))),
    new Pose2d(4.588256, 7.424674, new Rotation2d(Math.toRadians(180))),
    new Pose2d(4.269994, 4.638040, new Rotation2d(Math.toRadians(90))),
    new Pose2d(4.021836, 4.390136, new Rotation2d(Math.toRadians(180))),
    new Pose2d(4.021836, 4.034536, new Rotation2d(Math.toRadians(180))),
    new Pose2d(4.269994, 3.431286, new Rotation2d(Math.toRadians(270))),
    new Pose2d(4.588256, 0.644398, new Rotation2d(Math.toRadians(180))),
    new Pose2d(0.007620, 0.665988, new Rotation2d(Math.toRadians(0))),
    new Pose2d(0.007620, 1.097788, new Rotation2d(Math.toRadians(0))),
    new Pose2d(0.008128, 3.745738, new Rotation2d(Math.toRadians(0))),
    new Pose2d(0.008128, 4.177538, new Rotation2d(Math.toRadians(0)))
  };
  */
  // OBJECT TRACKER SUBSYSTEM
  public static final double CAMERA_TILT = 37; // previous was 35
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
