package frc.robot;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.InterpolatingDouble;
import frc.robot.Util.InterpolatingTreeMap;
import frc.robot.Util.PolynomialRegression;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This class should not be
 * used for any other purpose. All constants should be declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
/**
 * Place to hold robot-wide numerical or boolean constants. This class should
 * not be used for any
 * other purpose. All constants should be declared globally (i.e. public
 * static). Do not put
 * anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

  public static final boolean tuningMode = true;

  public static final class IDs {

    // Tuner X
    public static final int pigeon = 0;
    public static final int swerveDriveTalon0 = 1;
    public static final int swerveTurnTalon0 = 2;
    public static final int swerveDriveTalon1 = 3;
    public static final int swerveTurnTalon1 = 4;
    public static final int swerveDriveTalon2 = 5;
    public static final int swerveTurnTalon2 = 6;
    public static final int swerveDriveTalon3 = 7;
    public static final int swerveTurnTalon3 = 8;
    public static final int swerveCANcoder0 = 9;
    public static final int swerveCANcoder1 = 10;
    public static final int swerveCANcoder2 = 11;
    public static final int swerveCANcoder3 = 12;

    // REV Hardware Client
    public static final int flywheelLeft = 19;
    public static final int flywheelRight = 20;

    // PWF Config Page 10.49.44.2:5812
    public static final int intakesensor = 1;
    public static final int shootersensor = 2;
    public static final int ampsensor = 3;

  }


  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


  public class XboxConstants {
    // Joystick Axises
    public static final int L_JOYSTICK_HORIZONTAL = 0;
    public static final int L_JOYSTICK_VERTICAL = 1;
    public static final int LT = 2;
    public static final int RT = 3;
    public static final int R_JOYSTICK_HORIZONTAL = 4;
    public static final int R_JOYSTICK_VERTICAL = 5;

    // Controller Buttons
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LB_BUTTON = 5;
    public static final int RB_BUTTON = 6;
    public static final int SELECT_BUTTON = 7;
    public static final int START_BUTTON = 8;

    // These buttons are when you push down the left and right circle pad
    public static final int L_JOYSTICK_BUTTON = 9;
    public static final int R_JOYSTICK_BUTTON = 10;

    // D Pad Buttons
    public static final int DPAD_UP = 0;
    public static final int DPAD_UP_RIGHT = 45;
    public static final int DPAD_RIGHT = 90;
    public static final int DPAD_DOWN_RIGHT = 135;
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_DOWN_LEFT = 225;
    public static final int DPAD_LEFT = 270;
    public static final int DPAD_UP_LEFT = 315;

    // Controller Zeroes
    public static final double ZERO = 0.15;
  }

  /**
   * Gotten from here
   * https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
   */
  public static class AprilTags {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo
        .loadAprilTagLayoutField();

    public static int BLUE_SOURCE_LEFT = 1;
    public static int BLUE_SOURCE_RIGHT = 2;
    public static int RED_SPEAKER_BOTTOM = 3;
    public static int RED_SPEAKER_TOP = 4;
    public static int RED_AMP = 5;
    public static int BLUE_AMP = 6;
    public static int BLUE_SPEAKER_TOP = 7;
    public static int BLUE_SPEAKER_BUTTON = 8;
    public static int RED_SOURCE_LEFT = 9;
    public static int RED_SOURCE_RIGHT = 10;
    public static int RED_STAGE_BOTTOM = 11;
    public static int RED_STAGE_TOP = 12;
    public static int RED_STAGE_SIDE = 13;
    public static int BLUE_STAGE_SIDE = 14;
    public static int BLUE_STAGE_TOP = 15;
    public static int BLUE_STAGE_BOTTOM = 16;
  }

  public static Pose2d mirrorPose(Pose2d bluePose) {
    return new Pose2d(
        Constants.AprilTags.aprilTagFieldLayout.getFieldLength() - bluePose.getX(),
        bluePose.getY(),
        Rotation2d.fromRadians(Math.PI - bluePose.getRotation().getRadians()));
  }

  public static class PoseConfig {
    // Increase these numbers to trust your model's state estimates less.
    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;

    // Increase these numbers to trust global measurements from vision less.
    public static final double kVisionStdDevX = 5;
    public static final double kVisionStdDevY = 5;
    public static final double kVisionStdDevTheta = 500;
  }

  public static final boolean kIsTuningMode = true;
  public static final double kConfigTimeoutSeconds = 0.1;

  public static class Setpoints {

    public static final double PivotStowAngle = 0;
    public static final double ElevatorStowHeight = 0;

    public static final double PivotIntakeAngle = 10;
    
    public static final double intakeFeedVolts = 10; 
    public static final double injectFeedVolts = -8; 
    public static final double slowFeedVolts = 4;
    public static final double scoringFeedVolts = 8;
    
    public static final double intakingTargetVoltage = 8;
    public static final double outtakingTargetVoltage = -8;

    public static final double ampEjectTargetVoltage = 11;
    public static final double ampInjectTargetVoltage = -6;
    
    public static final double pivotMinClamp = 0;
    public static final double pivotMaxClamp = 58;
    
    public static final double shooterMinClamp = 0;
    public static final double shooterMaxClamp = 10000;
  }

  public static final double AprilTagHeights[] = {
    53.38,
    53.38,
    57.13,
    57.13,
    53.38,
    53.38,
    57.13,
    57.13,
    53.38,
    53.38,
    52,
    52,
    52,
    52,
    52,
    52
};


  public static final class AmpConstants {
    
    public static final int ampTalonID = 18; 
    public static final String ampTalonCANBus = "rio";

    public static final int ampSensorID = 3; 
    public static final RangingMode ampSensorRange = RangingMode.Short;
    public static final double ampSampleTime = 0;

    public static final double isNotePresentTOF = 200; // Milimeters

    public static final TalonFXConfiguration kAmpConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive));

    public static final DutyCycleOut ampDutyCycle = new DutyCycleOut(0, false, false, false, false);
    public static final TorqueCurrentFOC ampTorqueControl = new TorqueCurrentFOC(0, 0.95, 0, false, false, false);
  }

  public static final class ElevatorConstants {

    public static final int elevatorLeaderTalonID = 21; 
    public static final int elevatorFollowerTalonID = 22;
    public static final String elevatorTalonCANBus = "rio";

    public static final double elevatorGearRatio = 1/0.0681; // Sensor to Mechanism Ratio
    public static final double elevatorPinionRadius = Units.inchesToMeters(.48295); // Meters

    public static final double maxElevatorHeight = 0.2; // Meters

    public static final TalonFXConfiguration kElevatorConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(false))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive))
      .withSlot0(new Slot0Configs()
        .withKV(0)
        .withKA(0)
        .withKP(15)
        .withKI(0)
        .withKD(0)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(0.35))
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(elevatorGearRatio))
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(6)
        .withMotionMagicAcceleration(18)
        .withMotionMagicJerk(0))
      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(elevatorMetersToRotations(maxElevatorHeight)));

    public static final PositionVoltage elevatorPositionControl = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    public static final Follower followerControl = new Follower(elevatorLeaderTalonID, false);

    public static final double heightErrorTolerance = 0.0025; // Meters

    public static final double kElevatorFastUpdateFrequency = 50; // Hertz
    public static final double kElevatorMidUpdateFrequency = 40; // Hertz

    public static double elevatorMetersToRotations(double meters) {

      return meters / (2 * Math.PI * elevatorPinionRadius);
    }

    public static double elevatorRotationsToMeters(double rotations) {

      return rotations * (2 * Math.PI * elevatorPinionRadius);
    }
  }

  public static final class FeederConstants {

    public static final int feederTalonID = 13; 
    public static final String feederTalonCANBus = "rio";

    public static final int feederSensorID = 4; 
    public static final RangingMode feederSensorRange = RangingMode.Short;
    public static final double feederSampleTime = 0;

    public static final double isNotePresentTOF = 180; // Milimeters

    public static final TalonFXConfiguration kFeederConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive));

    public static final DutyCycleOut feederDutyCycle = new DutyCycleOut(0, false, false, false, false);
    public static final TorqueCurrentFOC feederTorqueControl = new TorqueCurrentFOC(0, 0.95, 0, false, false, false);
  }

  public static final class IntakeConstants {

    public static final int intakeTalonID = 17;
    public static final String intakeTalonCANBus = "rio";

    public static final int intakeSensorID = 1;
    public static final RangingMode intakeSensorRange = RangingMode.Short;
    public static final double intakeSampleTime = 0;

    public static final double isNotePresentTOF = 350; // Milimeters

    public static final TalonFXConfiguration kIntakeConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40)
        .withSupplyCurrentLimit(40)
        .withStatorCurrentLimitEnable(false)
        .withSupplyCurrentLimitEnable(false))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive));


    public static final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0, true, false, false, false);
    public static final TorqueCurrentFOC intakeTorqueControl = new TorqueCurrentFOC(0, 0.95, 0, false, false, false);
  }

  public static final class PivotConstants {

    public static final int pivotTalonID =  16; 
    public static final String pivotTalonCANBus = "rio";
    public static final int pivotEncoderID = 60;
    public static final String pivotEncoderCANBus = "rio";

    public static final double pivotGearRatio = 111.65; // Sensor to Mechanism Ratio

    public static final Rotation2d pivotMinAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d pivotMaxAngle = Rotation2d.fromDegrees(58);

    public static final TalonFXConfiguration kPivotConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(60)
        .withSupplyCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive))
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0.45)
        .withMotionMagicAcceleration(0.45)
        .withMotionMagicJerk(0))
      .withSlot0(new Slot0Configs()
        .withKV(0)
        .withKA(0)
        .withKP(1300) //1000
        .withKI(0)
        .withKD(1)
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(6)
        .withKS(0))
      .withFeedback(new FeedbackConfigs()
        //  .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        //  .withFeedbackRemoteSensorID(pivotEncoderID)
      .withSensorToMechanismRatio(pivotGearRatio))
        //  .withRotorToSensorRatio(pivotGearRatio))
      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(pivotMaxAngle.getRotations())
        .withReverseSoftLimitThreshold(pivotMinAngle.getRotations()));

    public static final CANcoderConfiguration kPivotEncoderConfiguration = new CANcoderConfiguration()
    .withMagnetSensor(new MagnetSensorConfigs()
      .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
      .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    public static final MotionMagicExpoVoltage pivotPositionControl = new MotionMagicExpoVoltage(0, true, 0, 0, true, false, false);

    public static final double kPivotPositionUpdateFrequency = 50; // Hertz
    public static final double kPivotErrorUpdateFrequency = 50; // Hertz

    public static final Rotation2d angleErrorTolerance = Rotation2d.fromDegrees(2.5); // Degrees
  }

  public static final class ShooterConstants {

    public static final int shooterTalonLeaderID = 14; 
    public static final int shooterTalonFollowerID = 15;
    public static final String shooterTalonCANBus = "rio";

    //public static final double shooterGearRatio = 0.5; // Sensor to Mechanism Ratio
    public static final double shooterGearRatio = 0.5;

    public static final double shooterVelocityTolerance = 250; // RPM

    public static final TalonFXConfiguration kShooterConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withSupplyCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true))
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive))
      .withSlot0(new Slot0Configs()
        //.withKS(1) 4499: s=1, v=.2, p=12, 
        .withKV(.075) // 0.075
        .withKP(.205) // 0.125
        .withKI(0)
        .withKD(0))
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(shooterGearRatio));

    public static final VelocityVoltage shooterControl = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
    public static final Follower followerControl = new Follower(shooterTalonLeaderID, true);
    public static final double kShooterVelocityUpdateFrequency = 10; // Hertz
  }

  public static double[][] kRPMValues = {
    {9.80, 8600},
  {8.90, 8600},
  {7.62,8600},
  {6.85, 8600},
  {6.62, 8600},
  {6.33, 8600},
  {6.02, 8500},
    {5.51, 8500},
    {4.8, 8000},
    {4.33, 7000},
    {3.71, 7000},
    {3.09, 7000},
    {2.63, 7000},
    {1.90, 7000},
    {1.40, 7000}
};

public static double[][] kPivotValues = {
  {9.80, 17.25},
  {8.90, 17.75},
  {7.62, 18.65},
  {6.85, 19},
  {6.62, 19.5},
  {6.33, 19.75},
  {6.02, 20},
  {5.51, 21},
    {4.8, 22.75},
    {4.33, 24.5},
    {3.71, 27},
    {3.09, 30},
    {2.63, 34},
    {1.90, 43},
    {1.40, 52}
};

public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPivotMap = new InterpolatingTreeMap<>();
public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap = new InterpolatingTreeMap<>();

public static PolynomialRegression kPivotRegression;
public static PolynomialRegression kRPMRegression;

static {
    for (double[] pair : kRPMValues) {
        kRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }

    for (double[] pair : kPivotValues) {
        kPivotMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }

    kPivotRegression = new PolynomialRegression(kPivotValues, 1);
    kRPMRegression = new PolynomialRegression(kRPMValues, 1);
}

	
    public static class PhotonVisionConstants {
        //public static final String kCameraName = "front_cam";
        public static class front_left_cam {
            public static final String kCameraName = "front_left";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(Units.inchesToMeters(9.812), Units.inchesToMeters(9.29),
                            Units.inchesToMeters(8.693)),
                    new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(0)));
        }

        public static class top_right_cam {
            public static final String kCameraName = "top_right";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(.159, -.213,.53),
                    new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(0)));
        }
       

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class LimelightConstants {
        public static final String kCameraName = "limelight";
    }

    public static class SwerveConstants {
        public static final double kSwerveDriveSupplyCurrentLimit = 30.00;
        public static final boolean kSwerveDriveSupplyCurrentLimitEnable = true;
        public static final double kSwerveDriveSupplyCurrentThreshold = 90.00;
        public static final double kSwerveDriveSupplyTimeThreshold = 0.01;
    }

    public static class ControllerConstants {
        public static final double triggerThreshold = 0.4;

    }

    public static class FieldConstants {
        public static final Translation2d BLUE_SPEAKER = new Translation2d(0, 0);
        public static final Translation2d RED_SPEAKER = new Translation2d(1, 1);
        public static final double BLUE_AUTO_PENALTY_LINE = 8.6; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double RED_AUTO_PENALTY_LINE = 8; // X distance from origin to center of the robot almost fully crossing the midline

        public static final double NOTE_DIAMETER = 14; // Outer diameter of note

    }
}
