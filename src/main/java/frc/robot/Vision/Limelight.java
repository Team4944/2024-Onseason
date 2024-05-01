package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.RectanglePoseArea;

public class Limelight extends SubsystemBase {
  Alliance alliance;
  private String ll = "limelight";
  private Boolean enable = false;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private Pose2d botpose;
  private static final RectanglePoseArea field =
        new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));

  /** Creates a new Limelight. */
  public Limelight() {
  }

  @Override
  public void periodic() {

    double LLdistance = getDistance();
    SmartDashboard.putNumber("Limelight Range: ", LLdistance);
  }

     public static NetworkTable getAprilTagDetector(){
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    public static void init() {
    }


    public double getDistance() {
        double ty = getAprilTagDetector().getEntry("ty").getDouble(0);
        double tid =  getAprilTagDetector().getEntry("tid").getDouble(-1);
       if (tid == -1) return 0;
        double h2 = Constants.AprilTagHeights[1];
        double angleToGoal = Units.degreesToRadians(28 + ty);
        double heightToGoal = h2 - 23; //5in back from robot center
        double distance = heightToGoal / Math.tan(angleToGoal);
        return Units.inchesToMeters(distance);
    }

    public static boolean canSeeAprilTag(){
        return getAprilTagDetector().getEntry("tv").getDouble(0) == 1;
    }

    public static double[] getBotPoseArray(){
        return getAprilTagDetector().getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    public static Pose2d getBotPose(){
        double[] poseArray = getBotPoseArray();
        return new Pose2d(poseArray[0],poseArray[1],Rotation2d.fromDegrees(poseArray[5]));
    }

    public static double getLatency(){
        return Timer.getFPGATimestamp() - getBotPoseArray()[6]/1000.0;
    }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }

  public void trustLL(boolean trust) {
    this.trust = trust;
  }
}
