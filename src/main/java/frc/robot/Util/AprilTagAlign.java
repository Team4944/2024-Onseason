package frc.robot.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagAlign {

   public static PIDController rotationPID = createPIDController();
    private static PIDController createPIDController() {
        PIDController pid = new PIDController(0.03, 0, 0.0006);
        pid.setTolerance(0.01);
        pid.enableContinuousInput(0, 360);
        pid.setSetpoint(0);
        return pid;
        
    }
   
    public static double getR() {
    //     Translation2d target = FieldUtil.getAllianceSpeakerPosition();
    //     SmartDashboard.putNumber("POSE TARGET X", target.getX());
    //     SmartDashboard.putNumber("POSE TARGET Y", target.getY());

    //      Supplier<Rotation2d> robotAngle = () -> Rotation2d.fromRadians(  // Find the angle to turn the robot to
    // Math.atan((PoseTracker.field.getRobotPose().getY() - target.getY())
    //     / (PoseTracker.field.getRobotPose().getX() - target.getX())));

    //     return rotationPID.calculate(PoseTracker.field.getRobotPose().getRotation().getDegrees(), robotAngle.get().getDegrees());
    return rotationPID.calculate(NetworkTableInstance.getDefault().getTable("limelight-amp").getEntry("tx").getDouble(0));
    } }
