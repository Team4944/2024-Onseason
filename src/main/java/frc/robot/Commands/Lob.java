package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints;
import frc.robot.subystems.Feeder.FeederSubsystem;
import frc.robot.subystems.Pivot.PivotSubsystem;
import frc.robot.subystems.Shooter.ShooterSubsystem;

public class Lob extends Command {
    private ShooterSubsystem shooter;
    private PivotSubsystem pivot;
    private FeederSubsystem feeder;
    private Boolean auto;
    private int x;
    


  /** Creates a new Shoot. */
  public Lob(ShooterSubsystem shooter, PivotSubsystem pivot, FeederSubsystem feeder, Boolean auto) {

    this.shooter = shooter;
    this.pivot = pivot;
    this.feeder = feeder;
    this.auto = auto;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, pivot, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int x =0;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    pivot.setAngle(Rotation2d.fromDegrees(10));

    shooter.setVelocity(7500);

    if ((shooter.isAtSetpoint() && pivot.isAtSetpoint() && x > 30)) {
      feeder.setFeederVoltage(Setpoints.scoringFeedVolts);
    } else {
      feeder.setFeederVoltage(0);
    }
    x++;

    // SmartDashboard.putNumber("TargetAngle", SpeakerShotRegression.calculateWristAngle(targetDistance).getDegrees());
    // SmartDashboard.putNumber("TargetDistance", targetDistance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    feeder.stop();
    pivot.stow();
    x=0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(auto==true) {
      return !feeder.isNotePresentTOF();
  }
    return false;
  }
}
