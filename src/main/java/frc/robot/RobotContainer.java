package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.*;
import frc.robot.Util.AprilTagLock;
import frc.robot.Vision.Limelight;
import frc.robot.subystems.Amp.AmpSubsystem;
import frc.robot.subystems.Elevator.ElevatorSubsystem;
import frc.robot.subystems.Feeder.FeederSubsystem;
import frc.robot.subystems.Intake.IntakeSubsystem;
import frc.robot.subystems.LED.LEDSubsystem;
import frc.robot.subystems.Pivot.PivotSubsystem;
import frc.robot.subystems.Shooter.ShooterSubsystem;
import frc.robot.subystems.drive.Drive;
import frc.robot.subystems.drive.GyroIO;
import frc.robot.subystems.drive.GyroIOPigeon2;
import frc.robot.subystems.drive.ModuleIO;
import frc.robot.subystems.drive.ModuleIOSim;
import frc.robot.subystems.drive.ModuleIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
    Limelight vision = new Limelight();
   IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  AmpSubsystem m_ampSubsystem = new AmpSubsystem();
  PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  FeederSubsystem m_feederSubsystem = new FeederSubsystem();
  LEDSubsystem m_ledSubsystemSubsystem = new LEDSubsystem(m_intakeSubsystem, m_shooterSubsystem, m_ampSubsystem, m_feederSubsystem, null);
  private final GyroIOPigeon2 gyro = new GyroIOPigeon2(true);

  // Auto Chooser
  private SendableChooser<Command> autoChooser;

  private boolean speakerMode = true;
  private boolean ampMode = false;
  private boolean manualMode = false;

  private BooleanSupplier speakerModeSupplier = () -> speakerMode;
  private BooleanSupplier ampModeSupplier = () -> ampMode;

  // Controllers
  public static CommandXboxController m_driverCtrl = new CommandXboxController(0);
  private final CommandXboxController m_operatorCtrl = new CommandXboxController(1);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIOPigeon2(true),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));
            Shuffleboard.getTab("Subsystems").add("Swerve", drive);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
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
        break;
       
    }

    NamedCommands.registerCommand("shootSub", new Shoot(m_shooterSubsystem, m_pivotSubsystem, vision, m_feederSubsystem, true).withTimeout(5));
    NamedCommands.registerCommand("intakeShooter", new IndexToShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem, false).andThen(new RefeedShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem)).andThen(new SlowRefeed(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem)).withTimeout(5));
    configureAutos();
    configureButtonBindings();
  }

  private void configureAutos() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Choser", autoChooser);

  }

  public void configureButtonBindings() {


    // Drive Controls
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_driverCtrl.getLeftY(),
            () -> -m_driverCtrl.getLeftX(),
            () -> ((m_driverCtrl.y().getAsBoolean() || m_driverCtrl.rightBumper().getAsBoolean()) && speakerMode) ? AprilTagLock.getR() : -m_driverCtrl.getRightX()));


       m_operatorCtrl.y().onTrue(runOnce(() -> {
      speakerMode = true;
      ampMode = false;
    }));

     m_operatorCtrl.rightBumper().whileTrue(new IntakeNote(m_intakeSubsystem));

    // m_operatorCtrl.y().whileTrue(new IndexToShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem, true).andThen(new RefeedShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem)).andThen(new SlowRefeed(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem))).onFalse(runOnce(() -> m_intakeSubsystem.stop()).alongWith(runOnce(() -> m_ampSubsystem.stop())).alongWith(runOnce(() -> m_feederSubsystem.stop())));
    // m_operatorCtrl.a().whileTrue(new IndexToAmp(m_intakeSubsystem, m_ampSubsystem, m_elevatorSubsystem, m_feederSubsystem, true)).onFalse(runOnce(() -> m_intakeSubsystem.stop()).alongWith(runOnce(() -> m_ampSubsystem.stop())).alongWith(runOnce(() -> m_feederSubsystem.stop())));

    m_operatorCtrl.a().onTrue(runOnce(() -> {
      speakerMode = false;
      ampMode = true;
    }));

    // reset the field-centric heading on start button press
    m_driverCtrl.start().onTrue(runOnce(() -> gyro.zeroGyro()));

      m_driverCtrl.a().whileTrue(new IntakeNote(m_intakeSubsystem));
     // m_driverCtrl.rightBumper().whileTrue(new IntakeNote(m_intakeSubsystem));
      m_operatorCtrl.y().whileTrue(new IndexToShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem, false).andThen(new RefeedShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem)).andThen(new SlowRefeed(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem))).onFalse(runOnce(() -> m_intakeSubsystem.stop()).alongWith(runOnce(() -> m_ampSubsystem.stop())).alongWith(runOnce(() -> m_feederSubsystem.stop())));
      m_operatorCtrl.a().whileTrue(new IndexToAmp(m_intakeSubsystem, m_ampSubsystem, m_elevatorSubsystem, m_feederSubsystem, false)).onFalse(runOnce(() -> m_intakeSubsystem.stop()).alongWith(runOnce(() -> m_ampSubsystem.stop())).alongWith(runOnce(() -> m_feederSubsystem.stop())));
    m_driverCtrl.povUp().whileTrue(new Lob(m_shooterSubsystem, m_pivotSubsystem, m_feederSubsystem, false)).onFalse(runOnce(() -> m_shooterSubsystem.stop()));
    m_driverCtrl.y().and(speakerModeSupplier).whileTrue(new Shoot(m_shooterSubsystem, m_pivotSubsystem, vision, m_feederSubsystem, false)).onFalse(runOnce(() -> m_shooterSubsystem.stop()));
    m_driverCtrl.povDown().onTrue(runOnce(() -> m_feederSubsystem.setFeederVoltage(10))).onFalse(runOnce(() -> m_feederSubsystem.stop()));
    // m_driverCtrl.a().and(speakerModeSupplier).whileTrue(new IndexToShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem, false).andThen(new RefeedShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem)).andThen(new SlowRefeed(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem))).onFalse(runOnce(() -> m_intakeSubsystem.stop()).alongWith(runOnce(() -> m_ampSubsystem.stop())).alongWith(runOnce(() -> m_feederSubsystem.stop())));
    // m_driverCtrl.a().and(ampModeSupplier).whileTrue(new IndexToAmp(m_intakeSubsystem, m_ampSubsystem, m_elevatorSubsystem, m_feederSubsystem, false)).onFalse(runOnce(() -> m_intakeSubsystem.stop()).alongWith(runOnce(() -> m_ampSubsystem.stop())).alongWith(runOnce(() -> m_feederSubsystem.stop())));
    m_driverCtrl.x().onTrue(runOnce(() -> m_pivotSubsystem.setAngle(Rotation2d.fromDegrees(52))));
    m_driverCtrl.b().onTrue(runOnce(() -> m_pivotSubsystem.setAngle(Rotation2d.fromDegrees(0))));
    m_operatorCtrl.povUp().onTrue(runOnce(() -> m_elevatorSubsystem.setHeight(.13)));
     m_operatorCtrl.povRight().onTrue(runOnce(() -> m_elevatorSubsystem.setHeight(.149)));
    m_operatorCtrl.povDown().onTrue(runOnce(() -> m_elevatorSubsystem.setHeight(0)));
    //  m_driverCtrl.leftBumper().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(rpm.get()))).onFalse(runOnce(() -> m_shooterSubsystem.stop()));
    m_driverCtrl.y().and(ampModeSupplier).whileTrue(runOnce(() -> m_ampSubsystem.setAmpVoltage(12))).onFalse(runOnce(() -> m_ampSubsystem.stop()));
    m_operatorCtrl.leftBumper().and(ampModeSupplier).whileTrue(runOnce(() -> m_ampSubsystem.setAmpVoltage(-10)).alongWith(runOnce(() -> m_feederSubsystem.setFeederVoltage(-10))).alongWith(runOnce(() -> m_shooterSubsystem.setShooterVoltage(-10))).alongWith(runOnce(() -> m_intakeSubsystem.setIntakeVoltage(-10)))).onFalse(runOnce(() -> m_ampSubsystem.stop()).alongWith(runOnce(() -> m_feederSubsystem.stop())).alongWith(runOnce(() -> m_intakeSubsystem.stop())).alongWith(runOnce(() -> m_shooterSubsystem.setShooterVoltage(0))));        
    m_operatorCtrl.leftBumper().and(speakerModeSupplier).whileTrue(runOnce(() -> m_ampSubsystem.setAmpVoltage(10)).alongWith(runOnce(() -> m_feederSubsystem.setFeederVoltage(-10))).alongWith(runOnce(() -> m_shooterSubsystem.setShooterVoltage(-10))).alongWith(runOnce(() -> m_intakeSubsystem.setIntakeVoltage(-10)))).onFalse(runOnce(() -> m_ampSubsystem.stop()).alongWith(runOnce(() -> m_feederSubsystem.stop())).alongWith(runOnce(() -> m_intakeSubsystem.stop())).alongWith(runOnce(() -> m_shooterSubsystem.setShooterVoltage(0))));        
  }


  public void checkSensors() {

    SmartDashboard.putBoolean("Speaker Mode", speakerMode);
    SmartDashboard.putBoolean("Amp Mode", ampMode);
    SmartDashboard.putBoolean("Manual Mode", manualMode);

  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}