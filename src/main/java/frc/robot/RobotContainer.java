// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.statemachine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOSpark;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSpark;
import frc.robot.util.AutoTargetUtil;
import me.nabdev.oxconfig.ConfigurableParameter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final Hood hood;
  private final Intake intake;
  private final Spindexer spindexer;
  private final Kicker kicker;
  private final Shooter shooter;
  private final Turret turret;
public static Field2d fieldSim = new Field2d();
  public final StateMachine stateMachine;
  public final AutoTargetUtil autoTargetUtil;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  public static SwerveDriveSimulation driveSimulation = null;
  public IntakeSimulation intakeSimulation = null;

  private ConfigurableParameter<Double> shootAngle =
      new ConfigurableParameter<Double>(45.0, "Shoot Angle");
  private ConfigurableParameter<Double> shootSpeed =
      new ConfigurableParameter<Double>(7.0, "Shoot Speed");

  public Timer simShotTimer = new Timer();

  // private ConfigurableP
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d shooterSim;
  private final MechanismLigament2d hoodSim;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                (pose) -> {});
        hood = new Hood(new HoodIOSpark());
        intake = new Intake(new IntakeIOSpark());
        shooter = new Shooter(new ShooterIOSpark());
        spindexer = new Spindexer(new SpindexerIOSpark());
        kicker = new Kicker(new KickerIOSpark());
        turret = new Turret(new TurretIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        RobotContainer.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));

        this.intakeSimulation =
            IntakeSimulation.InTheFrameIntake(
                // Specify the type of game pieces that the intake can collect
                "Fuel",
                // Specify the drivetrain to which this intake is attached
                driveSimulation,
                // Width of the intake
                Inches.of(16.5),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.FRONT,
                // The intake can hold up to 1 note
                30);

        this.intakeSimulation.startIntake();

        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        drive.setPose(new Pose2d(2, 2, new Rotation2d()));
        hood = new Hood(new HoodIOSim());
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        kicker = new Kicker(new KickerIO() {});
        turret = new Turret(new TurretIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        hood = new Hood(new HoodIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        kicker = new Kicker(new KickerIO() {});
        turret = new Turret(new TurretIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    DriverStation.getGameSpecificMessage();

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoTargetUtil = new AutoTargetUtil(drive);
    stateMachine =
        new StateMachine(
            controller, drive, intake, spindexer, kicker, shooter, turret, hood, autoTargetUtil);

    mech = new Mechanism2d(3, 3);
    root = mech.getRoot("Shooter", 1.5, 0);
    shooterSim = root.append(new MechanismLigament2d("Shooter", 1, 90));
    hoodSim =
        shooterSim.append(
            new MechanismLigament2d("Hood", 0.5, 90.0, 6.0, new Color8Bit(Color.kPurple)));
  }

  public void updateMechanism() {
    SmartDashboard.putData("Mech2d", mech);
    hoodSim.setAngle(hood.getHoodAngle());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
