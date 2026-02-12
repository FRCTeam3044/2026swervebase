package frc.robot.statemachine.States;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.climber.Climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.List;
import java.util.function.DoubleSupplier;

import me.nabdev.oxconfig.ConfigurableClass;
import me.nabdev.oxconfig.ConfigurableClassParam;
import me.nabdev.oxidation.State;
import me.nabdev.oxidation.StateMachineBase;
import me.nabdev.oxidation.util.SmartXboxController;

public class TestState extends State implements ConfigurableClass {
  private final ConfigurableClassParam<Double> shooterPosition = new ConfigurableClassParam<>(this, 0.0,
      "Test Shooter Position");
  private final ConfigurableClassParam<Double> turretPosition = new ConfigurableClassParam<>(this, 0.0,
      "Test Turret Position");
  private final ConfigurableClassParam<Double> hoodPosition = new ConfigurableClassParam<>(this, 0.0,
      "Test Hood Position");

  public TestState(
      StateMachineBase stateMachine, //Setting base variables 
      CommandXboxController controllerOne,
      CommandXboxController controllerTwo,
      Drive drive,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shooter shooter,
      Spindexer spindexer,
      Turret turret,
      Climber climber) {
    super(stateMachine);
    SmartXboxController testControllerOne = new SmartXboxController(controllerOne, loop);
    SmartXboxController testControllerTwo = new SmartXboxController(controllerTwo, loop);

    /*
     * Button Bindings:
     * 
     * When Controller One: Joysticks (Drive)
     * When Controller Two: Joysticks (Move Subsystems Manually)
     * 
     * Aiming Subsystems:
     * - Controller One (Press to go to position)
     * - Controller Two (Press and move joystick to move manually)
     * A = Hood (left joystick Y)
     * B = Shooter (Right joystick Y)
     * X = Turret (Left joystick X)
     * 
     * Other Subsystems:
     * 1LB = Intake Top
     * 1RB = Intake Bottom
     * 1Y = Intake Roller Spin
     * 1LT = Kicker Shoot
     * 1RT = Kicker Block
     * 2LB = Climber Top
     * 2RB = Climber Bottom
     * 2LT = Climber Climb Position
     * 
     * 
     * A = Climber top position
     * B = Climber climb position
     * X = Climber goes to bottom position
     * Y = Climber runs at speed (Left Joystick Y)
     */

     //
    DoubleSupplier rightY = () -> -MathUtil.applyDeadband(controllerOne.getRightY(), DriveCommands.DEADBAND);
    DoubleSupplier leftY = () -> -MathUtil.applyDeadband(controllerOne.getLeftY(), DriveCommands.DEADBAND);
    DoubleSupplier leftX = () -> -MathUtil.applyDeadband(controllerOne.getLeftX(), DriveCommands.DEADBAND);

    testControllerOne.a().whileTrue(hood.setPosition(hoodPosition::get)); //When A is pressed, the hood will move to the position specified by the hoodPosition parameter
    testControllerOne.b().whileTrue(shooter.runSpeed(() -> RPM.of(shooterPosition.get()))); //When B is pressed, the shooter will run at the speed specified by the shooterPosition parameter
    //testControllerOne.x().whileTrue(turret.setAngle(() -> Degrees.of(turretPosition.get()))); //When X is pressed, the turret will move to the angle specified by the turretPosition parameter
    
    //Testing stuff
    testControllerOne.x().and(testControllerOne.y()).whileTrue(climber.climberTop());// When y is pressed, set climber pos to top-------------------------------------------------------
    testControllerOne.b().and(testControllerTwo.b()).whileTrue(climber.climberPulledUp()); // When b is pressed, climber goes to pulled up position
    testControllerOne.x().and(testControllerOne.a()).whileTrue(climber.climberBottom()); // When x is pressed, climber goes to bottom position

    testControllerOne.y(). whileTrue(climber.setSpeedWParameter(leftY)); // Sets the speed of the climber motor to the amount the joystick is moved
    // End testing stuff

    testControllerTwo.a().whileTrue(hood.runPercent(leftY));
    testControllerTwo.b().whileTrue(shooter.runPercent(rightY));
    testControllerTwo.x().whileTrue(turret.runPercent(leftX));

    testControllerOne.leftBumper().or(testControllerTwo.leftBumper()).whileTrue(intake.intakeTop());
    testControllerOne.rightBumper().or(testControllerTwo.rightBumper()).whileTrue(intake.intakeBottom());
    testControllerOne.y().whileTrue(intake.runRollers());
    testControllerOne.leftTrigger().or(testControllerTwo.leftTrigger()).whileTrue(kicker.shootKicker());
    testControllerOne.rightTrigger().or(testControllerTwo.rightTrigger()).whileTrue(kicker.blockKicker());

    // testController.rightTrigger().whileTrue(turret.rotate(null));
    startWhenActive(
        DriveCommands.joystickDrive(
            drive,
            () -> -controllerOne.getLeftY(),
            () -> -controllerOne.getLeftX(),
            () -> -controllerOne.getRightX(),
            false));

  }

  @Override
  public List<ConfigurableClassParam<?>> getParameters() {
    return List.of(shooterPosition, turretPosition, hoodPosition);
  }

  @Override
  public String getKey() {
    return "TestMode";
  }
}
