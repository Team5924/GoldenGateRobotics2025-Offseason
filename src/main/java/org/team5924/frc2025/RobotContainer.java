/*
 * RobotContainer.java
 */

/* 
 * Copyright (C) 2024-2025 Team 5924 - Golden Gate Robotics and/or its affiliates.
 *
 * This file, and the associated project, are offered under the GNU General
 * Public License v3.0. A copy of this license can be found in LICENSE.md
 * at the root of this project.
 *
 * If this file has been separated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2025;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.team5924.frc2025.commands.drive.DriveCommands;
import org.team5924.frc2025.generated.TunerConstantsGamma;
import org.team5924.frc2025.subsystems.climber.Climber;
import org.team5924.frc2025.subsystems.climber.ClimberIO;
import org.team5924.frc2025.subsystems.climber.ClimberIOTalonFX;
import org.team5924.frc2025.subsystems.drive.Drive;
import org.team5924.frc2025.subsystems.drive.GyroIO;
import org.team5924.frc2025.subsystems.drive.GyroIOPigeon2;
import org.team5924.frc2025.subsystems.drive.ModuleIO;
import org.team5924.frc2025.subsystems.drive.ModuleIOSim;
import org.team5924.frc2025.subsystems.drive.ModuleIOTalonFX;
import org.team5924.frc2025.subsystems.pivot.IntakePivot;
import org.team5924.frc2025.subsystems.pivot.IntakePivot.IntakePivotState;
import org.team5924.frc2025.subsystems.pivot.IntakePivotIO;
import org.team5924.frc2025.subsystems.pivot.IntakePivotIOKrakenFOC;
import org.team5924.frc2025.subsystems.rollers.intake.Intake;
import org.team5924.frc2025.subsystems.rollers.intake.Intake.IntakeState;
import org.team5924.frc2025.subsystems.rollers.intake.IntakeIO;
import org.team5924.frc2025.subsystems.rollers.intake.IntakeIOKrakenFOC;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Climber climber;
  private final Intake intake;
  private final IntakePivot intakePivot;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstantsGamma.FrontLeft),
                new ModuleIOTalonFX(TunerConstantsGamma.FrontRight),
                new ModuleIOTalonFX(TunerConstantsGamma.BackLeft),
                new ModuleIOTalonFX(TunerConstantsGamma.BackRight));
        climber = new Climber(new ClimberIOTalonFX());
        intake = new Intake(new IntakeIOKrakenFOC());
        intakePivot = new IntakePivot(new IntakePivotIOKrakenFOC());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstantsGamma.FrontLeft),
                new ModuleIOSim(TunerConstantsGamma.FrontRight),
                new ModuleIOSim(TunerConstantsGamma.BackLeft),
                new ModuleIOSim(TunerConstantsGamma.BackRight));
        climber = new Climber(new ClimberIO() {});
        intake = new Intake(new IntakeIO() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        climber = new Climber(new ClimberIO() {});
        intake = new Intake(new IntakeIO() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        break;
    }

    // Set up auto routines
    boolean isCompetition = true;

    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer
    autoChooser =
        AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) ->
                isCompetition ? stream.filter(auto -> auto.getName().startsWith("2")) : stream);

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

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    // Nope. It's slow mode now.
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY() * Constants.SLOW_MODE_MULTI,
                () -> -driveController.getLeftX() * Constants.SLOW_MODE_MULTI,
                () -> -driveController.getRightX() * Constants.SLOW_MODE_MULTI));

    // Switch to X pattern when X button is pressed
    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // climber -> deploy down, spinn + drive into cage, lifts up inside
    //      controlled w/ operator dpad

    operatorController
        .povUp()
        .onTrue(Commands.runOnce(() -> climber.setState(Climber.ClimberState.LINEUP_FORWARD)));

    operatorController
        .povDown()
        .onTrue(Commands.runOnce(() -> climber.setState(Climber.ClimberState.HANGING)));

    // driver hold right trigger/release -> ground intake down + spin/up
    // operator y -> shoot

    driveController
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.setGoalState(IntakeState.IN);
                  intakePivot.setGoalState(IntakePivotState.INTAKE_FLOOR);
                }));

    driveController
        .rightTrigger()
        .onFalse(
            Commands.runOnce(
                () -> {
                  intake.setGoalState(IntakeState.OFF);
                  intakePivot.setGoalState(IntakePivotState.MOVING);
                }));

    operatorController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.setGoalState(IntakeState.TROUGH_OUT);
                  intakePivot.setGoalState(IntakePivotState.SCORE_TROUGH);
                }));

    operatorController
        .y()
        .onFalse(
            Commands.runOnce(
                () -> {
                  intake.setGoalState(IntakeState.OFF);
                  intakePivot.setGoalState(IntakePivotState.MOVING);
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
