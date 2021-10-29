// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TimedDrive;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoDrive2;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain _driveTrain;
  private final Joystick _leftJoystick;
  private final Joystick _rightJoystick;
  private final TankDrive _tankDrive;
  private final ArcadeDrive _arcadeDrive;
  private final TimedDrive _timedDrive;
  private final AutoDrive _autoDrive;
  private final AutoDrive2 _autoDrive2;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    _driveTrain = new DriveTrain();
    _leftJoystick = new Joystick(Constants.USBOrder.Zero);
    _rightJoystick = new Joystick(Constants.USBOrder.One);
    _tankDrive = new TankDrive(_driveTrain, _leftJoystick, _rightJoystick);
    _arcadeDrive = new ArcadeDrive(_driveTrain, _leftJoystick);
    _timedDrive = new TimedDrive(_driveTrain);
    _autoDrive = new AutoDrive(_driveTrain);
    _autoDrive2= new AutoDrive2(_driveTrain);
    // commented out to run autodrive as autonomous
   // _driveTrain.setDefaultCommand(_timedDrive);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return _autoDrive2;
  }
}
