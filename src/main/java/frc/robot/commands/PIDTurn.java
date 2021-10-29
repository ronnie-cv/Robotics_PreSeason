// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class PIDTurn extends CommandBase {
  private AHRS navx = new AHRS(SPI.Port.kMXP);
  private final DriveTrain driveTrain;
  private double angle;
  private double speed;
  private double error;
  /** Creates a new Turn. */
  public PIDTurn(DriveTrain dt, double a, double s) {
    driveTrain = dt;
    angle = a;
    speed = s;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    navx.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = angle - driveTrain.getAngle();
    error = (error/angle);
    speed = error * 0.7;
    if (speed > 0.7){
      speed = 0.7;
    }
    if (speed < 0.1){
      speed = 0.1;
    }
    if (navx.getAngle() < 0){
      driveTrain.tankDrive(-speed, speed);
    } else if (navx.getAngle() > 0){
      driveTrain.tankDrive(speed, -speed); 
    } else{
      driveTrain.tankDrive(0,0);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Current Angle: " + navx.getAngle());
    return Math.abs(navx.getAngle()) > Math.abs(angle);
  }
}
