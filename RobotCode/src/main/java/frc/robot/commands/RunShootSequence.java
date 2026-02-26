// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallHandlingSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunShootSequence extends Command {
  private final BallHandlingSubsystem m_BallHandlingSubsystem;

  /** Creates a new RunShootSequence. */
  public RunShootSequence(BallHandlingSubsystem ballHandlingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballHandlingSubsystem);
    m_BallHandlingSubsystem = ballHandlingSubsystem;
    Preferences.setDouble("ShootRPM",Preferences.getDouble("ShootRPM", 2900));
  }

  private final Timer currentTimer = new Timer();
  private double TargetRPM = 0;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO: get distance from the center
    double distance = 3;
    //calculate target rpm by distance
    //TargetRPM = 763 + (217*distance) + (-6.91*distance * distance); //based on 65deg
    TargetRPM = 841 + (405*distance) + (-17.8*distance * distance); //based on 80 deg

    currentTimer.restart();
    double RPM = Preferences.getDouble("ShootRPM", 2900);
    m_BallHandlingSubsystem.setShooterVelocity(RPM);
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // run the shooter
    //m_BallHandlingSubsystem.runShooter(0.55);

    if (m_BallHandlingSubsystem.shooterAtVelocity() ) {
      m_BallHandlingSubsystem.moveBottomFeeder(1);
      m_BallHandlingSubsystem.moveColumnFeeder(.5);
      m_BallHandlingSubsystem.moveColumnKicker(.5); 
      m_BallHandlingSubsystem.runPickup(0.33);
      SmartDashboard.putString("Loop Part", "shooting");
    }
    //if the shooter is no longer at velocity, and has been shooting for 1 second, then stop.
    else{
      m_BallHandlingSubsystem.moveBottomFeeder(0);
      m_BallHandlingSubsystem.moveColumnFeeder(0);
      m_BallHandlingSubsystem.moveColumnKicker(0);
      SmartDashboard.putString("Loop Part", "resetting....");
    }
    

  }


  public void execute2() {
    // run the shooter
    //m_BallHandlingSubsystem.runShooter(0.55);

    double starttime = 2;
    double shoottime = 2;
    //m_BallHandlingSubsystem.setShooterVelocity(TargetRPM);
  SmartDashboard.putNumber("Timer", currentTimer.get());
    //if the shooter is at velocity, and timer is not running, run it.
    if (m_BallHandlingSubsystem.shooterAtVelocity() && currentTimer.isRunning() == false) {
      currentTimer.restart();
      SmartDashboard.putString("Loop Part", "starting timer");
    }
    //if the timer has gone for 0.5 seconds, go anyway.
    else if (currentTimer.hasElapsed(starttime) && currentTimer.hasElapsed(starttime + shoottime) == false){
      m_BallHandlingSubsystem.moveBottomFeeder(1);
      m_BallHandlingSubsystem.moveColumnFeeder(1);
      m_BallHandlingSubsystem.moveColumnKicker(1); 
      m_BallHandlingSubsystem.runPickup(0.33);
      SmartDashboard.putString("Loop Part", "shooting");
    }
    //if the shooter is no longer at velocity, and has been shooting for 1 second, then stop.
    else if( m_BallHandlingSubsystem.shooterAtVelocity() == false && currentTimer.hasElapsed(starttime)) {
      currentTimer.stop();
      m_BallHandlingSubsystem.moveBottomFeeder(0);
      m_BallHandlingSubsystem.moveColumnFeeder(0);
      m_BallHandlingSubsystem.moveColumnKicker(0);
      SmartDashboard.putString("Loop Part", "resetting....");
    }
    else{
      
      SmartDashboard.putString("Loop Part", "n/a");
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_BallHandlingSubsystem.moveBottomFeeder(0);
    m_BallHandlingSubsystem.moveColumnFeeder(0);
    m_BallHandlingSubsystem.moveColumnKicker(0);
    m_BallHandlingSubsystem.setShooterVelocity(0);
    m_BallHandlingSubsystem.runPickup(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
