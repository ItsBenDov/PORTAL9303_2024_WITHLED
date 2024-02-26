// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax topshootmotor;
  private final CANSparkMax bottomshootmotor;
  private final PIDController shooterSpeedController;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topshootmotor = new CANSparkMax(31, MotorType.kBrushless);
    topshootmotor.setInverted(false);
    bottomshootmotor = new CANSparkMax(32, MotorType.kBrushless);
    shooterSpeedController = new PIDController(0, 0, 0);
    //pid-tolerance (1/2)
    shooterSpeedController.setTolerance(Constants.AutoConstants.shootereMotorTolerance);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("topshootmotorspeed",gettopVelocity());
    SmartDashboard.putNumber("bottomshootmotorspeed",getbottomVelocity());
  }
  public void setshootmotor(double topshootingSpeed,double bottomshootingSpeed){
    topshootmotor.set(shooterSpeedController.calculate(gettopVelocity(),topshootingSpeed));
    bottomshootmotor.set(shooterSpeedController.calculate(getbottomVelocity(),bottomshootingSpeed));

  }
  public void setshootmotorPercent(double topshootingSpeed,double bottomshootingSpeed){
    topshootmotor.set(topshootingSpeed);
    bottomshootmotor.set(bottomshootingSpeed);

  }
  public Command setshootspeedCommand(double topshootspeed,double bottomshootspeed){
    return new RunCommand(()-> setshootmotor(topshootspeed,bottomshootspeed));
  }

  public boolean isatSetpoint(){
    //pid-tolerance (2/2)
    return shooterSpeedController.atSetpoint();

    //IF the ".atSetpoint()" function above is not a good tolerance function, use the follow line and "betweenTolerance" function
    /* 
    if((betweenTolerance(getbottomVelocity(), gettopVelocity(), getbottomVelocity())==true)&&(betweenTolerance(getbottomVelocity(), gettopVelocity(), getbottomVelocity())==true)){
      return true;
    } 

    return false;
    */    
  }
  public boolean betweenTolerance(double speed, double acceptableOverSpeed, double acceptableUnderSpeed)
  {
    if(speed<acceptableOverSpeed && speed>acceptableUnderSpeed)
    return true;

    return false;
  }
  
  public void stopmotors(){
    topshootmotor.stopMotor();
  }
  private double gettopVelocity(){
    return topshootmotor.getEncoder().getVelocity();
  }
  private double getbottomVelocity(){
    return bottomshootmotor.getEncoder().getVelocity();
  }
}