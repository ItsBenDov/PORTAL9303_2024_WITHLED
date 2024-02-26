// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.LEDControl;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDSettings extends Command 
{
    int r;
    int g;
    int b;
    private final LEDControl LEDsubsystem;

    public LEDSettings(LEDControl LEDsubsystem, int r, int g, int b) 
    {
      this.r=r;
      this.g=g;
      this.b=b;

      this.LEDsubsystem = LEDsubsystem;
        addRequirements(LEDsubsystem);
    }


  public void turnLED_off(){
    LEDsubsystem.turnLedColor_off();

  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LEDsubsystem.setLedColor_Green();

    // ^^^
    //you can set a custom led color using this function:
    //LEDsubsystem.setLedColor_Specific(r, g, b);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}