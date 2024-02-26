// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class LEDControl extends SubsystemBase {
private static final int ledDataPort=Constants.LEDConstants.LEDStripDataPort;
private static final int numOfIndividualLeds=Constants.LEDConstants.numOfLEDs;

private static AddressableLED led;
private static AddressableLEDBuffer ledBuffer;

private static SimpleWidget colorWidget;
private static GenericEntry colorWidgetEntry;

//OG-leftovers: public static boolean colorSet=OperatorConstants.colorset;
//OG-leftovers: public static boolean color=OperatorConstants.color; (both are booleans set to false)


  public LEDControl() {

  led= new AddressableLED(ledDataPort);
  ledBuffer= new AddressableLEDBuffer(numOfIndividualLeds);

  led.setLength(numOfIndividualLeds);
  led.setData(ledBuffer);
  led.start();  


  // Create Boolean widget that displays the color
  colorWidget=Constants.LEDConstants.TAB.add("Color",false);
  colorWidget.withPosition(0, 4);

  //simply formating the color black into hexcode for the shuffleboard to read
  String hexColor_black = String.format("#%02x%02x%02x", 0,0,0);
  colorWidget.withProperties(Map.of("colorWhenFalse", hexColor_black));

  colorWidgetEntry = colorWidget.getEntry();
  }

/////////////////////////////////////////
// general usage functions

public static void displayColor_In_ShuffleboardLEDColorWidget(int r, int g, int b){
    // Convert RGB values to hexadecimal
    String hexColor = String.format("#%02x%02x%02x", r, g, b);

    // Set the hexadecimal color value for the Shuffleboard widget
    colorWidget.withProperties(Map.of("colorWhenTrue", hexColor));
    colorWidgetEntry.setBoolean(true);

    //OG-leftovers: colorSet = true;
}

  public static void setLedColor_Specific(int r,int g, int b){
     
    for(int i =0; i<numOfIndividualLeds; i++){
      ledBuffer.setRGB(i, r, g, b);
    }
    displayColor_In_ShuffleboardLEDColorWidget(r,g,b);
  }

//black:
  public static void turnLedColor_off(){
        
    for(int i =0; i<numOfIndividualLeds; i++){
      ledBuffer.setRGB(i, 0,0,0);
    }
    displayColor_In_ShuffleboardLEDColorWidget(0,0,0);
  }
/////////////////////////
// specific color functions

//green yellow red:
   public static void setLedColor_Green(){
    for(int i =0; i<numOfIndividualLeds; i++){
      ledBuffer.setRGB(i, 0,255,0);
    }
    displayColor_In_ShuffleboardLEDColorWidget(0,255,0);
  }

  public static void setLedColor_Yellow(){
    for(int i =0; i<numOfIndividualLeds; i++){
      ledBuffer.setRGB(i, 255,255,0);
    }
    displayColor_In_ShuffleboardLEDColorWidget(255,255,0);
  }

//(1/2) team color vvv
 public static void setLedColor_Red(){
    for(int i =0; i<numOfIndividualLeds; i++){
      ledBuffer.setRGB(i, 255,0,0);
    }
    displayColor_In_ShuffleboardLEDColorWidget(255,0,0);   
  }
//blue:
//(2/2) team color vvv
 public static void setLedColor_Blue(){
    for(int i =0; i<numOfIndividualLeds; i++){
      ledBuffer.setRGB(i, 0,0,255);
    }
    displayColor_In_ShuffleboardLEDColorWidget(0,0,255);   
  }

//white:
  public static void setLedColor_White(){
        
    for(int i =0; i<numOfIndividualLeds; i++){
      ledBuffer.setRGB(i, 255,255,255);
    }
    displayColor_In_ShuffleboardLEDColorWidget(255,255,255);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}