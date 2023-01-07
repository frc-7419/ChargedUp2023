// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  public AddressableLED led;
  public AddressableLEDBuffer ledBuffer;
  public AddressableLED led1;
  public AddressableLEDBuffer ledBuffer1;

  public LEDSubsystem() {
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());
    // led1 = new AddressableLED(1);
    // ledBuffer1 = new AddressableLEDBuffer(60);
    // led1.setLength(ledBuffer1.getLength());
  }

  @Override
  public void periodic() {}
  
  public AddressableLED getLed(){
    return led;
  }

  public AddressableLEDBuffer getLedBuffer(){
    return ledBuffer;
  }

  public void setLEDColor(int red, int green, int blue){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setRGB(i, red, green, blue);
   }
   led.setData(ledBuffer);
  }

  public void startLed(){
    led.setData(ledBuffer);
    led.start();
  }
  
  public void stopLed(){
    led.setData(ledBuffer);
    led.stop();
  }

  public void rainbowLED(int rainbowFirstPixelHue) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the HSV value to led
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by certain number to make the rainbow "move" (change from 3 to greater number if needed)
    
  }

  // public AddressableLED getLed1(){
  //   return led1;
  // }

  // public AddressableLEDBuffer getLedBuffer1(){
  //   return ledBuffer;
  // }

  // public void setLED1Color(int red, int green, int blue){
  //   for (var i = 0; i < ledBuffer.getLength(); i++) {
  //     // Sets the specified LED to the RGB values for red
  //     ledBuffer.setRGB(i, red, green, blue);
  //  }
  //  led.setData(ledBuffer);
  // }
  // public void setLED1HSV(int h, int s, int v) {
  //   for (var i = 0; i < ledBuffer.getLength(); i++) {
  //     ledBuffer.setHSV(i, h, s, v);
  //   }
  // }

  // public void startLed1(){
  //   led.setData(ledBuffer);
  //   led.start();
  // }
  
  // public void stopLed1(){
  //   led.setData(ledBuffer);
  //   led.stop();
  // }

  // public void rainbowLED1(int rainbowFirstPixelHue, int v) {
  //   for (var i = 0; i < ledBuffer.getLength(); i++) {
  //     final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
  //     // Set the HSV value to led
  //     ledBuffer.setHSV(i, hue, 255, v);
  //   }
  //   // Increase by certain number to make the rainbow "move" (change from 3 to greater number if needed)
    
  // }

}
