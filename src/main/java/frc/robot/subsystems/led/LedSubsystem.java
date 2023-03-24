// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  public AddressableLED led;

  public AddressableLEDBuffer ledBuffer;

  public LedSubsystem() {
    led = new AddressableLED(1);
    ledBuffer = new AddressableLEDBuffer(50);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
  }

  public AddressableLED getLed() {
    return led;
  }

  public AddressableLEDBuffer getLedBuffer() {
    return ledBuffer;
  }

  public void startLed() {
    led.setData(ledBuffer);
    led.start();
  }

  public void stopLed() {
    led.setData(ledBuffer);
    led.stop();
  }

  public void rainbowLED(int rainbowFirstPixelHue) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the HSV value to led
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    led.setData(ledBuffer);
  }

  public void setLEDColor(int h, int s, int v) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, h, s, v);
    }
    led.setData(ledBuffer);
  }

  public void setUkraine() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      if ((int) i % 2 == 0) {
        ledBuffer.setHSV(i, LEDConstants.blueH, LEDConstants.blueS, LEDConstants.blueV);
      } else {
        ledBuffer.setHSV(i, LEDConstants.yellowH, LEDConstants.yellowS, LEDConstants.yellowV);
      }
    }
    led.setData(ledBuffer);
  }

  public void setLEDBlue() {
    setLEDColor(LEDConstants.blueH, LEDConstants.blueS, LEDConstants.blueV);
  }

  public void setLEDYellow() {
    setLEDColor(LEDConstants.yellowH, LEDConstants.yellowS, LEDConstants.yellowV);
  }

  public void setLEDRed() {
    setLEDColor(LEDConstants.redH, LEDConstants.redS, LEDConstants.redV);
  }

  public void setLEDGreen() {
    setLEDColor(LEDConstants.greenH, LEDConstants.greenS, LEDConstants.greenV);
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
  //   // Increase by certain number to make the rainbow "move" (change from 3 to greater number if
  // needed)

  // }

}
