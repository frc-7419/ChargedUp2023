/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;

  // private AddressableLED led = new AddressableLED(0);
  // private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
  // private int rainbowFirstPixelHue = 0;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
      new Thread(() -> {
      UsbCamera cam1 = CameraServer.startAutomaticCapture();
      UsbCamera cam2 = CameraServer.startAutomaticCapture();
      CvSink video1 = CameraServer.getVideo(cam1);
      CvSink video2 = CameraServer.getVideo(cam2);
      cam1.setResolution(400, 320);
      cam2.setResolution(400, 320);
      CvSource combined = CameraServer.putVideo("Front view", 800, 320); //edit these values later

      Mat s1 = new Mat();
      Mat s2 = new Mat();
      Mat o = new Mat();
      byte[] rgb1;
      byte[] rgb2;

      while (!Thread.interrupted()) {
        if (video1.grabFrame(s1)==0 || video2.grabFrame(s2)==0) {
          continue;
        }
        rgb1 = new byte[3];
        rgb2 = new byte[3];
        for (int row=0;row<320;row++) {
          for (int col=0;col<400;col++) {
            s1.get(row, col, rgb1);
            s2.get(row, col, rgb2);
            o.put(row, col, rgb1);
            o.put(row, col+400, rgb2);
          }
        }
        combined.putFrame(o);
      }

    }).start();
  }

  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    // led.setLength(ledBuffer.getLength());
    // for (var i = 0; i < ledBuffer.getLength(); i++) {
    //   final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
    //   // Set the HSV value to led
    //   ledBuffer.setHSV(i, hue, 255, 128);
    // }
    // rainbowFirstPixelHue += 3;
    // rainbowFirstPixelHue %= 180;
  }

  @Override
  public void autonomousInit() {
    robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }
  
  @Override
  public void teleopInit() {
    robotContainer.getAutonomousCommand().cancel();
    robotContainer.setDefaultCommands();
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
