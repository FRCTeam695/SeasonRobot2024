// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED m_LED;
  private final AddressableLEDBuffer m_LED_Buffer;
  private final StringSubscriber scoreLocationSub;
  private final BooleanSubscriber amplifySub;
  private final NetworkTable table;
  private final NetworkTableInstance inst;
  private String color;

  public LEDSubsystem() {
    // LED
    m_LED = new AddressableLED(0);
    m_LED_Buffer = new AddressableLEDBuffer(21);
    m_LED.setLength(m_LED_Buffer.getLength());
    color = "";
    // NT
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("SideCar");
    scoreLocationSub = table.getStringTopic("Score Location").subscribe("");
    amplifySub = table.getBooleanTopic("Amplify").subscribe(false);

    m_LED.setData(m_LED_Buffer);
    m_LED.start();
    //color = "green";
  }

  public void setColorToWhite() {
    color = "white";
  }

  public void setColorToOrange() {
    color = "orange";
  }

  public void setColorToGreen() {
    color = "green";
  }

  public void turnColorOff() {
    color = "off";
  }

  // RED IF ERROR
  public void updateColor() {
    // OVERRIDE COLOR IF AMPLIFY COMMAND
    if (amplifySub.get()) {
      for (int i = 0; i < m_LED_Buffer.getLength(); ++i) {
        m_LED_Buffer.setRGB(i, 255, 255, 255);
      }
      m_LED.setData(m_LED_Buffer);
      return;
    }
    switch (color) {
      case "off":
        for (int i = 0; i < m_LED_Buffer.getLength(); ++i) {
          m_LED_Buffer.setRGB(i, 0, 0, 0);
        }
        break;
      case "white":
        for (int i = 0; i < m_LED_Buffer.getLength(); ++i) {
          m_LED_Buffer.setRGB(i, 255, 255, 255);
        }
        break;
      case "green":
        for (int i = 0; i < m_LED_Buffer.getLength(); ++i) {
          m_LED_Buffer.setRGB(i, 0, 255, 0);
        }
        break;
      case "orange":
        for (int i = 0; i < m_LED_Buffer.getLength(); ++i) {
          m_LED_Buffer.setRGB(i, 255, 20, 0);
        }
        break;
      default:
        for (int i = 0; i < m_LED_Buffer.getLength(); ++i) {
          m_LED_Buffer.setRGB(i, 255, 0, 0);
        }
        break;
    }
    m_LED.setData(m_LED_Buffer);

  }

  @Override
  public void periodic() {
    updateColor();
  }
}