// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED m_LED;
  private final AddressableLEDBuffer  m_LED_Buffer;

  public LEDSubsystem() {
    m_LED = new AddressableLED(2);
    m_LED_Buffer = new AddressableLEDBuffer(24);
    m_LED.setLength(m_LED_Buffer.getLength());




    m_LED.setData(m_LED_Buffer);
    m_LED.start();
  }

  public void setColorToOrange(){
    for(int i = 0; i < m_LED_Buffer.getLength(); ++i){
        m_LED_Buffer.setRGB(i, 255, 20, 0);
    }

    m_LED.setData(m_LED_Buffer);
  }

  public void setColorToGreen(){
    SmartDashboard.putNumber("LED", 1);
    for(int i = 0; i < m_LED_Buffer.getLength(); ++i){
        m_LED_Buffer.setRGB(i, 0, 255, 0);
    }

    m_LED.setData(m_LED_Buffer);
  }

  public void turnColorOff(){
    for(int i = 0; i < m_LED_Buffer.getLength(); ++i){
        m_LED_Buffer.setRGB(i, 0, 0, 0);
    }

    m_LED.setData(m_LED_Buffer);
  }
}
