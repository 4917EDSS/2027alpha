// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LedSub extends SubsystemBase {
  private final int LEFT_HEADLIGHT_ID = CanSub.createCANId(0, 6, 8, 10);
  private final int ELEVATOR_COLOR_ID = CanSub.createCANId(1, 6, 8, 10);
  private final int ELEVATOR_BLINK_ID = CanSub.createCANId(2, 6, 8, 10);
  private final int RIGHT_HEADLIGHT_ID = CanSub.createCANId(0, 7, 8, 10);
  private final int CLIMB_COLOR_ID = CanSub.createCANId(1, 7, 8, 10);
  private final int CLIMB_BLINK_ID = CanSub.createCANId(2, 7, 8, 10);
  int m_prevElevatorId = 0;
  byte[] m_prevElevatorArr = new byte[0];
  int m_prevClimbId = 0;
  byte[] m_prevClimbArr = new byte[0];
  byte R = 0;
  byte G = 127;
  byte B = 0;

  /** Creates a new LedSub. */
  public LedSub() {

    setElevatorColor(R, G, B);
    // work(R, G, B);
    turnOffHeadlights();
  }

  @Override
  public void periodic() {}

  public void turnOnHeadlights() {
    leftHeadLight(true);
    rightHeadLight(true);
  }

  public void turnOffHeadlights() {
    leftHeadLight(false);
    rightHeadLight(false);
  }

  public void setElevatorColor(Byte R, Byte G, Byte B) {
    byte[] output_data = new byte[8];
    output_data[0] = R;
    output_data[1] = G;
    output_data[2] = B;
    sendToArduino(ELEVATOR_COLOR_ID, output_data);
  }

  public void setClimbColor(Byte R, Byte G, Byte B) {
    byte[] output_data = new byte[8];
    output_data[0] = R;
    output_data[1] = G;
    output_data[2] = B;
    sendToArduino(CLIMB_COLOR_ID, output_data);
  }

  public void blink(byte startR, byte startG, byte startB, byte endR, byte endG, byte endB) {
    byte[] output_data = new byte[8];
    output_data[0] = startR;
    output_data[1] = startG;
    output_data[2] = startB;
    output_data[3] = endR;
    output_data[4] = endG;
    output_data[5] = endB;
    sendToArduino(ELEVATOR_BLINK_ID, output_data);
    sendToArduino(CLIMB_BLINK_ID, output_data);
  }

  public void leftHeadLight(Boolean enable) {
    byte on = 0;
    if(enable) {
      on = 1;
    }
    byte[] output_data = new byte[8];
    output_data[0] = on;
    sendToArduino(LEFT_HEADLIGHT_ID, output_data);
  }

  public void rightHeadLight(Boolean enable) {
    byte on = 0;
    if(enable) {
      on = 1;
    }
    byte[] output_data = new byte[8];
    output_data[0] = on;
    sendToArduino(RIGHT_HEADLIGHT_ID, output_data);
  }

  // private boolean sameAsBefore(int canId, byte[] output_data) {
  //   byte[] prevArr;
  //   int prevId;
  //   if(canId == ELEVATOR_COLOR_ID || canId == ELEVATOR_BLINK_ID) {
  //     prevArr = m_prevElevatorArr;
  //     prevId = m_prevElevatorId;
  //     m_prevElevatorArr = output_data;
  //     m_prevElevatorId = canId;
  //   } else if(canId == CLIMB_COLOR_ID || canId == CLIMB_BLINK_ID) {
  //     prevArr = m_prevClimbArr;
  //     prevId = m_prevClimbId;
  //     m_prevClimbArr = output_data;
  //     m_prevClimbId = canId;
  //   } else {
  //     return false;
  //   }
  //   if(output_data.length != prevArr.length || prevId != canId) {
  //     return false;
  //   }
  //   for(int i = 0; i < prevArr.length; i++) {
  //     if(prevArr[i] != output_data[i]) {
  //       return false;
  //     }
  //   }
  //   return true;
  // }

  private void sendToArduino(int canId, byte[] output_data) {
    // To avoid flooding our CAN Bus if we call this repeatedly, we first check
    // to make sure that this is a genuinely new request.
    // if(sameAsBefore(canId, output_data)) {
    //   return;
    // }
    ByteBuffer targetedMessageID = ByteBuffer.allocateDirect(4);//Must be direct
    targetedMessageID.order(ByteOrder.LITTLE_ENDIAN); //Set order of bytes
    targetedMessageID.asIntBuffer().put(0, canId); //Put the arbID into the buffer
    //Send a message back to the same device
    // Note that NO REPEAT means it will only send the output once. 
    CANJNI.FRCNetCommCANSessionMuxSendMessage(targetedMessageID.getInt(), output_data,
        CANJNI.CAN_SEND_PERIOD_NO_REPEAT);
  }
}
