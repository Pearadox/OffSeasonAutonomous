/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Navx Passthrough: https://pdocs.kauailabs.com/navx-mxp/installation/io-expansion/
 * Navx Port : RoboRio Channel Address
 * 0 : 10
 * 1 : 11
 * 2 : 12
 * 3 : 13
 * 4 : 18
 * 5 : 19
 * 6 : 20
 * 7 : 21
 * 8 : 22
 * 9 : 23
 */
public class RobotMap {
  
  public static boolean enableCameras = true;
  public static boolean gyroDrive = false;

  public static double ticksPerRev = 256;  //ticks
  public static double wheelDiameter = 6./12.;  //ft
  public static double halfTurn = 1090; //ticks per 180 degrees on EACH side

  public static double feetPerTick = Math.PI * wheelDiameter / ticksPerRev;

  public static int leftEncoderA = 6;
  public static int leftEncoderB = 7;
  public static int rightEncoderA = 9;
  public static int rightEncoderB = 8;

  public static int ultrasonicTrig = 10;
  public static int ultrasonicEcho = 11;
  public static int limitSwitchArm = 3;

  public static int CANLeftSlave1Victor = 11;
  public static int CANLeftSlave2Victor = 10;
  public static int CANRightSlave1Victor = 12;
  public static int CANRightSlave2Victor = 13;
  public static int CANLeftMasterTalon = 14;
  public static int CANRightMasterTalon = 16;

  public static int CANIntakeVictor = 20;

  public static int CANArmBLDCSparkMax = 30;
  public static int CANBoxIntakeVictor = 31;

  public static int CANClimberLeftSparkMax = 40;
  public static int CANClimberRightSparkMax = 41;

  public static int IntakeSolenoid1 = 0;
  public static int IntakeSolenoid2 = 4;
  public static int MothDoubleSolenoidForward = 3;
  public static int MothDoubleSolenoidReverse = 2;
  public static int MothDoubleSolenoidForward2 = 6 ;
  public static int MothDoubleSolenoidReverse2 = 7;
}