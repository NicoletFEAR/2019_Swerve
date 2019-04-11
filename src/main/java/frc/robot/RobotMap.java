/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    // SWERVE MOTOR IDs
    public static int frontRightMotorA_ID = 10;
    public static int frontRightMotorB_ID = 11;

    public static int frontLeftMotorA_ID = 20;
    public static int frontLeftMotorB_ID = 21;

    public static int backRightMotorA_ID = 30;
    public static int backRightMotorB_ID = 31;

    public static int backLeftMotorA_ID = 40;
    public static int backLeftMotorB_ID = 41;

    // JOYSTICK DEAD ZONES
    public static double driveJoystickXDeadZone = 0.1;
    public static double driveJoystickYDeadZone = 0.1;
    public static double driveJoystickZDeadZone = 0.1;

    // ROBOT INFO
    public static double wheelLengthDistance = 24.0;
    public static double wheelWidthDistance = 18.5;

    public static double encoderCodesPerRev;

    public static void init() {

    }

}
