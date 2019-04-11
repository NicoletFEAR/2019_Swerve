/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;

/**
 * Add your docs here.
 */
public class OI {

    public XboxController xbox1; // the drive controller // input 0 on driver station
    private Button xbox1LeftStick;
    private Button xbox1RightStick;
    private Button xbox1LBumper;
    private Button xbox1RBumper;
    private Button xbox1Start;
    private Button xbox1Back;
    private Button xbox1X;
    private Button xbox1Y;
    private Button xbox1B;
    private Button xbox1A;

    public OI() {

        // XBOX 1
        xbox1 = new XboxController(0);

        xbox1A = new JoystickButton(xbox1, 1);
        xbox1B = new JoystickButton(xbox1, 2);
        xbox1X = new JoystickButton(xbox1, 3);
        xbox1Y = new JoystickButton(xbox1, 4);
        xbox1LBumper = new JoystickButton(xbox1, 5);
        xbox1RBumper = new JoystickButton(xbox1, 6);
        xbox1Back = new JoystickButton(xbox1, 7);
        xbox1Start = new JoystickButton(xbox1, 8);
        xbox1LeftStick = new JoystickButton(xbox1, 9);
        xbox1RightStick = new JoystickButton(xbox1, 10);

    }

    public XboxController getXbox1() {
        return xbox1;
    }

}
