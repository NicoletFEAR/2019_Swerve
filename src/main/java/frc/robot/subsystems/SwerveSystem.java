/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.OpenLoopSwerve;

/**
 * The SwerveSystem takes in joystick movements and calculates the outputs for
 * each module.
 */
public class SwerveSystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static SwerveModule FR;
  public static SwerveModule FL;
  public static SwerveModule BL;
  public static SwerveModule BR;

  public boolean[] wheelIsFront;

  public static AHRS navX;

  public boolean isFieldOriented;

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new OpenLoopSwerve()); // sets the default command
    // to be our open swerve driving command
  }

  public SwerveSystem() { // makes a swerveSystem with four modules
    navX = new AHRS(SPI.Port.kMXP); // makes the new navX
    navX.reset(); // resets navX

    this.isFieldOriented = (false);
    SmartDashboard.putBoolean("isFieldOriented", isFieldOriented);

    this.FR = new SwerveModule(RobotMap.frontRightMotorA_ID, RobotMap.frontRightMotorB_ID);
    this.FL = new SwerveModule(RobotMap.frontLeftMotorA_ID, RobotMap.frontLeftMotorB_ID);
    this.BL = new SwerveModule(RobotMap.backLeftMotorA_ID, RobotMap.backLeftMotorB_ID);
    this.BR = new SwerveModule(RobotMap.backRightMotorA_ID, RobotMap.backRightMotorB_ID);

    this.wheelIsFront[0] = true;
    this.wheelIsFront[1] = true;
    this.wheelIsFront[2] = true;
    this.wheelIsFront[3] = true;

  }

  public void switchControlModeOrientation() {
    isFieldOriented = !isFieldOriented;
    SmartDashboard.putBoolean("isFieldOriented", isFieldOriented);
  }

  public double[] convertJoystickToIntentAxes() { // takes

    double intentAxes[] = new double[3]; // makes an array of doubles to hold the 3 joystick axes, x, y, and r

    // X AXIS
    if (Math.abs(Robot.oi.getXbox1().getX(GenericHID.Hand.kLeft)) <= RobotMap.driveJoystickXDeadZone) {
      // dead zone on the joystick so it does not move accidentaly
      intentAxes[0] = 0;
    } else {
      intentAxes[0] = Robot.oi.getXbox1().getX(GenericHID.Hand.kLeft); // places axes 1 into index 0 of array
    }

    // Y AXIS
    if (Robot.oi.getXbox1().getY(GenericHID.Hand.kLeft) <= RobotMap.driveJoystickYDeadZone) {
      // dead zone on the joystick so it does not move accidentaly
      intentAxes[1] = 0;
    } else {
      intentAxes[1] = (Robot.oi.getXbox1().getY(GenericHID.Hand.kLeft));
      // places axes 2 into index 1 of array (may have to invert y axis)
    }

    // ROTATION
    if (Math.abs(Robot.oi.getXbox1().getY(GenericHID.Hand.kLeft)) <= RobotMap.driveJoystickZDeadZone) {
      // dead zone on the joystick so it does not move accidentaly
      intentAxes[2] = 0;
    } else {
      intentAxes[2] = Robot.oi.getXbox1().getX(GenericHID.Hand.kRight); // places axes 3 into index 2 of array
    }

    // FIELD ORIENTED
    if (isFieldOriented) { // change the x and y so that forward is field oriented

      double navxAngle = navX.getAngle(); // get the value of the navX

      double OCombinedMag = Math.abs(Math.sqrt((intentAxes[0] * intentAxes[0]) + (intentAxes[1] * intentAxes[1])));
      // magnitude of desired movement (joystick)
      double OCombinedAngle = ((180 / Math.PI) * Math.atan2(intentAxes[1], intentAxes[0]));
      SmartDashboard.putNumber("OldCombinedAngle", OCombinedAngle);

      double NCombinedMag = OCombinedMag;
      double NCombinedAngle = OCombinedAngle + navxAngle;
      SmartDashboard.putNumber("NewCombinedAngle", NCombinedAngle);

      intentAxes[1] = Math.sin((Math.PI / 180) * NCombinedAngle) * NCombinedMag;
      intentAxes[0] = Math.cos((Math.PI / 180) * NCombinedAngle) * NCombinedMag;

      SmartDashboard.putNumber("x", intentAxes[0]);
      SmartDashboard.putNumber("y", intentAxes[1]);

    }

    return intentAxes; // return the three joystick axes

  }

  public double[] convertIntentToDesiredAnglesAndSpeeds(double[] intentAxes) {

    double desiredAnglesAndSpeeds[] = new double[8];
    // makes an array of doubles to hold the 3 joystick axes, x, y, and r

    double x = intentAxes[0];
    double y = intentAxes[1];
    double z = intentAxes[2];

    double L = RobotMap.wheelLengthDistance; // 24.0; FB
    double W = RobotMap.wheelWidthDistance; // 18.5; LR
    double r = Math.sqrt((L * L) + (W * W));

    double a = x - z * (L / r);
    double b = x + z * (L / r);
    double c = y - z * (W / r);
    double e = y + z * (W / r);

    double frontRightSpeed = Math.sqrt((b * b) + (c * c));
    double frontLeftSpeed = Math.sqrt((b * b) + (e * e));
    double backLeftSpeed = Math.sqrt((a * a) + (e * e));
    double backRightSpeed = Math.sqrt((a * a) + (c * c));

    double frontRightAngle = -180 * (Math.atan2(b, c) / Math.PI);
    double frontLeftAngle = -180 * (Math.atan2(b, e) / Math.PI);
    double backLeftAngle = -180 * (Math.atan2(a, e) / Math.PI);
    double backRightAngle = -180 * (Math.atan2(a, c) / Math.PI);

    desiredAnglesAndSpeeds[0] = frontRightSpeed;
    desiredAnglesAndSpeeds[1] = frontRightAngle;

    desiredAnglesAndSpeeds[2] = frontLeftSpeed;
    desiredAnglesAndSpeeds[3] = frontLeftAngle;

    desiredAnglesAndSpeeds[4] = backLeftSpeed;
    desiredAnglesAndSpeeds[5] = backLeftAngle;

    desiredAnglesAndSpeeds[6] = backRightSpeed;
    desiredAnglesAndSpeeds[7] = backRightAngle;

    if (Math.abs(Robot.oi.getXbox1().getX(GenericHID.Hand.kLeft)) <= RobotMap.driveJoystickXDeadZone
        && Math.abs(Robot.oi.getXbox1().getY(GenericHID.Hand.kLeft)) <= RobotMap.driveJoystickYDeadZone
        && Math.abs(Robot.oi.getXbox1().getX(GenericHID.Hand.kRight)) <= RobotMap.driveJoystickZDeadZone) {
      // You dopn't want the wheels to move when you don't want to move
      desiredAnglesAndSpeeds[0] = 0.0;
      desiredAnglesAndSpeeds[1] = 0.0;
      desiredAnglesAndSpeeds[2] = 0.0;
      desiredAnglesAndSpeeds[3] = 0.0;
      desiredAnglesAndSpeeds[4] = 0.0;
      desiredAnglesAndSpeeds[5] = 0.0;
      desiredAnglesAndSpeeds[6] = 0;
      desiredAnglesAndSpeeds[7] = 0;
    }

    return desiredAnglesAndSpeeds;

  }

  public double calculateRotationToAngle(double currentEncoderRaw, double desiredAngle, int whichModule) {

    double ticksPerHalfRot = 833.0;

    double newEncPos = 0;

    double differenceToTargetInDegrees;

    double a = (currentEncoderRaw / ticksPerHalfRot); // example: 1.2 means it has gone 1 half rotation and then 2/10s
                                                      // of a half rotation too far
    double b = (int) a; // get just the integer portion of a
    double c = a - b; // get just the decimal portion of a

    if (Math.abs(a) > 1) { // if its gone past a half rotation

      if (b % 2 == 0) { // if even (has gone around full rotation and on same side)
        newEncPos = (c * ticksPerHalfRot);
      } else { // if odd (has gone around half rotation and therefore on the opposite side)
        if (currentEncoderRaw >= ticksPerHalfRot) { // if it has gone too far counterclockwise
          newEncPos = -ticksPerHalfRot + (c * ticksPerHalfRot);
        } else if (currentEncoderRaw <= -ticksPerHalfRot) { // gone too far clockwise
          newEncPos = ticksPerHalfRot + (c * ticksPerHalfRot);
        }
      }

    } else { // if no continuous needs to happen:
      newEncPos = currentEncoderRaw; // leave it because its in the right range
    }

    // FLIPS ENCODER POS IF WHEEL FACING BACKWARDS:
    if (wheelIsFront[whichModule] == false) { // facing back
      if (newEncPos > 0) { // if it's positive:
        newEncPos = -(ticksPerHalfRot) + newEncPos; // rotates it 180
      } else { // if it's negative:
        newEncPos = (ticksPerHalfRot) + newEncPos; // rotates it 180
      }
    } // END FLIP

    desiredAngle = (desiredAngle) * (RobotMap.encoderCodesPerRev / 360); // converts to encoder ticks
    differenceToTargetInDegrees = (desiredAngle - newEncPos) / (RobotMap.encoderCodesPerRev / 360);
    // calculates distance to desiredAngle from current enc pos and converts to
    // degrees

    // calculate most efficient direction and distance to get to target angle
    if (Math.abs(differenceToTargetInDegrees) > 180) { // if it is more efficient to go around the other way :
      if (differenceToTargetInDegrees < 0) { // if the long way would be negative (clockwise) :
        differenceToTargetInDegrees = 360 + differenceToTargetInDegrees; // counterclockwise (+)
      } else { // if the long way would be positive (counterclockwise) :
        differenceToTargetInDegrees = differenceToTargetInDegrees - 360; // clockwise (-)
      }
    } // if the normal way is OK then differneceToTargetInDegrees is fine as it is

    // FLIPPING WHEELS:
    if (Math.abs(differenceToTargetInDegrees) > 95) { // ideally is 90 CHAAANGE!!!!!!!!!!!!!

      wheelIsFront[whichModule] = !(wheelIsFront[whichModule]); // tell it that you want to flip that wheel!

      // ENCODER FLIPPING: (same as before)
      if (newEncPos > 0) { // if it's positive:
        newEncPos = -(ticksPerHalfRot) + newEncPos; // rotates it 180
      } else { // if it's negative:
        newEncPos = (ticksPerHalfRot) + newEncPos; // rotates it 180
      }

      // RECALCULATE THE DISTANCE TO TARGET ANGLE WITH NEW ENCODER VALUE: (repeat,
      // same as above)
      desiredAngle = (desiredAngle) * (RobotMap.encoderCodesPerRev / 360); // converts to encoder ticks
      differenceToTargetInDegrees = (desiredAngle - newEncPos) / (RobotMap.encoderCodesPerRev / 360);
      // calculates distance to desiredAngle from current enc pos and converts to
      // degrees

      // calculate most efficient direction and distance to get to target angle
      if (Math.abs(differenceToTargetInDegrees) > 180) { // if it is more efficient to go around the other way :
        if (differenceToTargetInDegrees < 0) { // if the long way would be negative (clockwise) :
          differenceToTargetInDegrees = 360 + differenceToTargetInDegrees; // counterclockwise (+)
        } else { // if the long way would be positive (counterclockwise) :
          differenceToTargetInDegrees = differenceToTargetInDegrees - 360; // clockwise (-)
        }
      } // if the normal way is OK then differneceToTargetInDegrees is fine as it is
    } // END OF FLIPPING CODE

    return differenceToTargetInDegrees;

  }

  public void convertDesiredWheelMotionToMotorOutputs(double[] desiredAnglesAndSpeeds) {

    // FR MODULE ---------------------------------------------------------------------------------------
    double angleDist_FR = calculateRotationToAngle(FR.getEncoderPos(), desiredAnglesAndSpeeds[1], 0);
    double idealWheelSpeed_FR = desiredAnglesAndSpeeds[0];
    if (!wheelIsFront[0]) { idealWheelSpeed_FR = -(idealWheelSpeed_FR); } 
    double idealTurningSpeed_FR = 0;
    double turnImportance_FR = 0;

    angleDist_FR = angleDist_FR / 90;

    if (angleDist_FR >= 0) {
      idealTurningSpeed_FR = (1 - ((angleDist_FR - 1) * (angleDist_FR - 1))); // (0,1)
    } else {
      idealTurningSpeed_FR = -(1 - ((-angleDist_FR - 1) * (-angleDist_FR - 1))); // (0,-1)
    }

    turnImportance_FR = Math.abs(angleDist_FR) / 90;
    turnImportance_FR = 1 - (turnImportance_FR - 1) * (turnImportance_FR - 1); // (0,1)

    double wheelSpeedAllowance_FR = 1 - turnImportance_FR; // (0,1)
    // END FR MODULE ----------------------------------------------------------------------------------
    
    // FL MODULE ---------------------------------------------------------------------------------------
    double angleDist_FL = calculateRotationToAngle(FL.getEncoderPos(), desiredAnglesAndSpeeds[1], 0);
    double idealWheelSpeed_FL = desiredAnglesAndSpeeds[0];
    if (!wheelIsFront[1]) { idealWheelSpeed_FL = -(idealWheelSpeed_FL); } 
    double idealTurningSpeed_FL = 0;
    double turnImportance_FL = 0;

    angleDist_FL = angleDist_FL / 90;

    if (angleDist_FL >= 0) {
      idealTurningSpeed_FL = (1 - ((angleDist_FL - 1) * (angleDist_FL - 1))); // (0,1)
    } else {
      idealTurningSpeed_FL = -(1 - ((-angleDist_FL - 1) * (-angleDist_FL - 1))); // (0,-1)
    }

    turnImportance_FL = Math.abs(angleDist_FL) / 90;
    turnImportance_FL = 1 - (turnImportance_FL - 1) * (turnImportance_FL - 1); // (0,1)

    double wheelSpeedAllowance_FL = 1 - turnImportance_FL; // (0,1)
    // END FL MODULE ----------------------------------------------------------------------------------
    
    // BL MODULE ---------------------------------------------------------------------------------------
    double angleDist_BL = calculateRotationToAngle(BL.getEncoderPos(), desiredAnglesAndSpeeds[1], 0);
    double idealWheelSpeed_BL = desiredAnglesAndSpeeds[0];
    if (!wheelIsFront[2]) { idealWheelSpeed_BL = -(idealWheelSpeed_BL); }
    double idealTurningSpeed_BL = 0;
    double turnImportance_BL = 0;

    angleDist_BL = angleDist_BL / 90;

    if (angleDist_BL >= 0) {
      idealTurningSpeed_BL = (1 - ((angleDist_BL - 1) * (angleDist_BL - 1))); // (0,1)
    } else {
      idealTurningSpeed_BL = -(1 - ((-angleDist_BL - 1) * (-angleDist_BL - 1))); // (0,-1)
    }

    turnImportance_BL = Math.abs(angleDist_BL) / 90;
    turnImportance_BL = 1 - (turnImportance_BL - 1) * (turnImportance_BL - 1); // (0,1)

    double wheelSpeedAllowance_BL = 1 - turnImportance_BL; // (0,1)
    // END BL MODULE ----------------------------------------------------------------------------------

    // BR MODULE ---------------------------------------------------------------------------------------
    double angleDist_BR = calculateRotationToAngle(BR.getEncoderPos(), desiredAnglesAndSpeeds[1], 0);
    double idealWheelSpeed_BR = desiredAnglesAndSpeeds[0];
    if (!wheelIsFront[3]) { idealWheelSpeed_BR = -(idealWheelSpeed_BR); }
    double idealTurningSpeed_BR = 0;
    double turnImportance_BR = 0;

    angleDist_BR = angleDist_BR / 90;

    if (angleDist_BR >= 0) {
      idealTurningSpeed_BR = (1 - ((angleDist_BR - 1) * (angleDist_BR - 1))); // (0,1)
    } else {
      idealTurningSpeed_BR = -(1 - ((-angleDist_BR - 1) * (-angleDist_BR - 1))); // (0,-1)
    }

    turnImportance_BR = Math.abs(angleDist_BR) / 90;
    turnImportance_BR = 1 - (turnImportance_BR - 1) * (turnImportance_BR - 1); // (0,1)

    double wheelSpeedAllowance_BR = 1 - turnImportance_BR; // (0,1)
    // END BR MODULE ----------------------------------------------------------------------------------

    double overallWheelSpeedAllowance = 1;
    if (overallWheelSpeedAllowance > wheelSpeedAllowance_FR) {
      overallWheelSpeedAllowance = wheelSpeedAllowance_FR;
    }
    if (overallWheelSpeedAllowance > wheelSpeedAllowance_FL) {
      overallWheelSpeedAllowance = wheelSpeedAllowance_FL;
    }
    if (overallWheelSpeedAllowance > wheelSpeedAllowance_BL) {
      overallWheelSpeedAllowance = wheelSpeedAllowance_BL;
    }
    if (overallWheelSpeedAllowance > wheelSpeedAllowance_BR) {
      overallWheelSpeedAllowance = wheelSpeedAllowance_BR;
    }
    

    // BEGIN FR MODULE ---------------------------------------------------------------------------------------
    double scaledWheelSpeed_FR = overallWheelSpeedAllowance * Math.abs(idealWheelSpeed_FR); // (0,1)

    if (Math.abs(idealTurningSpeed_FR) + scaledWheelSpeed_FR > 1) { // cant have that speed diff with that avg turning
      if (idealTurningSpeed_FR < 0) { // TURNS NEG
        if (idealWheelSpeed_FR > 0) { // wheel speed pos
          FR.setMotorA(-1 + 2 * scaledWheelSpeed_FR);
          FR.setMotorB(-1);
        } else { // wheel speed neg
          FR.setMotorA(-1);
          FR.setMotorB(-1 + 2 * scaledWheelSpeed_FR);
        }
      } else { // TURNS POS
        if (idealWheelSpeed_FR > 0) { // wheel speed pos
          FR.setMotorA(1);
          FR.setMotorB(1 - 2 * scaledWheelSpeed_FR);
        } else { // wheel speed neg
          FR.setMotorA(1 - 2 * scaledWheelSpeed_FR);
          FR.setMotorB(1);
        }
      }
    } else { // avg and diff both ok
      if (idealWheelSpeed_FR > 0) { // wheel speed pos
        FR.setMotorA(idealTurningSpeed_FR + scaledWheelSpeed_FR);
        FR.setMotorB(idealTurningSpeed_FR - scaledWheelSpeed_FR);
      } else { // wheel speed neg
        FR.setMotorA(idealTurningSpeed_FR - scaledWheelSpeed_FR);
        FR.setMotorB(idealTurningSpeed_FR + scaledWheelSpeed_FR);
      }
    }
    // END FR MODULE ---------------------------------------------------------------------------------------

    // BEGIN FL MODULE ---------------------------------------------------------------------------------------
    double scaledWheelSpeed_FL = overallWheelSpeedAllowance * Math.abs(idealWheelSpeed_FL); // (0,1)

    if (Math.abs(idealTurningSpeed_FL) + scaledWheelSpeed_FL > 1) { // cant have that speed diff with that avg turning
      if (idealTurningSpeed_FL < 0) { // TURNS NEG
        if (idealWheelSpeed_FL > 0) { // wheel speed pos
          FL.setMotorA(-1 + 2 * scaledWheelSpeed_FL);
          FL.setMotorB(-1);
        } else { // wheel speed neg
          FL.setMotorA(-1);
          FL.setMotorB(-1 + 2 * scaledWheelSpeed_FL);
        }
      } else { // TURNS POS
        if (idealWheelSpeed_FL > 0) { // wheel speed pos
          FL.setMotorA(1);
          FL.setMotorB(1 - 2 * scaledWheelSpeed_FL);
        } else { // wheel speed neg
          FL.setMotorA(1 - 2 * scaledWheelSpeed_FL);
          FL.setMotorB(1);
        }
      }
    } else { // avg and diff both ok
      if (idealWheelSpeed_FL > 0) { // wheel speed pos
        FL.setMotorA(idealTurningSpeed_FL + scaledWheelSpeed_FL);
        FL.setMotorB(idealTurningSpeed_FL - scaledWheelSpeed_FL);
      } else { // wheel speed neg
        FL.setMotorA(idealTurningSpeed_FL - scaledWheelSpeed_FL);
        FL.setMotorB(idealTurningSpeed_FL + scaledWheelSpeed_FL);
      }
    }
    // END FL MODULE ---------------------------------------------------------------------------------------

    // BEGIN BL MODULE ---------------------------------------------------------------------------------------
    double scaledWheelSpeed_BL = overallWheelSpeedAllowance * Math.abs(idealWheelSpeed_BL); // (0,1)

    if (Math.abs(idealTurningSpeed_BL) + scaledWheelSpeed_BL > 1) { // cant have that speed diff with that avg turning
      if (idealTurningSpeed_BL < 0) { // TURNS NEG
        if (idealWheelSpeed_BL > 0) { // wheel speed pos
          BL.setMotorA(-1 + 2 * scaledWheelSpeed_BL);
          BL.setMotorB(-1);
        } else { // wheel speed neg
          BL.setMotorA(-1);
          BL.setMotorB(-1 + 2 * scaledWheelSpeed_BL);
        }
      } else { // TURNS POS
        if (idealWheelSpeed_BL > 0) { // wheel speed pos
          BL.setMotorA(1);
          BL.setMotorB(1 - 2 * scaledWheelSpeed_BL);
        } else { // wheel speed neg
          BL.setMotorA(1 - 2 * scaledWheelSpeed_BL);
          BL.setMotorB(1);
        }
      }
    } else { // avg and diff both ok
      if (idealWheelSpeed_BL > 0) { // wheel speed pos
        BL.setMotorA(idealTurningSpeed_BL + scaledWheelSpeed_BL);
        BL.setMotorB(idealTurningSpeed_BL - scaledWheelSpeed_BL);
      } else { // wheel speed neg
        BL.setMotorA(idealTurningSpeed_BL - scaledWheelSpeed_BL);
        BL.setMotorB(idealTurningSpeed_BL + scaledWheelSpeed_BL);
      }
    }
    // END BL MODULE ---------------------------------------------------------------------------------------

    // BEGIN BR MODULE ---------------------------------------------------------------------------------------
    double scaledWheelSpeed_BR = overallWheelSpeedAllowance * Math.abs(idealWheelSpeed_BR); // (0,1)

    if (Math.abs(idealTurningSpeed_BR) + scaledWheelSpeed_BR > 1) { // cant have that speed diff with that avg turning
      if (idealTurningSpeed_BR < 0) { // TURNS NEG
        if (idealWheelSpeed_BR > 0) { // wheel speed pos
          BR.setMotorA(-1 + 2 * scaledWheelSpeed_BR);
          BR.setMotorB(-1);
        } else { // wheel speed neg
          BR.setMotorA(-1);
          BR.setMotorB(-1 + 2 * scaledWheelSpeed_BR);
        }
      } else { // TURNS POS
        if (idealWheelSpeed_BR > 0) { // wheel speed pos
          BR.setMotorA(1);
          BR.setMotorB(1 - 2 * scaledWheelSpeed_BR);
        } else { // wheel speed neg
          BR.setMotorA(1 - 2 * scaledWheelSpeed_BR);
          BR.setMotorB(1);
        }
      }
    } else { // avg and diff both ok
      if (idealWheelSpeed_BR > 0) { // wheel speed pos
        BR.setMotorA(idealTurningSpeed_BR + scaledWheelSpeed_BR);
        BR.setMotorB(idealTurningSpeed_BR - scaledWheelSpeed_BR);
      } else { // wheel speed neg
        BR.setMotorA(idealTurningSpeed_BR - scaledWheelSpeed_BR);
        BR.setMotorB(idealTurningSpeed_BR + scaledWheelSpeed_FR);
      }
    }
    // END FR MODULE ---------------------------------------------------------------------------------------

  }

}
