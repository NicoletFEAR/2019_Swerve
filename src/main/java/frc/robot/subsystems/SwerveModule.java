/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class SwerveModule extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // MOTORS:
  public TalonSRX motorA;
  public TalonSRX motorB;
  public SensorCollection moduleEncoder;

  public boolean wheelIsFront;


  public SwerveModule(int motorA_ID, int motorB_ID) { // makes a module with its angleMotorSpeedControllerID and driveMotorSpeedControllerID
		wheelIsFront = true;
    
    // MOTORS
    this.motorA = new TalonSRX(motorA_ID);
    this.motorA.setNeutralMode(NeutralMode.Coast);
    this.motorA.configClosedloopRamp(10);

    this.motorB = new TalonSRX(motorB_ID);
    this.motorB.setNeutralMode(NeutralMode.Coast);
    this.motorB.configClosedloopRamp(10);
		
		this.moduleEncoder = this.motorA.getSensorCollection();
		
		//this.ModuleAnglePIDController = new ModuleAnglePID (this); // makes an angle PID controller for this ModuleDriver
		this.moduleEncoder.setQuadraturePosition(0, 10);
    	this.moduleEncoder.setQuadraturePosition(0, 10); // sets the encoder position to 0 when the ModuleDriver is first created
    	// the second value is timeoutMS
    	this.moduleEncoder.setQuadraturePosition(0, 10); // sets the encoder position to 0 when the ModuleDriver is first created
	}
    
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  }

  public double getEncoderPos() {
    return this.moduleEncoder.getQuadraturePosition();
  }

  public void setMotorA(double speed) {
    this.motorA.set(ControlMode.PercentOutput, speed);
  }

  public void setMotorB(double speed) {
    this.motorB.set(ControlMode.PercentOutput, speed);
  }

}
