# 2019_Swerve
Swerve drive code written after the 2019 season.

# IDs
FR motor A (& encoder) 10\
FR motor B             11\
 \
FL motor A (& encoder) 20\
FL motor B             21\
 \
BL motor A (& encoder) 30\
BL motor B             31\
 \
BR motor A (& encoder) 40\
BR motor B             41\


# Controls
Normally, swerve drive control with x and y motion on the left joystick and z rotation on the right joystick.\
Manual control based on the two joysticks directly controlling the two motors if A button held. (Left controls motor A on module FR, right controls motor B on module FR)

# Current Unknowns/Issues
Problem: encoder is assumed to be positive clockwise\
  Solution: inside SwerveModule.java, put a negative in the getEncoderPos() method\
Problem: both motors assumed to be positive clockwise\
  Solution: inside SwerveModule.java, put a negative in the setMotor_() method\
Problem: navX is assumed to be positive clockwise\
  Solution: inside SwerveSystem.java, put a negative in front of the navX.getAngle() (there is only one)\
Problem: no command to switch to field oriented mode yet\
  Solution: add instant command to call Robot.swervy.switchControlModeOrientation() method\


