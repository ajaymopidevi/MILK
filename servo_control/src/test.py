import maestro
servo = maestro.Controller()
servo.setAccel(9,4)      #set servo 0 acceleration to 4
servo.setTarget(9,6000)  #set servo to move to center position
servo.setSpeed(9,1)     #set speed of servo 1
x = servo.getPosition(9) #get the current position of servo 1
servo.close()

