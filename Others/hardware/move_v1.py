import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(5, GPIO.OUT)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

def Blink(numTimes,speed):
    for i in range(0,numTimes):
        print("Iteration " + str(i+1))
        GPIO.output(5,True)
        GPIO.output(7,False)
        GPIO.output(11,False)
        GPIO.output(13,True)
        time.sleep(speed)
        GPIO.output(5,True)
        GPIO.output(7,True)
        GPIO.output(11,True)
        GPIO.output(13,True)
        time.sleep(speed)
        GPIO.output(5,False)
        GPIO.output(7,True)
        GPIO.output(11,True)
        GPIO.output(13,False)
        time.sleep(speed)
        GPIO.output(5,False)
        GPIO.output(7,False)
        GPIO.output(11,False)
        GPIO.output(13,False)
        time.sleep(speed)
    print(" Done")
    time.sleep(speed)
    #GPIO.cleanup()
    
iter = 2 #raw_input("Enter numTimes")
spd = 0.1 # raw_input("Enter speed")

Blink(int(iter),float(spd))
