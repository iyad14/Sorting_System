from numpy.core.fromnumeric import shape
import cv2
import time
import RPi.GPIO as IO
import numpy as np
from tkinter import *


# start capturing video using using the camera
# setting the resolution and frame rate of the video
video = cv2.VideoCapture(0)
video.set(3, 1280)
video.set(4, 1024)
video.set(5, 30)

# code for stepper motor control
P_A1 = 7  # adapt to your wiring
P_A2 = 11  # ditto
P_B1 = 16  # ditto
P_B2 = 18  # ditto
delay = 0.005  # time to settle
servopwm = 13


# Window method for user interface
def openWindow():
    window = Tk()

    # set background colors,fonts, size of elements of gui
    window.configure(bg="black")
    window.geometry("500x400+400+200")
    shpbtn = Button(window, text='Shape sort')
    clrbtn = Button(window, text='Color sort')
    shpbtn.place(x=325, y=200)
    clrbtn.place(x=325, y=500)
    label = Label(window, text="Choose the mode of sorting: ")
    label.config(bg='#000000')
    label.config(fg="#ffffff")
    shpbtn.config(command=lambda: [window.destroy(), Shape()])  # performs a shape sort when clicked and closes pop window
    clrbtn.config(command=lambda: [window.destroy(), Color()])  # performs a color sort when clicked and closes pop window
    shpbtn.config(font=('Arial', 25, 'bold'))
    shpbtn.config(bg='#368f46')
    shpbtn.config(fg='#000000')
    label.config(font=('Arial', 15))
    clrbtn.config(font=('Arial', 25, 'bold'))
    clrbtn.config(bg='#368f46')
    clrbtn.config(fg='#000000')

    # pack the elements on the window with a certain padding
    label.pack(pady=50, padx=25)
    shpbtn.pack(pady=5)
    clrbtn.pack(pady=5)

    window.mainloop()


# defining the sequence for the stepper motor forward rotation
def forwardStep():
    setStepper(1, 0, 0, 0)
    setStepper(1, 1, 0, 0)
    setStepper(0, 1, 0, 0)
    setStepper(0, 1, 1, 0)
    setStepper(0, 0, 1, 0)
    setStepper(0, 0, 1, 1)
    setStepper(0, 0, 0, 1)
    setStepper(1, 0, 0, 1)


# defining the sequence for the stepper motor backward rotation
def backwardStep():
    setStepper(1, 0, 0, 1)
    setStepper(0, 0, 0, 1)
    setStepper(0, 0, 1, 1)
    setStepper(0, 0, 1, 0)
    setStepper(0, 1, 1, 0)
    setStepper(0, 1, 0, 0)
    setStepper(1, 1, 0, 0)
    setStepper(1, 0, 0, 0)


# setting the pins used to connect the stepper motor
def setStepper(in1, in2, in3, in4):
    IO.setmode(IO.BOARD)
    IO.output(P_A1, in1)
    IO.output(P_A2, in2)
    IO.output(P_B1, in3)
    IO.output(P_B2, in4)
    time.sleep(delay) #delay until the object is dropped by the servo
    return 


# function to stimulate direction and degree of rotation of stepper motor
# 512 step for 360 degrees
def stepper(direction, steps):
    IO.setmode(IO.BOARD)
    IO.setwarnings(False)
    IO.setup(7, IO.OUT)  # stepper PA1
    IO.setup(11, IO.OUT)  # stepper PA2
    IO.setup(16, IO.OUT)  # stepper PB1
    IO.setup(18, IO.OUT)  # stepper PB2

    # conditional for loop that runs the code for the number of steps given
    # in the function arguments
    # the direction given as an argument is checked to specify direction mode of the motor
    for i in range(steps):
        if direction == "forward":
            forwardStep()
        else:
            backwardStep()


# function to rotate the head of the servo to drop object in desired location
# first we rotate the motor one half rotation(180 degrees) then return it back(0)
def drop():
    servo_motor(180)
    IO.setmode(IO.BOARD)
    IO.setup(servopwm, IO.OUT)
    servo_motor(0)


# code for servo motor functionality
# setting up all output and pwm pins needed
def servo_motor(angle):
    # Then, setup the GPIO mode as BOARD so that you can reference the PINs and not the BCM pins.
    IO.setup(servopwm, IO.OUT)
    # Next, create a variable for the servo. I called mine “PWM.” Send a 50 Hz PWM signal on that GPIO pin
    # using the GPIO.PWM() function.
    # #Start the signal at 0.
    pwm = IO.PWM(servopwm, 50)
    pwm.start(0)
    
    # calculate duty cycle using the angle given
    duty = angle / 18 + 3
    IO.output(servopwm, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1) #small delay for the object ot be dropped smoothly
    IO.output(servopwm, False)
    pwm.ChangeDutyCycle(duty)
    # Lastly, clean up the code by stopping the PWM signal and running the cleanup function on the GPIO pins.
    pwm.stop()
    IO.cleanup()


# conveyor function to run the conveyor
def conveyor(duty_cycle1, front, back, duration=0):

    #setting the pins used for running the conveyor via the motor driver
    inA = 36
    inB = 37
    # pin 36 connected to IN1 of the motor drive
    IO.output(inA, back)
    # pin 37 connected to IN2 of the motor drive
    IO.output(inB, front)
    # pin 33 connected to EN1 of the motor drive
    pwm1 = IO.PWM(33, 100)
    pwm1.start(duty_cycle1)

    time.sleep(duration)
    IO.output(inA, 1)
    IO.output(inB, 1)


# function for checking the color of the object
def checkColor():
    color = "none"

    for i in range(20):

        # take a frame of the running video
        # convert colors from BGT to HSV in order to make use of openCV methods
        _, frame = video.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        # setting code for green color detection and ranges of light and dark green
        low_green = np.array([40, 150, 20])
        high_green = np.array([70, 255, 255])
        green_mask = cv2.inRange(hsv_frame, low_green, high_green)  #mask all pixels that are not green range
        green_sum = np.sum(green_mask) #sum of pixels detected to use as a condition below
        # green = cv2.bitwise_and(frame, frame, mask=green_mask)



        # setting code for yellow color detection and ranges of light and dark yellow
        low = np.array([20, 100, 100])
        high = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_frame, low, high)
        sum_yellow = np.sum(yellow_mask)
        # yellow = cv2.bitwise_and(frame, frame, mask=yellow_mask)

        # for verification of the color:
        # first make sure either a yellow or green color is detected
        if green_sum > 1500 or sum_yellow > 1500:
            if green_sum > sum_yellow:              #the larger pixel sum indicates the color
                print("green")
                color = "green"
                return color
            else:
                print("yellow")
                color = "yellow"
                return color

    #     cv2.imshow("Green", green)
    #     cv2.imshow("Yellow", yellow)

    return color


#def nothing(x):
    # any operation
#    pass



# check shape function
def checkShape():
    shape = "none"

    #trackbars to tune the readings

    #     cv2.namedWindow("Trackbars")
    #     cv2.createTrackbar("L-H", "Trackbars", 0, 180, nothing)
    #     cv2.createTrackbar("L-S", "Trackbars", 66, 255, nothing)
    #     cv2.createTrackbar("L-V", "Trackbars", 134, 255, nothing)
    #     cv2.createTrackbar("U-H", "Trackbars", 180, 180, nothing)
    #     cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
    #     cv2.createTrackbar("U-V", "Trackbars", 243, 255, nothing)

    # take a frame of the running video
    # set a font for the contours to be drawn around object
    #font = cv2.FONT_HERSHEY_COMPLEX
    _, frame = video.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # convert colors from BGT to HSV in order to make use of openCV methods

    #     l_h = cv2.getTrackbarPos("L-H", "Trackbars")
    #     l_s = cv2.getTrackbarPos("L-S", "Trackbars")
    #     l_v = cv2.getTrackbarPos("L-V", "Trackbars")
    #     u_h = cv2.getTrackbarPos("U-H", "Trackbars")
    #     u_s = cv2.getTrackbarPos("U-S", "Trackbars")
    #     u_v = cv2.getTrackbarPos("U-V", "Trackbars")


    # set the ranges tuned from the trackbars as default
    lower_red = np.array([0, 66, 62])
    upper_red = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red) #mask all not in light range

    # we define a 5x5 kernel of 1s to multiply the pixels by 1
    # hence the pixels needed are left the same
    # this is to erode the region of the object detected to draw contours properly
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel)

    # Contours detection
    if int(cv2.__version__[0]) > 3:
        # Opencv 4.x.x
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  #find the contours on eroded shape of object
    else:
        # Opencv 3.x.x
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    #as long as there are contours run the loop
    for cnt in contours:
        area = cv2.contourArea(cnt) #find area of object surface
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True) #approximate the contour with an accuracy factor of 0.02

        #if the area detected is large enough, hence an object is detected, draw the contours
        if area > 3000:
            cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)
            if len(approx) == 3: # if 3 lines made the contour hence triangle
                print("Triangle")
                return "Triangle"
            elif  len(approx)==6 : # if 4 or 5 lines made the contour hence rectangle
                print("Rectangle")
                return "Square"
            else: #if neither a rectangle nor a triangle was detected then the object shape is rejected
                #rejected items variable is called circle to match shapes used with prototype
                print("Circle")
                return "Circle"
    #
    #             cv2.imshow("Frame", frame)
    #             cv2.imshow("Mask", mask)

    cv2.destroyAllWindows()


# Calling code portion for color detection
def Color():

    # call checkColor to check color of the object
    color = checkColor()

    # Based on the returned color the stepper is turned on
    # at a backward direction with a number of steps leading to desired sorting destination
    # after dropping the object using the servo, return the arm to initial position
    # using stepper function at a forward direction
    # Turns on the conveyor belt at a certain conveyor duty for a certain duration

    if color == 'green':
        # We turn on the green LED
        greenlight(1)
        conveyor(conveyor_duty, 1, 0, conveyor_duration)
        stepper("backward", 56)
        drop()
        stepper('forward', 56)
        # We turn off the green LED
        greenlight(0)

    # If the color detected is yellow
    elif color == 'yellow':
        yellowlight(1)
        conveyor(conveyor_duty, 1, 0, conveyor_duration)
        stepper("backward", 112)
        drop()
        stepper('forward', 112)
        # We turn off the yellow LED
        yellowlight(0)
    else:
        # We turn on the red LED
        redlight(1)
        conveyor(conveyor_duty, 0, 1, conveyor_duration)
        # We turn off the red LED
        redlight(0)



# Calling code portion for shape detection
def Shape():
    shape = "none"

    #call check shape function
    shape = checkShape()
    color = "none"
    #call checkcolor to turn colors accordingly
    color = checkColor()

    # based on the returned shape, we turn the proper LED
    # move the stepper for a proper steps
    if shape == 'Triangle':
        conveyor(50, 1, 0, 0.7)
        if color == "green":
            greenlight(1)
        elif color == "yellow":
            yellowlight(1)
        else:
            redlight(1)
        conveyor(conveyor_duty, 1, 0, conveyor_duration)
        stepper("backward", 168)
        drop()
        stepper('forward', 168)
        if color == "green":
            greenlight(0)
        elif color == "yellow":
            yellowlight(0)
        else:
            redlight(0)


    elif shape == 'Square':
        conveyor(conveyor_duty, 1, 0, conveyor_duration)
        if color == "green":
            greenlight(1)
        elif color == "yellow":
            yellowlight(1)
        else:
            redlight(1)
        conveyor(conveyor_duty, 1, 0, conveyor_duration)
        stepper("backward", 224)
        drop()
        stepper('forward', 224)
        if color == "green":
            greenlight(0)
        elif color == "yellow":
            yellowlight(0)
        else:
            redlight(0)


    else:
        if color == "green":
            greenlight(1)
        elif color == "yellow":
            yellowlight(1)
        redlight(1)
        conveyor(conveyor_duty, 0, 1, conveyor_duration)
        if color == "green":
            greenlight(0)
        elif color == "yellow":
            yellowlight(0)
        redlight(0)


#functions for turning  the LEDs on/off occurding to function argument
def greenlight(i):
    IO.setup(38, IO.OUT)  # green LED
    IO.output(38, i)


def yellowlight(i):
    IO.setup(29, IO.OUT)  # yellow LED
    IO.output(29, i)


def redlight(i):
    IO.setup(31, IO.OUT)  # red LED
    IO.output(31, i)


# Main method
# setting proper mode to BOARD and setting up conveyor inputs
# run the conveyor to place object  under camera
# setting conveyor duration and duty cycle
IO.setmode(IO.BOARD)
IO.setwarnings(False)
IO.setup(13, IO.OUT)  # servopwm
IO.setup(36, IO.OUT)  # Conveyor A
IO.setup(37, IO.OUT)  # Conveyor B
IO.setup(33, IO.OUT)  # conveyor pwmO



conveyor_duration = 0.7
conveyor_duty = 100

conveyor(conveyor_duty, 1, 0, 0.4)
openWindow()
