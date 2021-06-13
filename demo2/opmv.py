#import pyb, sensor, image, time, math

#sensor.reset()
#sensor.set_pixformat(sensor.GRAYSCALE)
#sensor.set_framesize(sensor.QQVGA) # we run out of memory if the resolution is much bigger...
#sensor.skip_frames(time = 2000)
#sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
#sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
#clock = time.clock()

#f_x = (2.8 / 3.984) * 160 # find_apriltags defaults to this if not set
#f_y = (2.8 / 2.952) * 120 # find_apriltags defaults to this if not set
#c_x = 160 * 0.5 # find_apriltags defaults to this if not set (the image.w * 0.5)
#c_y = 120 * 0.5 # find_apriltags defaults to this if not set (the image.h * 0.5)

#def degrees(radians):
   #return (180 * radians) / math.pi
#def dis_in_cm(distance):
    #return distance * 3.3

#uart = pyb.UART(3,9600,timeout_char=1000)
#uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)

#while(True):
    #clock.tick()
    #img = sensor.snapshot()
    #if (img.find_apriltags()):
        #for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11
          #img.draw_rectangle(tag.rect(), color = (255, 0, 0))
          #img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
          ## The conversion is nearly 6.2cm to 1 -> translation
          #print_args = (dis_in_cm(tag.x_translation()), dis_in_cm(tag.y_translation()), dis_in_cm(tag.z_translation()), \
                #degrees(tag.x_rotation()), degrees(tag.y_rotation()), degrees(tag.z_rotation()))
          #x_diff = dis_in_cm(tag.x_translation())
          #z_dis = abs(dis_in_cm(tag.z_translation()))
          #x_range = 16;
          ## Translation units are unknown. Rotation units are in degrees.
          #print("Tx: %f, Ty %f, Tz %f, Rx %f, Ry %f, Rz %f" % print_args)
          #if (z_dis <= 50 and z_dis > 40):
              #x_range = 16
              #if (abs(x_diff) >= x_range and x_diff < 0):
                  #turn_right = 1
                  #turn_left = 0
              #elif (abs(x_diff) >= x_range and x_diff > 0):
                  #turn_right = 0
                  #turn_left = 1
              #else:
                  #turn_right = 0
                  #turn_left = 0
          #elif (z_dis <= 40 and z_dis > 30):
              #x_range = 12
              #if (abs(x_diff) >= x_range and x_diff < 0):
                  #turn_right = 1
                  #turn_left = 0
              #elif (abs(x_diff) >= x_range and x_diff > 0):
                  #turn_right = 0
                  #turn_left = 1
              #else:
                  #turn_right = 0
                  #turn_left = 0
          #elif (z_dis <= 30 and z_dis > 20):
              #x_range = 8
              #if (abs(x_diff) >= x_range and x_diff < 0):
                  #turn_right = 1
                  #turn_left = 0
              #elif (abs(x_diff) >= x_range and x_diff > 0):
                  #turn_right = 0
                  #turn_left = 1
              #else:
                  #turn_right = 0
                  #turn_left = 0
          #elif (z_dis <= 20 and z_dis > 10):
              #x_range = 4
              #if (abs(x_diff) >= x_range and x_diff < 0):
                  #turn_right = 1
                  #turn_left = 0
              #elif (abs(x_diff) >= x_range and x_diff > 0):
                  #turn_right = 0
                  #turn_left = 1
              #else:
                  #turn_right = 0
                  #turn_left = 0
          #else:
              #uart.write(("/stop/run \n").encode())
              #print("stop\n")

          #if (turn_right == 0 and turn_left == 0):
              #uart.write(("/goStraight/run -100 \n").encode())
              #print("straight")
          #elif (turn_right == 1 and turn_left == 0):
              #uart.write(("/turn2/run 70 -0.3 \n").encode())
              #print("right")
          #elif (turn_right == 0 and turn_left == 1):
              #uart.write(("/turn2/run 70 0.3 \n").encode())
              #print("left")
          #else:
              #uart.write(("/stop/run \n").encode())
              #print("stop\n")
          #time.sleep(0.07)
    #else:
        #uart.write(("/stop/run \n").encode())
        #print("stop\n")


THRESHOLD = (0,110) # Grayscale threshold for dark things...
BINARY_VISIBLE = True

import pyb, sensor, image, time, math

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()

uart = pyb.UART(3,9600,timeout_char=1000)
uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)

while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else sensor.snapshot()
    line = img.get_regression([(255,255) if BINARY_VISIBLE else THRESHOLD], False, (20, 0, 120, 20))

    if (line): img.draw_line(line.line(), color = 127)
    theta = line.theta() if(line) else 0
    rho = line.rho() if(line) else 0
    if (rho < 0) :
        theta = abs(theta - 180)

    mid_approx = abs(rho) / math.cos(math.radians(theta))
    if (line) :
        diff_dis = mid_approx - 80
        if (abs(diff_dis) <= 30 and diff_dis < 0 and theta > 60) :  # turn right
            uart.write(("/turn2/run 70 -0.3 \n").encode())
            print("right")
        elif(abs(diff_dis) <= 30 and diff_dis > 0 and theta > 60) : # turn left
            uart.write(("/turn2/run 70 0.3 \n").encode())
            print("left")
        elif (abs(diff_dis) <= 30) :                             # go straight
            uart.write(("/goStraight/run -100 \n").encode())
            print("straight")
        elif (diff_dis > 0) :                                    # turn left
            uart.write(("/turn2/run 70 0.3 \n").encode())
            print("left")
        elif (diff_dis < 0) :                                    # turn right
            uart.write(("/turn2/run 70 -0.3 \n").encode())
            print("right")
        else :
            uart.write(("/stop/run \n").encode())
            #print("stop\n")
        time.sleep(0.07)
        print("FPS %f, theta = %f, diff_dis = %f, mid_approx = %f\n" % (clock.fps(), theta, diff_dis, mid_approx))
    else :
        uart.write(("/stop/run \n").encode())
        print("stop\n")
        time.sleep(0.07)

