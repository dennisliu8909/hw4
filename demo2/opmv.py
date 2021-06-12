# 快速线性回归（巡线）例程
#
# 这个例子展示了如何在OpenMV Cam上使用get_regression（）方法来获得
# ROI的线性回归。 使用这种方法，你可以轻松地建立一个机器人，它可以
# 跟踪所有指向相同的总方向但实际上没有连接的线。 在线路上使用
# find_blobs（），以便更好地过滤选项和控制。
#
# 这被称为快速线性回归，因为我们使用最小二乘法来拟合线。然而，这种方法
# 对于任何具有很多（或者甚至是任何）异常点的图像都是不好的，
# 这会破坏线条拟合.

#设置阈值，（0，100）检测黑色线
THRESHOLD = (0,110) # Grayscale threshold for dark things...

#设置是否使用img.binary()函数进行图像分割
BINARY_VISIBLE = True # 首先执行二进制操作，以便您可以看到正在运行的线性回归...虽然可能会降低FPS。

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

    # Returns a line object similar to line objects returned by find_lines() and
    # find_line_segments(). You have x1(), y1(), x2(), y2(), length(),
    # theta() (rotation in degrees), rho(), and magnitude().
    #
    # magnitude() represents how well the linear regression worked. It goes from
    # (0, INF] where 0 is returned for a circle. The more linear the
    # scene is the higher the magnitude.

    # 函数返回回归后的线段对象line，有x1(), y1(), x2(), y2(), length(), theta(), rho(), magnitude()参数。
    # x1 y1 x2 y2分别代表线段的两个顶点坐标，length是线段长度，theta是线段的角度。
    # magnitude表示线性回归的效果，它是（0，+∞）范围内的一个数字，其中0代表一个圆。如果场景线性回归的越好，这个值越大。
    line = img.get_regression([(255,255) if BINARY_VISIBLE else THRESHOLD], False, (20, 0, 120, 20))

    if (line): img.draw_line(line.line(), color = 127)
    theta = line.theta() if(line) else 0
    rho = line.rho() if(line) else 0
    #print("FPS %f, theta = %f, rho = %f, mag = %s\n" % (clock.fps(), theta, rho, str(line.magnitude()) if (line) else "N/A"))
    #uart.write("FPS %f, theta = %f, rho = %f\n" % (clock.fps(), theta, rho))
    if (rho < 0) :
        theta = abs(theta - 180)

    mid_approx = abs(rho) / math.cos(math.radians(theta))
    if (line) :
        diff_dis = mid_approx - 80
        if (abs(diff_dis) <= 30 and diff_dis < 0 and theta > 60) :  # turn right
            uart.write(("/turn/run 70 -0.3 \n").encode())
            print("right")
        elif(abs(diff_dis) <= 30 and diff_dis > 0 and theta > 60) : # turn left
            uart.write(("/turn/run 70 0.3 \n").encode())
            print("left")
        elif (abs(diff_dis) <= 30) :                             # go straight
            uart.write(("/goStraight/run -100 \n").encode())
            print("straight")
        elif (diff_dis > 0) :                                    # turn left
            uart.write(("/turn/run 70 0.3 \n").encode())
            print("left")
        elif (diff_dis < 0) :                                    # turn right
            uart.write(("/turn/run 70 -0.3 \n").encode())
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



# About negative rho values:
# 关于负rho值:
#
# A [theta+0:-rho] tuple is the same as [theta+180:+rho].
# A [theta+0:-rho]元组与[theta+180:+rho]相同
