# 导入依赖的模�?
import sensor
import image
import time
import machine
from machine import UART
import pyb
from machine import LED
# 阈�?

#下面三个阈值最为关�?
threshold1=[#red
    (24, 82, 17, 83, -28, 127)
]
threshold2=[#green
    (50, 100, -48, 21, 9, 69)
]
threshold3=[#blue
   (9, 63, -4, 31, -64, -15)
]
thresholds=[threshold1,threshold2,threshold3]

# 初始化设�?
sensor.reset()  # 重置摄像头模块，以确保摄像头处于已知的初始状�?
sensor.set_pixformat(sensor.RGB565)  # 像素格式设置
sensor.set_framesize(sensor.QVGA)  # 分辨率大小设置，QVGA�?320x240
sensor.skip_frames(time=2000)  # 让摄像头跳过前两秒的帧，为了在实际图像捕获之前，摄像头有足够的时间自动调�?
sensor.set_auto_gain(False)  # 必须关闭自动增益以进行颜色跟�?
sensor.set_auto_whitebal(False)  # 必须关闭自动白平衡以进行颜色跟踪
clock = time.clock()
first_r=0
first_b=0
first_g=0
second_r=0
second_b=0
second_g=0
third_r=0
third_g=0
third_b=0
fourth_r=0
fourth_g=0
fourth_b=0
fifth_r=0
fifth_g=0
fifth_b=0
sixth_r=0
sixth_g=0
sixth_b=0
flag = 0
returnlist1=[0,0,0,0,0,0]
returnlist2=[0,0,0,0,0,0]
returnlist3=[0,0,0,0,0,0]
#下面三个judge函数很可能是没用的函�?
def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob = blob
            max_size = blob.pixels()
    return max_blob

i=0
j=0
k=0#i->mode1_count,j->mode2_count,k->mode3_count
uart=UART(3,9600,8,None,1,timeout=1000,read_buf_len=4096)
mode=0#模式选择
flag1=0#mode1_flag
flag2=0#mode2_flag
flag3=0#mode3_flag
led = LED("LED_BLUE")
led1=LED("LED_RED")
led1.on()
def compare_max(a,b,c):#该函数是用于寻找最大值的
    if(a>=b and a>=c):
        return a
    if(b>=a and b>=c):
        return b
    if(c>=a and c>=b):
        return c
def tianchong(list):#通过数值比较最终给识别数组中进行填充的函数
    if(compare_max(first_b,first_g,first_r)==first_r):
        list[0]=3
    if(compare_max(first_b,first_g,first_r)==first_b):
        list[0]=2
    if(compare_max(first_b,first_g,first_r)==first_g):
        list[0]=1
    if(compare_max(second_b,second_g,second_r)==second_r):
        list[1]=3
    if(compare_max(second_b,second_g,second_r)==second_b):
        list[1]=2
    if(compare_max(second_b,second_g,second_r)==second_g):
        list[1]=1
    if(compare_max(third_g,third_b,third_r)==third_b):
        list[2]=2
    if(compare_max(third_g,third_b,third_r)==third_g):
        list[2]=1
    if(compare_max(third_g,third_b,third_r)==third_r):
        list[2]=3
    if(compare_max(fourth_b,fourth_g,fourth_r)==fourth_r):
        list[3]=3
    if(compare_max(fourth_b,fourth_g,fourth_r)==fourth_g):
        list[3]=1
    if(compare_max(fourth_b,fourth_g,fourth_r)==fourth_b):
        list[3]=2
    if(compare_max(fifth_b,fifth_g,fifth_r)==fifth_r):
        list[4]=3
    if(compare_max(fifth_b,fifth_g,fifth_r)==fifth_g):
        list[4]=1
    if(compare_max(fifth_b,fifth_g,fifth_r)==fifth_b):
        list[4]=2
    if(compare_max(sixth_b,sixth_g,sixth_r)==sixth_r):
        list[5]=3
    if(compare_max(sixth_b,sixth_g,sixth_r)==sixth_g):
        list[5]=1
    if(compare_max(sixth_b,sixth_g,sixth_r)==sixth_b):
        list[5]=2
# 进入循环
#在题目中，要求绿色代表轻微干旱，蓝色代表一般干旱，红色代表严重干旱
while True:
    led1.off()
    led.on()
    if uart.any():  # 如果有数据可�?
        mode = int(uart.readline())  # 读取一行数�?
        if mode==1:

            first_r=0
            first_b=0
            first_g=0
            second_r=0
            second_b=0
            second_g=0
            third_r=0
            third_g=0
            third_b=0
            fourth_r=0
            fourth_g=0
            fourth_b=0
            fifth_r=0
            fifth_g=0
            fifth_b=0
            sixth_r=0
            sixth_g=0
            sixth_b=0
        if mode==2:

            first_r=0
            first_b=0
            first_g=0
            second_r=0
            second_b=0
            second_g=0
            third_r=0
            third_g=0
            third_b=0
            fourth_r=0
            fourth_g=0
            fourth_b=0
            fifth_r=0
            fifth_g=0
            fifth_b=0
            sixth_r=0
            sixth_g=0
            sixth_b=0
        if mode==3:

            first_r=0
            first_b=0
            first_g=0
            second_r=0
            second_b=0
            second_g=0
            third_r=0
            third_g=0
            third_b=0
            fourth_r=0
            fourth_g=0
            fourth_b=0
            fifth_r=0
            fifth_g=0
            fifth_b=0
            sixth_r=0
            sixth_g=0
            sixth_b=0
    if mode==1:#中间位置，ID1舵机位置253
            clock.tick()  # 用于更新时钟对象的时钟戳，方便后续计算帧率和测量时间间隔
            img = sensor.snapshot()

            # 更新 flag
            flag += 1
            if flag > 3:
                flag = 1

            # 根据当前 flag 使用相应的阈�?
            threshold = thresholds[flag-1]

            blobs = img.find_blobs(threshold,roi=[105,0,74,21], x_stride=20, y_stride=4, area_threshold=20*4,pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=3
                        first_r+=1
                        i+=1

                    elif flag == 2:#绿色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=1
                        first_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=2
                        first_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[110,35,65,11], x_stride=30, y_stride=6, area_threshold=20*6,pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=3
                        second_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=1
                        second_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=2
                        second_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[107,60,70,13], x_stride=30, y_stride=6,area_threshold=30*6,pixels_threshold=100, merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=3
                        third_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=1
                        third_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=2
                        third_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[105,88,75,14], x_stride=40, y_stride=6, area_threshold=40*6,pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #[judge1(blob.cx(),blob.cy())]=3
                        fourth_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=1
                        fourth_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=2
                        fourth_b+=1
                        i+=1

            #servo ID4:591
            blobs = img.find_blobs(threshold,roi=[100,124,82,13], x_stride=40, y_stride=6,area_threshold=40*6, pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=3
                        fifth_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=1
                        fifth_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=2
                        fifth_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[98,159,91,45], x_stride=40, y_stride=10,area_threshold=40*10, pixels_threshold=200,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=3
                        sixth_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=1
                        sixth_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        #returnlist1[judge1(blob.cx(),blob.cy())]=2
                        sixth_b+=1
                        i+=1

    elif mode==2:#左侧位置,ID1处于385位置
            clock.tick()  # 用于更新时钟对象的时钟戳，方便后续计算帧率和测量时间间隔
            img = sensor.snapshot()

            # 更新 flag
            flag += 1
            if flag > 3:
                flag = 1

            # 根据当前 flag 使用相应的阈�?
            threshold = thresholds[flag-1]

            blobs = img.find_blobs(threshold,roi=[28,5,80,20], x_stride=10, y_stride=5, area_threshold=40,pixels_threshold=40,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=3
                        first_r+=1
                        j+=1
                    elif flag == 2:#绿色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=1
                        first_g+=1
                        j+=1
                    elif flag == 3:#蓝色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=2
                        first_b+=1
                        j+=1
            blobs = img.find_blobs(threshold,roi=[33,38,63,10], x_stride=20, y_stride=5, area_threshold=50,pixels_threshold=70,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=3
                        second_r+=1
                        j+=1
                    elif flag == 2:#绿色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=1
                        second_g+=1
                        j+=1
                    elif flag == 3:#蓝色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=2
                        second_b+=1
                        j+=1
            blobs = img.find_blobs(threshold,roi=[20,64,82,10], x_stride=20, y_stride=5,area_threshold=60, pixels_threshold=60,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=3
                        third_r+=1
                        j+=1
                    elif flag == 2:#绿色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=1
                        third_g+=1
                        j+=1
                    elif flag == 3:#蓝色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=2
                        third_b+=1
                        j+=1
            blobs = img.find_blobs(threshold,roi=[17,91,72,14], x_stride=20, y_stride=6, area_threshold=100,pixels_threshold=70,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=3
                        fourth_r+=1
                        j+=1
                    elif flag == 2:#绿色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=1
                        fourth_g+=1
                        j+=1
                    elif flag == 3:#蓝色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=2
                        fourth_b+=1
                        j+=1

            #servo ID4:591
            blobs = img.find_blobs(threshold,roi=[14,123,70,18], x_stride=30, y_stride=8,area_threshold=200, pixels_threshold=200,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=3
                        fifth_r+=1
                        j+=1
                    elif flag == 2:#绿色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=1
                        fifth_g+=1
                        j+=1
                    elif flag == 3:#蓝色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=2
                        fifth_b+=1
                        j+=1
            blobs = img.find_blobs(threshold,roi=[7,159,75,41], x_stride=40, y_stride=10,area_threshold=300, pixels_threshold=300,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=3
                        sixth_r+=1
                        j+=1
                    elif flag == 2:#绿色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=1
                        sixth_g+=1
                        j+=1
                    elif flag == 3:#蓝色
                        #returnlist2[judge2(blob.cx(),blob.cy())]=2
                        sixth_b+=1
                        j+=1

    elif mode==3:#右侧位置,ID1处于125位置
            clock.tick()  # 用于更新时钟对象的时钟戳，方便后续计算帧率和测量时间间隔
            img = sensor.snapshot()

            # 更新 flag
            flag += 1
            if flag > 3:
                flag = 1

            # 根据当前 flag 使用相应的阈�?
            threshold = thresholds[flag-1]

            blobs = img.find_blobs(threshold,roi=[186,10,70,21], x_stride=10, y_stride=7, area_threshold=200,pixels_threshold=500,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=3
                        first_r+=1
                        k+=1
                    elif flag == 2:#绿色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=1
                        first_g+=1
                        k+=1
                    elif flag == 3:#蓝色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=2
                        first_b+=1
                        k+=1
            blobs = img.find_blobs(threshold,roi=[194,36,62,11], x_stride=10, y_stride=6, area_threshold=200,pixels_threshold=500,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=3
                        second_r+=1
                        k+=1
                    elif flag == 2:#绿色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=1
                        second_g+=1
                        k+=1
                    elif flag == 3:#蓝色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=2
                        second_b+=1
                        k+=1
            blobs = img.find_blobs(threshold,roi=[199,60,64,12], x_stride=10, y_stride=6,area_threshold=300, pixels_threshold=500,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=3
                        third_r+=1
                        k+=1
                    elif flag == 2:#绿色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=1
                        third_g+=1
                        k+=1
                    elif flag == 3:#蓝色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=2
                        third_b+=1
                        k+=1
            blobs = img.find_blobs(threshold,roi=[200,87,69,14], x_stride=10, y_stride=7, area_threshold=400,pixels_threshold=500,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=3
                        fourth_r+=1
                        k+=1
                    elif flag == 2:#绿色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=1
                        fourth_g+=1
                        k+=1
                    elif flag == 3:#蓝色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=2
                        fourth_b+=1
                        k+=1

            #servo ID4:591
            blobs = img.find_blobs(threshold,roi=[205,123,70,13], x_stride=20, y_stride=6,area_threshold=400, pixels_threshold=500,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=3
                        fifth_r+=1
                        k+=1
                    elif flag == 2:#绿色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=1
                        fifth_g+=1
                        k+=1
                    elif flag == 3:#蓝色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=2
                        fifth_b+=1
                        k+=1
            blobs = img.find_blobs(threshold,roi=[212,158,73,35], x_stride=20, y_stride=8,area_threshold=400, pixels_threshold=500,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=3
                        sixth_r+=1
                        k+=1
                    elif flag == 2:#绿色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=1
                        sixth_g+=1
                        k+=1
                    elif flag == 3:#蓝色
                        #returnlist3[judge3(blob.cx(),blob.cy())]=2
                        sixth_b+=1
                        k+=1
    if i>=20:
        if flag1==0:
            tianchong(returnlist1)
            uart.write(bytearray([0x05]))
            for value in returnlist1:
                uart.write(bytearray([value]))
            uart.write(bytearray([0x07]))
            print(returnlist1)

            flag1=1

    if j>=20:
        if flag2==0:
            tianchong(returnlist2)
            uart.write(bytearray([0x05]))
            for value in returnlist2:
                uart.write(bytearray([value]))
            uart.write(bytearray([0x07]))
            print(returnlist2)
            flag2=1

    if k>=20:
        if flag3==0:
            tianchong(returnlist3)
            uart.write(bytearray([0x05]))
            for value in returnlist3:
                uart.write(bytearray([value]))
            flag3=1
            uart.write(bytearray([0x07]))
            print(returnlist3)






