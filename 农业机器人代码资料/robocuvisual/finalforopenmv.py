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
    (15, 100, 11, 42, 0, 31)
]
threshold2=[#green
    (25, 96, -36, -1, 3, 38)
]
threshold3=[#blue
    (0, 57, -12, 13, -33, -7)
]
thresholds=[threshold1,threshold2,threshold3]

# 初始化设�?
sensor.reset()  # 重置摄像头模块，以确保摄像头处于已知的初始状�?
sensor.set_pixformat(sensor.RGB565)  # 像素格式设置
sensor.set_framesize(sensor.QVGA)  # 分辨率大小设置，QVGA�?320x240
sensor.skip_frames(time=2000)  # 让摄像头跳过前两秒的帧，为了在实际图像捕获之前，摄像头有足够的时间自动调�?
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
sensor.set_vflip(True)
sensor.set_hmirror(True)

GAIN_SCALE = 1.7
current_gain_in_decibels = sensor.get_gain_db()
sensor.set_auto_gain(False, \
    gain_db = current_gain_in_decibels * GAIN_SCALE)
clock = time.clock()
#the following is not used
first1_r=0
first1_b=0
first1_g=0
second1_r=0
second1_b=0
second1_g=0
third1_r=0
third1_g=0
third1_b=0
fourth1_r=0
fourth1_g=0
fourth1_b=0
fifth1_r=0
fifth1_g=0
fifth1_b=0
sixth1_r=0
sixth1_g=0
sixth1_b=0
first2_r=0
first2_b=0
first2_g=0
second2_r=0
second2_b=0
second2_g=0
third2_r=0
third2_g=0
third2_b=0
fourth2_r=0
fourth2_g=0
fourth2_b=0
fifth2_r=0
fifth2_g=0
fifth2_b=0
sixth2_r=0
sixth2_g=0
sixth2_b=0
first3_r=0
first3_b=0
first3_g=0
second3_r=0
second3_b=0
second3_g=0
third3_r=0
third3_g=0
third3_b=0
fourth3_r=0
fourth3_g=0
fourth3_b=0
fifth3_r=0
fifth3_g=0
fifth3_b=0
sixth3_r=0
sixth3_g=0
sixth3_b=0
flag = 0
returnlist1=[0,0,0,0,0,0]#middle
returnlist2=[0,0,0,0,0,0]#left
returnlist3=[0,0,0,0,0,0]#right
printflag=1
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
sendcount=0
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
def tianchong1(list):#通过数值比较最终给识别数组中进行填充的函数
    if(compare_max(first1_b,first1_g,first1_r)==first1_r):
        list[0]=3
    if(compare_max(first1_b,first1_g,first1_r)==first1_b):
        list[0]=2
    if(compare_max(first1_b,first1_g,first1_r)==first1_g):
        list[0]=1
    if(compare_max(second1_b,second1_g,second1_r)==second1_r):
        list[1]=3
    if(compare_max(second1_b,second1_g,second1_r)==second1_b):
        list[1]=2
    if(compare_max(second1_b,second1_g,second1_r)==second1_g):
        list[1]=1
    if(compare_max(third1_g,third1_b,third1_r)==third1_b):
        list[2]=2
    if(compare_max(third1_g,third1_b,third1_r)==third1_g):
        list[2]=1
    if(compare_max(third1_g,third1_b,third1_r)==third1_r):
        list[2]=3
    if(compare_max(fourth1_b,fourth1_g,fourth1_r)==fourth1_r):
        list[3]=3
    if(compare_max(fourth1_b,fourth1_g,fourth1_r)==fourth1_g):
        list[3]=1
    if(compare_max(fourth1_b,fourth1_g,fourth1_r)==fourth1_b):
        list[3]=2
    if(compare_max(fifth1_b,fifth1_g,fifth1_r)==fifth1_r):
        list[4]=3
    if(compare_max(fifth1_b,fifth1_g,fifth1_r)==fifth1_g):
        list[4]=1
    if(compare_max(fifth1_b,fifth1_g,fifth1_r)==fifth1_b):
        list[4]=2
    if(compare_max(sixth1_b,sixth1_g,sixth1_r)==sixth1_r):
        list[5]=3
    if(compare_max(sixth1_b,sixth1_g,sixth1_r)==sixth1_g):
        list[5]=1
    if(compare_max(sixth1_b,sixth1_g,sixth1_r)==sixth1_b):
        list[5]=2
def tianchong2(list):#通过数值比较最终给识别数组中进行填充的函数
            if(compare_max(first2_b,first2_g,first2_r)==first2_r):
                list[0]=3
            if(compare_max(first2_b,first2_g,first2_r)==first2_b):
                list[0]=2
            if(compare_max(first2_b,first2_g,first2_r)==first2_g):
                list[0]=1
            if(compare_max(second2_b,second2_g,second2_r)==second2_r):
                list[1]=3
            if(compare_max(second2_b,second2_g,second2_r)==second2_b):
                list[1]=2
            if(compare_max(second2_b,second2_g,second2_r)==second2_g):
                list[1]=1
            if(compare_max(third2_g,third2_b,third2_r)==third2_b):
                list[2]=2
            if(compare_max(third2_g,third2_b,third2_r)==third2_g):
                list[2]=1
            if(compare_max(third2_g,third2_b,third2_r)==third2_r):
                list[2]=3
            if(compare_max(fourth2_b,fourth2_g,fourth2_r)==fourth2_r):
                list[3]=3
            if(compare_max(fourth2_b,fourth2_g,fourth2_r)==fourth2_g):
                list[3]=1
            if(compare_max(fourth2_b,fourth2_g,fourth2_r)==fourth2_b):
                list[3]=2
            if(compare_max(fifth2_b,fifth2_g,fifth2_r)==fifth2_r):
                list[4]=3
            if(compare_max(fifth2_b,fifth2_g,fifth2_r)==fifth2_g):
                list[4]=1
            if(compare_max(fifth2_b,fifth2_g,fifth2_r)==fifth2_b):
                list[4]=2
            if(compare_max(sixth2_b,sixth2_g,sixth2_r)==sixth2_r):
                list[5]=3
            if(compare_max(sixth2_b,sixth2_g,sixth2_r)==sixth2_g):
                list[5]=1
            if(compare_max(sixth2_b,sixth2_g,sixth2_r)==sixth2_b):
                list[5]=2
def tianchong3(list):#通过数值比较最终给识别数组中进行填充的函数
                    if(compare_max(first3_b,first3_g,first3_r)==first3_r):
                        list[0]=3
                    if(compare_max(first3_b,first3_g,first3_r)==first3_b):
                        list[0]=2
                    if(compare_max(first3_b,first3_g,first3_r)==first3_g):
                        list[0]=1
                    if(compare_max(second3_b,second3_g,second3_r)==second3_r):
                        list[1]=3
                    if(compare_max(second3_b,second3_g,second3_r)==second3_b):
                        list[1]=2
                    if(compare_max(second3_b,second3_g,second3_r)==second3_g):
                        list[1]=1
                    if(compare_max(third3_g,third3_b,third3_r)==third3_b):
                        list[2]=2
                    if(compare_max(third3_g,third3_b,third3_r)==third3_g):
                        list[2]=1
                    if(compare_max(third3_g,third3_b,third3_r)==third3_r):
                        list[2]=3
                    if(compare_max(fourth3_b,fourth3_g,fourth3_r)==fourth3_r):
                        list[3]=3
                    if(compare_max(fourth3_b,fourth3_g,fourth3_r)==fourth3_g):
                        list[3]=1
                    if(compare_max(fourth3_b,fourth3_g,fourth3_r)==fourth3_b):
                        list[3]=2
                    if(compare_max(fifth3_b,fifth3_g,fifth3_r)==fifth3_r):
                        list[4]=3
                    if(compare_max(fifth3_b,fifth3_g,fifth3_r)==fifth3_g):
                        list[4]=1
                    if(compare_max(fifth3_b,fifth3_g,fifth3_r)==fifth3_b):
                        list[4]=2
                    if(compare_max(sixth3_b,sixth3_g,sixth3_r)==sixth3_r):
                        list[5]=3
                    if(compare_max(sixth3_b,sixth3_g,sixth3_r)==sixth3_g):
                        list[5]=1
                    if(compare_max(sixth3_b,sixth3_g,sixth3_r)==sixth3_b):
                        list[5]=2
# 进入循环
#在题目中，要求绿色代表轻微干旱，蓝色代表一般干旱，红色代表严重干旱
while True:

            clock.tick()  # 用于更新时钟对象的时钟戳，方便后续计算帧率和测量时间间隔
            img = sensor.snapshot()

            # 更新 flag
            flag += 1
            if flag > 3:
                flag = 1

            # 根据当前 flag 使用相应的阈�?
            threshold = thresholds[flag-1]
            #first->middle

            blobs = img.find_blobs(threshold,roi=[137,16,29,11], x_stride=20, y_stride=4, area_threshold=100,pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist1[0]=3
                        first1_r+=1
                        i+=1

                    elif flag == 2:#绿色
                        returnlist1[0]=1
                        first1_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist1[0]=2
                        first1_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[134,48,35,13], x_stride=30, y_stride=6, area_threshold=100,pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist1[1]=3
                        second1_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist1[1]=1
                        second1_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist1[1]=2
                        second1_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[135,80,34,18], x_stride=25, y_stride=10,area_threshold=100,pixels_threshold=100, merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist1[2]=3
                        third1_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist1[2]=1
                        third1_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist1[2]=2
                        third1_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[135,120,38,17], x_stride=25, y_stride=10, area_threshold=100,pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist1[3]=3
                        fourth1_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist1[3]=1
                        fourth1_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist1[3]=2
                        fourth1_b+=1
                        i+=1

            #servo ID4:591
            blobs = img.find_blobs(threshold,roi=[137,166,36,18], x_stride=25, y_stride=11,area_threshold=100, pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist1[4]=3
                        fifth1_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist1[4]=1
                        fifth1_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist1[4]=2
                        fifth1_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[132,208,36,19], x_stride=26, y_stride=12,area_threshold=100, pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist1[5]=3
                        sixth1_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist1[5]=1
                        sixth1_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist1[5]=2
                        sixth1_b+=1
                        i+=1



            # 根据当前 flag 使用相应的阈�?

            blobs = img.find_blobs(threshold,roi=[47,24,23,13], x_stride=18, y_stride=7, area_threshold=100,pixels_threshold=40,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist2[0]=3
                        first2_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist2[0]=1
                        first2_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist2[0]=2
                        first2_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[43,56,31,13], x_stride=26, y_stride=8, area_threshold=100,pixels_threshold=70,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist2[1]=3
                        second2_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist2[1]=1
                        second2_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist2[1]=2
                        second2_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[32,87,29,14], x_stride=25, y_stride=8,area_threshold=100, pixels_threshold=60,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist2[2]=3
                        third2_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist2[2]=1
                        third2_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist2[2]=2
                        third2_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[24,124,32,14], x_stride=25, y_stride=8, area_threshold=100,pixels_threshold=70,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist2[3]=3
                        fourth2_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist2[3]=1
                        fourth2_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist2[3]=2
                        fourth2_b+=1
                        i+=1

            #servo ID4:591
            blobs = img.find_blobs(threshold,roi=[23,161,33,16], x_stride=25, y_stride=9,area_threshold=100, pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist2[4]=3
                        fifth2_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist2[4]=1
                        fifth2_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist2[4]=2
                        fifth2_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[23,205,34,12], x_stride=29, y_stride=8,area_threshold=100, pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist2[5]=3
                        sixth2_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist2[5]=1
                        sixth2_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist2[5]=2
                        sixth2_b+=1
                        i+=1



            # 根据当前 flag 使用相应的阈�?

            blobs = img.find_blobs(threshold,roi=[230,18,30,10], x_stride=25, y_stride=7, area_threshold=100,pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist3[0]=3
                        first3_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist3[0]=1
                        first3_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist3[0]=2
                        first3_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[236,44,30,13], x_stride=25, y_stride=8, area_threshold=100,pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist3[1]=3
                        second3_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist3[1]=1
                        second3_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist3[1]=2
                        second3_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[241,77,31,13], x_stride=25, y_stride=7,area_threshold=100, pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist3[2]=3
                        third3_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist3[2]=1
                        third3_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist3[2]=2
                        third3_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[243,116,34,18], x_stride=26, y_stride=12, area_threshold=100,pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist3[3]=3
                        fourth3_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist3[3]=1
                        fourth3_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist3[3]=2
                        fourth3_b+=1
                        i+=1

            #servo ID4:591
            blobs = img.find_blobs(threshold,roi=[251,160,33,16], x_stride=27, y_stride=9,area_threshold=100, pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist3[4]=3
                        fifth3_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist3[4]=1
                        fifth3_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist3[4]=2
                        fifth3_b+=1
                        i+=1
            blobs = img.find_blobs(threshold,roi=[249,202,33,15], x_stride=26, y_stride=10,area_threshold=100, pixels_threshold=100,merge=True)
            if blobs:
                    blob=find_max(blobs)
                    img.draw_rectangle(blob.rect())  # 绘制矩形
                    if flag == 1:#红色
                        returnlist3[5]=3
                        sixth3_r+=1
                        i+=1
                    elif flag == 2:#绿色
                        returnlist3[5]=1
                        sixth3_g+=1
                        i+=1
                    elif flag == 3:#蓝色
                        returnlist3[5]=2
                        sixth3_b+=1
                        i+=1
            if i>=70:

                if flag1==0:
                    for sendcount in range(100000):


                        tianchong1(returnlist1)
                        tianchong2(returnlist2)
                        tianchong3(returnlist3)
                        uart.write(bytearray([0x05]))
                        for value in returnlist1:
                            uart.write(bytearray([value]))
                        for value in returnlist2:
                            uart.write(bytearray([value]))
                        for value in returnlist3:
                            uart.write(bytearray([value]))
                        uart.write(bytearray([0x07]))
                        if printflag==1:

                            print(returnlist1)
                            print(returnlist2)
                            print(returnlist3)
                            printflag=0
                        sensor.sleep(200)

                    flag1=1







