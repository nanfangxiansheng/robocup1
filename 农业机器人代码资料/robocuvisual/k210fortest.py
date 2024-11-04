import sensor
import image
import lcd
import time
from machine import UART
from board import board_info
from fpioa_manager import fm
fm.register(6, fm.fpioa.UART1_TX, force=True)
fm.register(7, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 9600,8,None,1,timeout=1000, read_buf_len=4096)
lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.run(1)
sensor.set_hmirror(True)
sensor.set_vflip(True)

def find_max(blobs):
    max_size=0
    for blob in blobs:

        if blob.pixels() > max_size:
            max_blob = blob
            max_size = blob.pixels()
    return max_blob
def find_max_special(blobs):#used to detct the long flower pot in line2
    max_blob=None
    max_size=0
    for blob in blobs:
        if blob.h()//blob.w()<=1.5:
            if blob.pixels()>max_size:
               max_blob=blob
               max_size=blob.pixels()
    return max_blob
#the fllowing color thresholds need to care and modify
white_threshold0  =(87, 100, -17, 10, -29, 5)#寻找白色色块,used for line3
white_crosscolor=(67, 100, -10, 9, -38, -8)#白色十字used for white cross detection
blue_threshold=(46, 69, 8, 36, -90, -59)#蓝色色块
red_threshold=(39, 85, 8, 46, -17, 13)#红色色块
green_threshold=(56, 86, -49, -28, -6, 22)#寻找绿色色块
thresholds=[blue_threshold,red_threshold,green_threshold]#第四行的识别综合阈�?
fwl1=0#第三行左侧第一个find_white
fwr1=0#第三行右�??
lastl=0#第四行左�??
lastr=0#第四行右�??
mode=5#change_mode
x=0#flowerpot place
y=0
flag=0
flag1=0#这是第四行左侧辅助发送信息的
flag2=0#这是第四行右侧辅助发送信息的
color=0
white_cross=0
cx=0#标志x之和
cy=0#标志y之和
tx=0#第三行检测花盆x之和
ty=0#第三行检测花盆y之和
gcount=0#检测花盆颜色的绿色的计数标�?
bcount=0#检测花盆颜色的蓝色的计数标�?
rcount=0#检测花盆颜色的红色的计数标�?
sumall=60#这是被除的总数，后面会进行修正调整
fx=0#这是第四行横坐标的加�?
fy=0#这是第四行纵坐标的加�?
s=0#area =w*h
pixt=0#three->pixels
sumallt=20#line3->left.sumall
sumalltr=20#line3->right.sumall
sumallf=0#line4->sumall
gewei=0#suitable for mode 5
shibai=0#suitable for mode 5
xgewei=0#suitbale for other mode
ygewei=0
xshibai=0
yshibai=0

while True:




    if(mode==1):#第三行左�??,line3->left
        img=sensor.snapshot()
        blobs = img.find_blobs([white_threshold0],x_stride=30,y_stride=30,area_thershold=400,pixel_threshold=1000,merge=True,margin=8)
        if blobs:
                b=find_max_special(blobs)
                if(b==None):
                   x=0
                   y=0
                   sumallt-=1

                else :
                   x=b.cx()
                   y=b.cy()
                   pixt=b.pixels()

                   tmp=img.draw_rectangle(b[0:4])
                   tmp=img.draw_cross(b[5], b[6])
                   c=img.get_pixel(b[5], b[6])
        fwl1+=1
        tx+=x
        ty+=y
        if x==0&y==0:
           sumallt-=1


        if fwl1==20:
           if(sumallt==0):
               sumallt=20
           x=tx//sumallt
           y=ty//sumallt
           sumallt=20
           tx=0
           ty=0
           xgewei=x%10
           ygewei=y%10
           xshibai=(x-xgewei)//10
           yshibai=(y-ygewei)//10
           uart.write(bytearray([0x01]))
           uart.write(bytearray([0x01]))
           uart.write(bytearray([xgewei]))
           uart.write(bytearray([xshibai]))
           uart.write(bytearray([ygewei]))
           uart.write(bytearray([yshibai]))
           uart.write(bytearray([0x01]))
           uart.write(bytearray([0x02]))

        lcd.display(img)
    if(mode==2):#第三行右�??,line3->right
        img=sensor.snapshot()
        blobs = img.find_blobs([white_threshold0],x_stride=30,y_stride=30,area_thershold=400,pixel_threshold=1000)
        if blobs:
                b=find_max(blobs)
                x=b.cx()
                y=b.cy()

                tmp=img.draw_rectangle(b[0:4])
                tmp=img.draw_cross(b[5], b[6])
                c=img.get_pixel(b[5], b[6])
        fwr1+=1
        if(fwr1>=1):
            tx+=x
            ty+=y
            if(x==0&y==0):
                sumalltr-=1

        if fwr1==20:
           if(sumalltr==0):
               sumalltr=20

           x=tx//sumalltr
           y=ty//sumalltr
           tx=0
           ty=0
           sumalltr=20
           xgewei=x%10
           ygewei=y%10
           xshibai=(x-xgewei)//10
           yshibai=(y-ygewei)//10
           uart.write(bytearray([0x01]))
           uart.write(bytearray([0x02]))
           uart.write(bytearray([xgewei]))
           uart.write(bytearray([xshibai]))
           uart.write(bytearray([ygewei]))
           uart.write(bytearray([yshibai]))
           uart.write(bytearray([0x01]))
           uart.write(bytearray([0x02]))

        lcd.display(img)
    if(mode==3):#第四行左�??,line4->left
        flag=flag+1
        if flag>3:
           flag=1
        threshold0=thresholds[flag-1]

        img=sensor.snapshot()
        blobs=img.find_blobs([threshold0],x_stride=70,y_stride=70,area_threshold=1000,pixel_threshold=2000)
        if blobs:
                b=find_max(blobs)
                x=b.cx()
                y=b.cy()
                if(lastl>=1):
                        fx+=x
                        fy+=y
                        sumallf+=1
                        if(x==0&y==0):
                            sumallf-=1

                        tmp=img.draw_rectangle(b[0:4])
                        tmp=img.draw_cross(b[5], b[6])
                        c=img.get_pixel(b[5], b[6])
                        if(flag==1):
                            bcount+=1
                        elif flag==2:
                            rcount+=1
                        elif flag==3:
                            gcount+=1
        lastl+=1
    if(mode==4):#第四行右�??,line4->right
            flag=flag+1
            if(flag>3):
               flag=1
            threshold0=thresholds[flag-1]

            img=sensor.snapshot()
            blobs=img.find_blobs([threshold0],x_stride=70,y_stride=70,area_threshold=1000,pixel_threshold=2000)
            if(blobs):
                b=find_max(blobs)
                x=b.cx()
                y=b.cy()
                if(lastr>=1):
                    fx+=x
                    fy+=y
                    sumallf+=1
                    if(x==0&y==0):
                        sumallf-=1
                    tmp=img.draw_rectangle(b[0:4])
                    tmp=img.draw_cross(b[5], b[6])
                    c=img.get_pixel(b[5], b[6])
                    if(flag==1):
                        bcount+=1
                    elif flag==2:
                        rcount+=1
                    elif flag==3:
                        gcount+=1
            lastr+=1

    if (mode==5):#识别白色十字,modify_lines

            img=sensor.snapshot()

            blobs = img.find_blobs([white_crosscolor],x_stride=30,y_stride=30,area_thershold=800,pixels_threshold=400)
            if blobs:
                    b=find_max(blobs)
                    x=b.cx()
                    y=b.cy()
                    s=b.area()
                    tmp=img.draw_rectangle(b[0:4])
                    tmp=img.draw_cross(b[5], b[6])
                    c=img.get_pixel(b[5], b[6])
            white_cross+=1
            if(white_cross>=1):
               if(x==0&y==0):
                    sumall-=1
               if(s>=18000):
                    x=0
                    y=0
                    sumall-=1

               if(s<=2000):
                    x=0
                    y=0
                    sumall-=1
               cx+=x
               cy+=y

            if(white_cross==60):
               if(sumall==0):
                   sumall=60
               x=cx//sumall
               y=cy//sumall
               cx=0
               cy=0
               sumall=60
               gewei=x%10
               shibai=(x-gewei)//10
               uart.write(bytearray([0x01]))
               uart.write(bytearray([0x05]))
               uart.write(bytearray([gewei]))
               uart.write(bytearray([shibai]))
               uart.write(bytearray([0x01]))
               uart.write(bytearray([0x01]))
               uart.write(bytearray([0x01]))
               uart.write(bytearray([0x02]))
               print(x)

            lcd.display(img)



    if(lastr>=40):
            if(flag2==0):
                if(sumallf==0):
                    sumallf=20
                x=fx//sumallf
                y=fy//sumallf
                if(gcount>=bcount and gcount>=rcount):
                    color=1
                if(bcount>=gcount and bcount>=rcount):
                    color=2
                if(rcount>=bcount and rcount>=gcount):
                    color=3
                fx=0
                fy=0
                gcount=0
                bcount=0
                rcount=0
                sumallf=0
                xgewei=x%10
                ygewei=y%10
                xshibai=(x-xgewei)//10
                yshibai=(y-yshibai)//10
                uart.write(bytearray([0x01]))
                uart.write(bytearray([0x04]))
                uart.write(bytearray([xgewei]))
                uart.write(bytearray([xshibai]))
                uart.write(bytearray([ygewei]))
                uart.write(bytearray([yshibai]))
                uart.write(bytearray([color]))
                uart.write(bytearray([0x02]))
                color=0

                lcd.display(img)
                flag2=1

    if(lastl>=40):
            if(flag1==0):
                if(sumallf==0):
                   sumallf=20
                x=fx//sumallf
                y=fy//sumallf
                if(gcount>=bcount and gcount>=rcount):
                    color=1
                if(bcount>=gcount and bcount>=rcount):
                    color=2
                if(rcount>=bcount and rcount>=gcount):
                    color=3
                fx=0
                fy=0

                gcount=0
                bcount=0
                rcount=0
                sumallf=0
                xgewei=x%10
                ygewei=y%10
                xshibai=(x-xgewei)//10
                yshibai=(y-yshibai)//10
                uart.write(bytearray([0x01]))
                uart.write(bytearray([0x03]))
                uart.write(bytearray([xgewei]))
                uart.write(bytearray([xshibai]))
                uart.write(bytearray([ygewei]))
                uart.write(bytearray([yshibai]))
                uart.write(bytearray([color]))
                uart.write(bytearray([0x02]))
                color=0

                lcd.display(img)
                flag1=1










