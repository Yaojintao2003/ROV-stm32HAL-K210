
# 同时运行openmv和k210
import sensor
import image
import lcd
import KPU as kpu

import utime
import math
#from board import board_info
from Maix import freq
from fpioa_manager import fm
from machine import UART

#point_threshold = (0, 34, -128, 127, 6, 127)
#tube_threshold = (40, 100, -20, 20, 15, 40)  #白天
#tube_threshold = (50, 100, -15, 10, 6, 30)   #晚上
#tube_threshold = (50, 100, -15, 10, -10, 30)
#tube_threshold = (55, 100, -20, 20, -60, 0) #地板初始化
#(50, 100, -40, 0, 0, 40)
#tube_threshold = (55, 100, -40, 0, 10, 45) #水面初始化
#tube_threshold = (60, 100, -5, 20, -20, 4)  #场地白线
#tube_threshold = (50, 100, -30, 30, -30, 30)
#tube_threshold = (45, 100, -30, 30, 0, 40) #202109151411
tube_threshold = (20, 100, -40, 40, -10, 40) #19：37 水中启动 不偏绿 偏暗

fm.register(9,fm.fpioa.UART1_TX)
fm.register(10,fm.fpioa.UART1_RX)
UART_USB = UART(UART.UART1, 115200, 8, None, 1, timeout = 1000, read_buf_len = 128)

lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((224, 224))
sensor.set_hmirror(0)
sensor.set_vflip(0)
sensor.set_auto_gain(False, gain_db = 8.786814) # must be turned off for color tracking
sensor.set_auto_whitebal(False, (0.0, 0.0, 0.0)) # must be turned off for color tracking
#sensor.set_auto_gain(False) # must be turned off for color tracking
#sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.set_contrast(0)
sensor.set_brightness(0)
sensor.set_saturation(0)
sensor.skip_frames(time = 2000)

#lcd.init()
#sensor.reset()
#sensor.set_pixformat(sensor.RGB565)
#sensor.set_framesize(sensor.QVGA)
#sensor.set_windowing((224, 224))
#sensor.run(1)
#sensor.set_vflip(0)
#sensor.set_hmirror(1)
#sensor.skip_frames(time = 2000)
clock = utime.clock()

task = kpu.load(0x300000)
#anchor = (0.5,0.86,0.99,1.0,1.0,1.09,1.13,1.14,1.15,1.86)
anchor = (0.58,0.78,0.92,0.98,1.04,1.05,1.05,1.11,1.17,1.8)

a = kpu.init_yolo2(task, 0.6, 0.3, 5, anchor)#这个就相当与一个api了
labels_txt = ("roll","REC")
Line_Angle = 0
Line_x = 0
Line_y = 0
Target_Flag = 1

lcd.rotation(1)

def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob[2] * blob[3] > max_size:
            max_blob = blob
            max_size = blob[2]*blob[3]
    return max_blob

while(True):
    img = sensor.snapshot()
    #img = sensor.snapshot().rotation_corr(z_rotation= 90)
    #img = sensor.snapshot()
    code = kpu.run_yolo2(task, img)
    clock.tick()
    Target_Flag = 1
    if code:
        for i in code:
            a = img.draw_rectangle(i.rect(), (0, 255, 0), 2)#a就是里面的图像
            #a = img.draw_rectangle((0, 0, 60, 120), (0, 255, 0), 2)#a就是里面的图像
            #a = img.draw_rectangle((i.rect()[0], i.rect()[1], i.rect()[2], i.rect()[3]), (0, 255, 0), 2)#a就是里面的图像
            a = lcd.display(img)
            for i in code:#i就是code
                lcd.draw_string(i.x()+45, i.y()-5, labels_txt[i.classid()]+" "+'%.2f' % i.value(), lcd.WHITE, lcd.GREEN)
                if i.classid() == 0:
                    Target_Flag = 0
                elif i.classid() == 1:
                    Target_Flag = 2
    img = img.rotation_corr(z_rotation= 90)
    blobs = img.find_blobs([tube_threshold], pixels_threshold=1000)
    if blobs:
        Blob = find_max(blobs)
        img.draw_rectangle(Blob[0:4])
        img.draw_arrow((Blob[5], Blob[6],Blob[5]+int(50*math.cos(Blob[7])),Blob[6]+int(50*math.sin(Blob[7]))),color = (255, 0, 0))
        Line_Angle = 90-Blob.rotation()*57.3
        Line_x = 112-Blob[5]
        Line_y = Blob[6]
        print('1 %d %d %d %d'%(Line_Angle, -Line_x, Line_y, Target_Flag))
        UART_USB.write('1 %d %d %d %d\r\n'%(Line_Angle, -Line_x, Line_y, Target_Flag))
    else:
        print('0 %d %d %d %d'%(Line_Angle, -Line_x, Line_y, Target_Flag))
        UART_USB.write('0 %d %d %d %d\r\n'%(Line_Angle, -Line_x, Line_y, Target_Flag))

    #print(sensor.get_rgb_gain_db(), sensor.get_gain_db())

a = kpu.deinit(task)
