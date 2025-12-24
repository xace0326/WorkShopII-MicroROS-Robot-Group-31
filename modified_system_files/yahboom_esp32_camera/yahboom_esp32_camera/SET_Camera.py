import socket
import time

staip = "192.168.2.93" #填写开启dotcker后信息的ip地址
PORT = 8888

vflip_OFF = "@vflip:0@"     #不倒转 
vflip_OPEN = "@vflip:1@"    #倒转
mirror_OFF = "@mirror:0@"   #不开启水平镜像
mirror_OPEN = "@mirror:1@"  #开启水平镜像


 



def set_Camera(vflip_flag,mirror_flag):
    if vflip_flag == True:
        send_data = vflip_OPEN
        sk.sendall(bytes(send_data, encoding="utf8"))
    elif vflip_flag == False:
        send_data = vflip_OFF
        sk.sendall(bytes(send_data, encoding="utf8"))

    time.sleep(1)
    if mirror_flag == True:
        send_data = mirror_OPEN
        sk.sendall(bytes(send_data, encoding="utf8"))
    elif mirror_flag == False:
        send_data = mirror_OFF
        sk.sendall(bytes(send_data, encoding="utf8"))


print("please input docket ipV4:")
staip = input()

sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# 连接服务器 socket tcp
try:
    sk.connect((staip,PORT))   
    set_Camera(True,True) #反转画面
    #set_Camera(False,False) #不反转画面
    print("Camera is set ok!")
    sk.close()
except KeyboardInterrupt:
    sk.close()
except Exception as e:
    print("Camera is set fail!")
    print("Program Error:", Exception)
    sk.close()
    
