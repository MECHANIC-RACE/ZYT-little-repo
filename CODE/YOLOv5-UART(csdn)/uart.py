import serial  
import time  
  
ser = serial.Serial('/dev/ttyAMA0',115200) # 串口初始化，根据实际情况修改串口号和波特率  

# 定义要输出的数字  
num =  196 

while True:  
            
    #ser.write(str(number).encode()) # 发送数字到串口
    #ser.write('30\n'.encode())
    
    #ser.write('30\r\n'.encode()) # 发送数据
    
    #ser.write(f'{number}\r\n'.encode())
    
    ser.write(str(int(num)).encode()) # 发送数字到串口   
    num += 1  
    if num > 205:  
        num = 196
    
    
    
    time.sleep(0.2) # 等待1秒钟  

