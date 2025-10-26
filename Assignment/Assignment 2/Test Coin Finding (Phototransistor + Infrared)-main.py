from cyberbot import *

coin_found = False  

def forward():
    bot(18).servo_speed(75)
    bot(19).servo_speed(-75)

def backwards():
    bot(18).servo_speed(-75)
    bot(19).servo_speed(75)
    sleep(250)

def right():
    bot(18).servo_speed(75)
    bot(19).servo_speed(75)
    sleep(250)

def left():
    bot(18).servo_speed(-75)
    bot(19).servo_speed(-75)
    sleep(250)

def stop():
    bot(18).servo_speed(0)
    bot(19).servo_speed(0)

while True:
    if coin_found:
        stop()  
        bot(16).tone(2000, 500)  
        sleep(500)  
        continue  

    bot(8).write_digital(1)        
    qt_left = bot(8).rc_time(1)
    bot(6).write_digital(1)        
    qt_right = bot(6).rc_time(1)

    if qt_left < 100 or qt_right < 100:  
        coin_found = True  
        continue  

    irL = bot(14, 13).ir_detect(37500)
    irR = bot(1, 2).ir_detect(37500)

    if irL == 0 and irR == 0:     
        backwards()                                 
        right()
    elif irL == 1 and irR == 0:   
        backwards()                                  
        left()
    elif irL == 0 and irR == 1:  
        backwards()                                   
        right()
    else:                                           
        forward()