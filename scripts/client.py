#! /usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import rospy
from std_msgs.msg import String
import sys, codecs

pub = rospy.Publisher('speechrecognition/voice_recog',String,queue_size=10)

def main():
    ip = "172.18.12.23" # show_ipでサーバー側のipを調べてここに設定する

    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    soc.connect((ip, 50007))

    while(1):
        data = soc.recv(4096).decode('utf-8')       # データを受信（1024バイトまで）
        if data != "":
            print "Client > ",data        # サーバー側の書き込みを表示
            pub.publish(data)
        else:
            print "音声認識のサーバー側が終了しました"
            soc.close()
            break

if __name__ == '__main__':
    rospy.init_node('speech_recog_client')
    main()
