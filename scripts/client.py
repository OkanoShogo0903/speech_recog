#! /usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import rospy
from std_msgs.msg import String
import sys, codecs

pub = rospy.Publisher('speechrecognition/voice_recog',String,queue_size=10)

ip = "172.18.12.53" # show_ipでサーバー側のipを調べてここに設定する
port = 50007

def main():

    # ソケット通信でTCPを使うセッティングをする
    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    soc.connect((ip, port))

    while(1):
        data = soc.recv(4096).decode('utf-8')       # データを受信（4096バイトまで）
        if data != "":
            print "Client > ",data        # サーバー側の書き込みを表示
            pub.publish(data)
        else:
            print "音声認識のサーバー側が終了したためクライアント側も終了します。"
            soc.close()
            break

if __name__ == '__main__':
    rospy.init_node('speech_recog_client')
    main()
