#! /usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import types

#ip = "localhost"
ip = "172.16.40.225" # show_ipでホスト側のipを調べてここに設定する
port = 50007

def send_message(_message):
    soc.send(_message.encode('utf-8')) # ソケットに入力したデータを送信
    

# main
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((ip, port))    # 指定したホスト(IP)とポートをソケットに設定
s.listen(1)                     # 1つの接続要求を待つ
try:
    soc, addr = s.accept()          # 要求が来るまでブロック
except:
    soc.close()
send_message("hello")
print("Conneted by"+str(addr))  #サーバ側の合図


