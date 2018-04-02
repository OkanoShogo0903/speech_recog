#! /usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import types
import threading
#import transcribe_streaming_mic
def send_message(message):
    soc.send(message.encode('utf-8')) # ソケットに入力したデータを送信
    

def main():
    #try:
    #    while 1:
    #        pass
    #except KeyboardInterrupt:
    #    ''' <Ctrl-c>押された時以外の例外が起きても処理は続行する '''
    #    print(soc.close())

    t=threading.Timer(1,main)
    t.start()

    #while (1):
    #    data = input("Client>") # 入力待機 
    #    soc.send(data.encode('utf-8')) # ソケットに入力したデータを送信

    #    if data == "q":             # qが押されたら終了
    #        soc.close()
    #        break
    #soc.close()


ip = "172.18.12.23" # show_ipでホスト側のipを調べてここに設定する
#ip = "localhost"
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((ip, 50007))    # 指定したホスト(IP)とポートをソケットに設定
s.listen(1)                     # 1つの接続要求を待つ
soc, addr = s.accept()          # 要求が来るまでブロック
print("Conneted by"+str(addr))  #サーバ側の合図

t=threading.Timer(1,main)
t.start()
