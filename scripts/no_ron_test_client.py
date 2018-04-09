#! /usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import sys, codecs

ip = "172.18.12.82" # show_ipでサーバー側のipを調べてここに設定する
port = 50007

def main():
    # ソケット通信でTCPを使うセッティングをする
    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    soc.connect((ip, port))

    while(1):
        data = soc.recv(4096).decode('utf-8')       # データを受信（4096バイトまで）
        if data != "":
            print ("Client > ",data)        # サーバー側の書き込みを表示
        else:
            print ("音声認識のサーバー側が終了したため")
            print ("クライアント側も終了します。")
            soc.close()
            break


if __name__ == '__main__':
    main()
