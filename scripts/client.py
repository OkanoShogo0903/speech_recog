# -*- coding: utf-8 -*-
import socket

def main():
    ip = "192.168.11.6" # show_ipでホスト側のipを調べてここに設定する

    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    soc.connect((ip, 50007))

    while(1):
        data = soc.recv(4096)       # データを受信（1024バイトまで）
        print ("Client>",data)        # サーバー側の書き込みを表示
        if data == "q":             # qが押されたら終了
            soc.close()
            break

if __name__ == '__main__':
    main()
