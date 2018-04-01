# -*- coding: utf-8 -*-
import socket

def main():
    ip = "192.168.11.6" # show_ipでホスト側のipを調べてここに設定する
    #ip = "localhost"
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((ip, 50007))    # 指定したホスト(IP)とポートをソケットに設定
    s.listen(1)                     # 1つの接続要求を待つ
    soc, addr = s.accept()          # 要求が来るまでブロック
    print("Conneted by"+str(addr))  #サーバ側の合図

    while (1):
        data = raw_input("Client>") # 入力待機
        soc.send(data)              # ソケットに入力したデータを送信

        if data == "q":             # qが押されたら終了
            soc.close()
            break


if __name__ == '__main__':
    main()
