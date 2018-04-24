# WellCome to Speech Recog
GoogleSpeechAPIを使った音声認識APIを利用するプログラムをおいておいてあります
python2.7とpython3.4の両方に対応するようになっている（はず）

## Install
GoogleのSDKを利用するので、そのインストールをする
~~~
$ curl https://sdk.cloud.google.com | bash
$ exec -l $SHELL
$ gcloud version

$ sudo pip install --upgrade google-cloud-speech
$ gcloud auth application-default login
~~~
これでは動作しない場合があって、その時は[このページ](https://qiita.com/j-un/items/dc46b3b766a7afb4080c)を参照
あと、GOOGLE_APPLICATION_CREDENTIALSの設定
鍵はメンバーからUSB等でもらうこと
~~~
`export GOOGLE_APPLICATION_CREDENTIALS=/鍵のPATH`
~~~

## Using
PCを二台用意することを想定しています  
以下の説明では、一方のマイクをつけて音声認識を行うPCをサーバ側、もう一方のPCをクライアント側とします  
**クライアント側にROSが入っている場合はclient.pyが使用できますが、ROSが入っていない場合はno_ron_test_client.pyが使用できます**  

1. ソースコードのダウンロード
`$ git clone https://github.com/OkanoShogo0903/speech_recog.git`でソースコードを任意の場所に落とす

2. IPの確認  
サーバ側で`bash speech_recog/script/show_ip.sh`を実行し、サーバPCのIPを確認  

3. IPの設定  
サーバ側の`speech_recog/script/server.py`を~~好みのテキストエディタ~~vimで開いて、グローバル変数のipを先ほど確認したipに設定します  
同様に、クライアント側の`speech_recog/script/client.py`を編集してipを設定します  

4. サーバ側の起動  
`python speech_recog/script/speech_recog_socket.py`
(`speech_recog_socket.py`は内部で`server.py`を呼び出している)  

5. クライアント側の起動  
`client.py`をROSRUNする

## Using
~~~
$ roscore
$ rosrun go_get_it_imp VoiceRecognizer.py
$ rosrun go_get_it_imp GoGetItNode.py
$ python speech_recog_normal.py
~~~

## Error
- ネットワークエラー集
  - `OSError: [Errno 99] Cannot assign requested address`  
      ネット繋がってないよ〜 :sob:  
- ネットワーク以外のエラー
  - DefaultCredentialsError
    GOOGLE_APPLICATION_CREDENTIALSを設定する
    `export GOOGLE_APPLICATION_CREDENTIALS=/鍵のPATH`
  - AuthMetadataPluginCallback
~~~
ERROR:root:AuthMetadataPluginCallback "<google.auth.transport.grpc.AuthMetadataPlugin object at 0x7f41cbc646a0>" raised exception!
Traceback (most recent call last):
  File "/home/okano/anaconda3/lib/python3.6/site-packages/urllib3/contrib/pyopenssl.py", line 441, in wrap_socket
    cnx.do_handshake()
  File "/home/okano/anaconda3/lib/python3.6/site-packages/OpenSSL/SSL.py", line 1716, in do_handshake
    self._raise_ssl_error(self._ssl, result)
  File "/home/okano/anaconda3/lib/python3.6/site-packages/OpenSSL/SSL.py", line 1449, in _raise_ssl_error
    raise SysCallError(-1, "Unexpected EOF")
OpenSSL.SSL.SysCallError: (-1, 'Unexpected EOF')

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/home/okano/anaconda3/lib/python3.6/site-packages/urllib3/connectionpool.py", line 595, in urlopen
    self._prepare_proxy(conn)
  File "/home/okano/anaconda3/lib/python3.6/site-packages/urllib3/connectionpool.py", line 816, in _prepare_proxy
    conn.connect()
  File "/home/okano/anaconda3/lib/python3.6/site-packages/urllib3/connection.py", line 326, in connect
    ssl_context=context)
  File "/home/okano/anaconda3/lib/python3.6/site-packages/urllib3/util/ssl_.py", line 329, in ssl_wrap_socket
    return context.wrap_socket(sock, server_hostname=server_hostname)
  File "/home/okano/anaconda3/lib/python3.6/site-packages/urllib3/contrib/pyopenssl.py", line 448, in wrap_socket
    raise ssl.SSLError('bad handshake: %r' % e)
ssl.SSLError: ("bad handshake: SysCallError(-1, 'Unexpected EOF')",)
~~~
## HotRef
~~~
$ lsusb
$ export GOOGLE_APPLICATION_CREDENTIALS='/home/nvidia/My Project 31035-185bec02f0ba.json'
~~~

TODO qiitaへの自作メモにリンク貼っとけ
