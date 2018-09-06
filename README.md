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
あと、GOOGLE_APPLICATION_CREDENTIALSの設定.
鍵はメンバーからUSB等でもらうこと.  

そして、以下の
~~~
export GOOGLE_APPLICATION_CREDENTIALS=鍵のPATH
~~~

## RosTopic
- **音声認識の結果を返すtopic**
```
voice_recog
```
- **音声認識のon,off**
```
google_req/start
google_req/stop
```
## Using
以下のコードを動かすとRosTopicが出ます.
~~~
$ roscore
$ python speech_recog/script/speech_recog_normal.py
~~~

## Warning
pyaudio0.2.7 -> 正常に動作しないバグを含んでいる。githubのUberi/speech_recognitionのREADMEで述べられている。
pyaudio0.2.11 -> 正常に動作する
以下のことを試す。

- `pip install pyaudio==0.2.11`
    - 無理なら`rm /usr/lib/python2.7/dist-packeages/pyaudio`でpyaudio消して再度pip installしてみて
    - `~/.cache/pip`中にあるのを全部消して(キャッシュクリア)しないと指定の0.2.11とかのヴァージョンのライブラリをインストールできない可能性あり
- Anacondaの仮想環境下で動作させるのもあるけど、その場合google関連のライブラリがpath通ってないのかエラー吐くのでそのへんはよくわからん

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

## Unknown
- This Warning happen in LabThinkPad

~~~
/usr/local/lib/python2.7/dist-packages/urllib3/util/ssl_.py:137: InsecurePlatformWarning: A true SSLContext object is not available. This prevents urllib3 from configuring SSL appropriately and may cause certain SSL connections to fail. You can upgrade to a newer version of Python to solve this. For more information, see https://urllib3.readthedocs.io/en/latest/advanced-usage.html#ssl-warnings
~~~

