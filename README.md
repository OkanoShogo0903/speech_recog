# WellCome to Speech Recog
GoogleSpeechAPIを使った音声認識APIを利用するプログラムをおいておいてあります
python2.7とpython3.4の両方に対応するようになっている（はず）

## Install
GoogleのSDKを利用するので、そのインストールをする
~~~
$ curl https://sdk.cloud.google.com | bash
$ exec -l $SHELL
$ gcloud version
$ gcloud auth login
$ gcloud auth list

$ sudo pip install --upgrade google-cloud-speech
$ gcloud auth application-default login
~~~
これでは動作しない場合があって、その時は[このページ](https://qiita.com/j-un/items/dc46b3b766a7afb4080c)を参照
あと、GOOGLE_APPLICATION_CREDENTIALSの設定
~~~
`export GOOGLE_APPLICATION_CREDENTIALS=/鍵のPATH`
~~~

## Using
PCを二台用意することを想定しています  
一方のマイクをつけて音声認識を行うPCをサーバ側、もう一方のPCをクライアント側とします  
**クライアント側にROSが入っている場合はclient.pyが使用できますが、ROSが入っていない場合はno_ron_test_client.pyが使用できます**  

1. IPの確認  
サーバ側で`bash speech_recog/script/show_ip.sh`を実行し、サーバPCのIPを確認  

2. IPの設定  
サーバ側の`speech_recog/script/server.py`を~~好みのテキストエディタ~~vimで開いて、グローバル変数のipを先ほど確認したipに設定します  
同様に、クライアント側の`speech_recog/script/client.py`を編集してipを設定します  

3. サーバ側の起動  
`python speech_recog/script/transcribe_streaming_mic.py`
(`transcribe_streaming_mic.py`は内部で`server.py`を呼び出している)  
4. クライアント側の起動  
`client.py`をROSRUNする

## HotRef
~~~
$ lsusb
$ export GOOGLE_APPLICATION_CREDENTIALS='/home/nvidia/My Project 31035-185bec02f0ba.json'
~~~

## Error
- ネットワークエラー集
  - `OSError: [Errno 99] Cannot assign requested address`  
      ネット繋がってないよ〜 :sob:  

- ネットワーク以外のエラー
  - DefaultCredentialsError
    GOOGLE_APPLICATION_CREDENTIALSを設定する
    `export GOOGLE_APPLICATION_CREDENTIALS=/鍵のPATH`

TODO qiitaへの自作メモにリンク貼っとけ
