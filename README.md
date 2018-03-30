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
あと、GOOGLE_APPLICATION_CREDENTIALSの設定
[このページ](https://qiita.com/j-un/items/dc46b3b766a7afb4080c)を参照
~~~
/tmp/mozilla_nvidia0/**鍵の名前**
~~~

## Using
~~~
$ cd speech_recog
$ python ./script/transcribe_streaming_mic.py 
~~~

## HotRef
~~~
$ lsusb
$ export GOOGLE_APPLICATION_CREDENTIALS='/home/nvidia/My Project 31035-185bec02f0ba.json'
~~~

## some error
TODO qiitaへの自作メモにリンク貼っとけ
DefaultCredentialsError
