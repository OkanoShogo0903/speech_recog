# WellCome to Speech Recog
GoogleSpeechAPIを使った音声認識APIを利用するプログラムをおいておいてあります
python2.7とpython3.4の両方に対応するように書いてます

## Install
GoogleのSDKを利用するので、そのインストールをする
~~~
$ sudo pip install --upgrade google-cloud-speech
$ gcloud auth application-default login

$ curl https://sdk.cloud.google.com | bash
$ exec -l $SHELL
$ gcloud version
$ gcloud auth login
$ gcloud auth list

~~~
あと、GOOGLE_APPLICATION_CREDENTIALSの設定
[このページ](https://qiita.com/j-un/items/dc46b3b766a7afb4080c)を参照
~~~
/tmp/mozilla_nvidia0/**鍵の名前**
~~~

## Using
~~~
python ./script/transcribe_streaming_mic.py 
~~~

## some error
TODO qiitaへの自作メモにリンク貼っとけ
DefaultCredentialsError
