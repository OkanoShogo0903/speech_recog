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

## Using
~~~
python ./script/transcribe_streaming_mic.py 
~~~

## some error
TODO qiitaへの自作メモにリンク貼っとけ
DefaultCredentialsError
