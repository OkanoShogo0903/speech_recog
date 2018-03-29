# WellCome to Speech Recog
GoogleSpeechAPIを使った音声認識APIを利用するプログラムをおいておいてあります
## Install
GoogleのSDKを利用するので、そのインストールをする
~~~
curl https://sdk.cloud.google.com | bash
exec -l $SHELL
gcloud version
gcloud auth login
~~~
## Using
~~~
python script/transcribe_streaming_mic.py 
~~~
