# -*- coding: utf-8 -*-
#!/usr/bin/env python

# Copyright 2017 Google Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Google Cloud Speech APIのsampleをベースにして書かれています
APIにストリームで音声を渡しています

using pip:

    pip install pyaudio

Example usage:
    python transcribe_streaming_mic.py
"""

# [START import_libraries]

from __future__ import division

IS_ROS_ACTIVE = False

import re
import sys
if IS_ROS_ACTIVE:
    import rospy
    from std_msgs.msg import String

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
from six.moves import queue
import threading
from datetime import datetime,timedelta
import time
import types as value_type
# [END import_libraries]

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

speech_loop = 'Null'

#voice_pub = rospy.Publisher('voice_recog',String,queue_size=10)

class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""
        
    def __init__(self, rate, chunk):               
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True
       
    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1, rate=self._rate,
            input=True, frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)
# [END audio_stream]


class WordClass(object):
    ''' ROSに音声認識の結果を投げるWORDスレッドを管理するクラス '''
    thread_num = 0 # スレッドの名前を通し番号で管理するためのクラス共通の値
    def __init__(self):

        # [よく変えるパラーメータ]
        self.loop_interval = 0.5
        self.threshold = timedelta(seconds=1)
        # [よく変えるパラーメータ END]

        self.stop_event = threading.Event() #停止させるかのフラグ
        self.word_stack = []
        self.word_add("",False) # なにも要素を持たないword_add[-1]を参照するとエラーになるのをカバー(不格好)

        WordClass.thread_num += 1
        self.thread = threading.Thread(target = self.word_manager, name="[Word " + str(WordClass.thread_num) + "]")
        self.thread.start()
    

    def __enter__(self):
        # withステートメントで処理するための記述
        return self
        

    def __exit__(self, type, value, traceback):
        #スレッドが停止するのを待つ
        self.stop_event.set()
        self.thread.join()


    def word_add(self,_word, _flag):
        self.word_stack.append({"word":_word, "is_final":_flag, "time_stamp":datetime.now(), "is_published":False})
        #print(word_stack[-1])

    
    def publish(self,_dict):
        ''' jsonにパースできないのはここで消す(仕様) '''
        delete_list = []
        for key in _dict:
            key_type = (type)(_dict[key])
            if key_type != str and key_type != bool:
                delete_list.append(key)
        # 削除
        for i in delete_list:
            _dict.pop(i)

        print(self.thread_name,key)
        print(self.thread_name," publish : ",_dict,self.thread_name)
        if IS_ROS_ACTIVE:
            voice_pub.publish(_dict) # ここで投げる


    def word_manager(self):
        '''
        APIから5秒間データを渡されなかった時、雑音を受け取っていると判断して、処理途中のデータを投げる
        '''
        self.thread_name = threading.currentThread().getName()

        while not self.stop_event.is_set():
            try:
                time.sleep(self.loop_interval)
                interval = datetime.now() - self.word_stack[-1]["time_stamp"]
                print(self.thread_name,interval)
                if self.word_stack[-1]["is_published"] == False:
                    # [FINAL]の処理
                    if self.word_stack[-1]["is_final"] == True: # 最終結果をそのまま渡す
                        print(self.thread_name,"++++++++++++++++++++")
                        self.word_stack[-1]["is_published"] = True
                        self.publish(self.word_stack[-1])
                        create_speech_recog_thread()
                        return
                    # [INTERRUPT]の処理
                    elif interval > self.threshold:
                        if self.word_stack[-1]["word"] != "": # 初期はエラー避けにword=""を入れてるのでそれは外す
                            # 一旦区切って、途中経過の文字列を処理後の文字列として送信する
                            print(self.thread_name,"--------------------")
                            self.word_stack[-1]["is_published"] = True
                            self.word_stack[-1]["is_final"] = True
                            self.publish(self.word_stack[-1])
                            create_speech_recog_thread()
                            return # 途中結果を最終結果として送ったら切れる

            except KeyboardInterrupt:
                return
# [END word manage]


def listen_print_loop(responses):
    """Iterates through server responses and prints them.

    The responses passed is a generator that will block until a response
    is provided by the server.

    Each response may contain multiple results, and each result may contain
    multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
    print only the transcription for the top alternative of the top result.

    In this case, responses are provided for interim results as well. If the
    response is an interim one, print a line feed at the end of it, to allow
    the next result to overwrite it, until the response is a final one. For the
    final one, print a newline to preserve the finalized transcription.
    """

    global speech_loop
    thread_name = threading.currentThread().getName()

    with WordClass() as word:
        while 1:
            try:
                #print(thread_name,"*** listen_print_loop ***")
                for response in responses: # ここでストリームで音声を送信して、GoogleAPIからの結果が来るまで待機する
                    if word.word_stack[-1]["is_final"] == True: # [INTERRUPT]時はMainスレッドをここで殺す
                        print(thread_name,"Die")
                        return

                    #print(thread_name,"**********************")
                    # 渡されたデータが空だった時の処理
                    if not response.results:
                        continue
                    result = response.results[0]
                    if not result.alternatives:
                        continue

                    # Display the transcription of the top alternative.
                    transcript = result.alternatives[0].transcript

                    if not result.is_final:
                        #print(thread_name," processing")
                        #print(thread_name,transcript)
                        word.word_add(transcript, False)
                    else:
                        #print(thread_name," result ")
                        #print(thread_name,transcript)
                        word.word_add(transcript, True) # [FINAL]
                        return

                    if speech_loop == 'stop':
                        print(thread_name,"***** break *****")
                        speech_loop = 'Null'
                        return


                    # 1秒に一回だけforの頭にいくようにして、
            except KeyboardInterrupt:
                return
            except:
                print(thread_name," --------- err -----------",len(threading.enumerate()))
                import traceback
                traceback.print_exc()
                if check_thread_name():
                    create_speech_recog_thread()
                return


def check_thread_name():
    ''' スレッド名の数字だけ取り出して、自分のスレッド名の番号が一番若かったときだけtrueを返す '''
    import re
    #pattern=r'([+-]?[0-9]+\.?[0-9]*)'
    pattern=r'([0-9]+\.?[0-9]*)'

    print("@",threading.currentThread())
    print("@",threading.currentThread().getName())
    number_list = []
    for t in threading.enumerate(): # 自分のスレッド以外かつcleanupスレッド以外の名前に含まれる番号を調べる
        print("@@@",t.getName())
        if t.getName() in 'Main':
            if not t.getName() is threading.currentThread().getName():
                print("@@@@@",t.getName())
                number_list.extend( re.findall(pattern,t.getName()))

    # 自分スレッドの番と、他のスレッドの番号を比較する
    my_num = re.findall(pattern,threading.currentThread().getName())
    print("@@@@@@@@@@@",number_list)
    print("@@@@@@@@@@@",my_num)
    for other_num in number_list:
        if other_num > my_num:
            return False # 自分のスレッドの番号よりも大きな番号があった時
    return True # 自分のスレッドの番号が一番大きかった時

create_num = 0
def create_speech_recog_thread():
    global create_num
    create_num += 1
    name = "[Main " + str(create_num) + "]"
    print(name,"<<<<< main start >>>>>")
    thread = threading.Thread(target = main, name=name)
    #main_thread.setDaemon(True)
    thread.start()
#    if thread.isAlive():
#        thread.join()


#def main(request):
def main():
    # See http://g.co/cloud/speech/docs/languages
    # for a list of supported languages.
    language_code = 'ja-JP'  # a BCP-47 language tag
    client = speech.SpeechClient()
    speech_contexts = [speech.types.SpeechContext(
        phrases=["kitchen", "living", "entrance", "bedroom", "dining", "bikkle", "cupnoodle", "finish", "Follow"]
    )]

    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code,
        speech_contexts=speech_contexts
        )
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)

    with MicrophoneStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator)

        responses = client.streaming_recognize(streaming_config, requests)

        # Now, put the transcription responses to use.
        listen_print_loop(responses)


#is_kill = False
#class SpeechRecog(object):
#    def __init__(self):
#        self.thread = threading.Thread(target = self.please)
#        self.thread.start()
#    
#
#    def __enter__(self):
#        # withステートメントで処理するための記述
#        return self
#        
#
#    def __exit__(self, type, value, traceback):
#        #スレッドが停止するのを待つ
#        self.stop_event.set()
#        self.thread.join()


def CB(request):
    global speech_loop
    speech_loop = request.data

    
if IS_ROS_ACTIVE:
    google_start_sub = rospy.Subscriber('google_req/start',String,main)
    google_stop_sub = rospy.Subscriber('google_req/stop',String,CB)


if __name__ == '__main__':
    if IS_ROS_ACTIVE:
        rospy.init_node('speech_recog')
        rospy.spin()
    else:
        create_speech_recog_thread()

        main_thread_name = threading.currentThread()
        while True:
            time.sleep(1)

            tlist = threading.enumerate()
            #if len(tlist) &lt; 2: break
            for t in tlist:
                if t is main_thread_name: continue
                print (t)


