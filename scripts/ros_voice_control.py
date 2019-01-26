#!/usr/bin/env python

"""This module is a simple demonstration of voice control
for ROS turtlebot using pocketsphinx
"""

import argparse
import roslib
import rospy
import os
from std_msgs.msg import String
import sys
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio


class ASRControl:
    """Simple voice control interface for ROS turtlebot

    Attributes:
        model: model path
        lexicon: pronunciation dictionary
        pub: where to send commands (default: 'mobile_base/commands/velocity')

    """
    def __init__(self, script_path):
        # initialize ROS

        rospy.init_node('speech_to_text')
        rospy.on_shutdown(self.shutdown)
        # you may need to change publisher destination depending on what you run

        self.pub_ = rospy.Publisher('speech_to_text', String, queue_size=10)

        # initialize pocketsphinx
        config = Decoder.default_config()
        config.set_string('-hmm', model)
        # config.set_string('-dict', lexicon)

        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)
        stream.start_stream()

        self.decoder = Decoder(config)
        self.decoder.start_utt()

        while not rospy.is_shutdown():
            buf = stream.read(1024)
            if buf:
                self.decoder.process_raw(buf, False, False)
            else:
                break
            self.parse_asr_result()

    def parse_asr_result(self):
        """
        move the robot based on ASR hypothesis
        """
        if self.decoder.hyp() != None:
            print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                for seg in self.decoder.seg()])
            print ("Detected keyphrase, restarting search")
            seg.word = seg.word.lower()
            self.decoder.end_utt()
            self.decoder.start_utt()
            # you may want to modify the main logic here
            if seg.word.find("yes") > -1:
                self.pub_.publish("yes")
            if seg.word.find("no") > -1:
                self.pub_.publish("no")

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop ASRControl")
        rospy.sleep(1)

if __name__ == '__main__':
    
    # path=ws+'/vocab/stt.dic'

    # my_lexicon =  os.path.abspath(os.path.join(__file__ ,"../vocab/stt.dic"))
    # print my_lexicon
    # parser = argparse.ArgumentParser(
    #     description='Get speech input and publish it.')
    # parser.add_argument('--model', type=str,
    #     default='/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k',
    #     help='''acoustic model path
    #     (default: /usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k)''')
    # parser.add_argument('--lexicon', type=str,
    #     default='stt.dic',
    #     help='''pronunciation dictionary
    #     (default: stt.dic)''')

    # args = parser.parse_args()
    ASRControl(sys.path[0])
    # ASRControl(args.model, args.lexicon)

