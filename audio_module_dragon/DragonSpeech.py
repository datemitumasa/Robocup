# coding: utf-8
from __future__ import unicode_literals
import sys
sys.path.append( "ros" )

import os
import rospy
import IPSettingGUI
import sys
import std_msgs
import codecs
import Grammar
import SRWindow
import winsound
import re


from std_msgs.msg import String
from audio_module_msg.msg import *
from audio_module_grammar_recognition.srv import *


class DragonSpeech():
    def __init__(self):

        rospy.init_node("SpeechRecognition")
        rospy.on_shutdown( self.shutdown )

        rospy.Service( "DragonSpeech/set_grammar" , SetGrammar, self.set_gam_srv )
        self.pub_recogres = rospy.Publisher('DragonSpeech/sentence', AudioSentence , queue_size=10)
        self.pub_lures = rospy.Publisher( "DragonSpeech/luresult", AudioLUResult, queue_size=10 )

        self.gram = Grammar.Grammar()
        self.gram.load( "sample.txt" )
        self.valid_gram_id = []

        #
        pattern = "|".join([ "%c%c" % (A,a) for A,a in zip( range(ord("A"), ord("Z")+1), range(ord("a"), ord("z")+1) ) ])
        print pattern
        self.large_letter_finder = re.compile(pattern)

        SRWindow.start_speech_recog( self.recog_callback, self.set_gram_file )


    """
    def play_sound( self, filename ):

        if self.play_soud_srv==None:
            try:
                self.play_soud_srv = rospy.ServiceProxy('/audio_player/Play', Play)
            except:
                pass


        try:
            self.play_soud_srv( filename )
        except:
            print "service call failed"
            try:
                winsound.PlaySound( filename , winsound.SND_FILENAME)
            except:
                pass
    """

    def recog_callback(self, sentence ):
        print "---- recognition result ----"
        # if len(self.large_letter_finder.findall(sentence)):
        #    print "!!!!! 大文字置換 !!!!!"
        #    sentence = self.large_letter_finder.sub("\\1",sentence[1:])
        items = self.large_letter_finder.findall(sentence)
        if len(items):
            print "!!!!! 大文字置換 !!!!!"
            for i in items:
                sentence = sentence.replace( i, i[1] )

        # 認識結果を送信
        recogres = AudioSentence()
        recogres.sentences.append( sentence )
        self.pub_recogres.publish( recogres )

        m = self.gram.match( sentence )
        print "認識結果", sentence
        if m:
            res = AudioLUResult()
            d = AudioLUData()
            for n, nid in zip(m[1], m[2]):
                d.noun_id.append(nid)
                d.noun_str.append(n)
            d.sentence = sentence
            d.sentence_id = m[0]
            res.sentences = recogres
            res.results.append(d)

            if len(self.valid_gram_id)==0 or (m[0] in self.valid_gram_id):
                self.pub_lures.publish( res )
                #self.play_sound( "beep_success.wav" )
            else:
                print "文法マッチなし"
        else:
            print "文法マッチなし"
        print "---------------------------"

    def set_gram_file(self,filename):
        print filename
        self.gram.load( filename )

    def set_gam_srv( self, req ):
        print"--- set grammar ---"
        codecs.open( "tmp_gram.txt" , "w" , "utf8" ).write(req.grammar.decode("utf8"))
        print req.grammar.decode("utf8")
        print "ID:",req.vaild_gram_id
        self.valid_gram_id = req.vaild_gram_id
        self.gram.load( "tmp_gram.txt" )
        print "------------------"
        return SetGrammarResponse(True)

    def change_grammaer(self,msg):
        pass

    def shutdown(self):
        pass

def main():
    if not IPSettingGUI.set_ip():
        print "キャンセルされました"
        return

    DragonSpeech()

if __name__ == '__main__':
    main()