# encoding: utf8
from __future__ import unicode_literals


import os
import rospy
from audio_module_msg.msg import *
from audio_module_grammar_recognition.srv import *



gram = """
[GRAMMAR]
hello : $noun_hello
my_name : * my name is $noun_person *
find : find $noun_object

[NOUN]
$noun_hello
hello : hello
bye : good bye
morning : good morning

$noun_person
person1 : nakamura
person2 : tanaka
person3 : taro

$noun_object
object1: coke
object2: coffee
object3: beer
"""

def callback_lv(res):
    try:
        print "--LV Result--"
        for i,s in enumerate(res.sentences):
            print i,s
        print "----------"
    except:
        print traceback.format_exc()

def callback_sv(res):
    try:
        print "--SV Result--"
        print res.results[0].sentence
        print "sentenceid:",res.results[0].sentence_id
        print "nounid:",res.results[0].noun_id
        print "nounstr:",res.results[0].noun_str
        print "-----"
    except:
        print traceback.format_exc()

def speech_recog_sample():
    rospy.init_node('speech_recog_sample')

    rospy.Subscriber("DragonSpeech/sentence", AudioSentence, callback_lv)
    rospy.Subscriber("DragonSpeech/luresult", AudioLUResult, callback_sv)

    SetGram = rospy.ServiceProxy('DragonSpeech/set_grammar', SetGrammar)
    SetGram( gram, [ "hello", "find" ] )

    rospy.spin()


def main():

    speech_recog_sample()


if __name__ == '__main__':
    main()
