#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import codecs
import Grammar
import traceback

from audio_module_msg.msg import AudioLUData, AudioLUResult, AudioSentence
from audio_module_grammar_recognition.srv import SetGrammar, SetGrammarResponse


class GrammarBasedLU(object):
    def __init__(self,):
        #ros
        rospy.init_node('GrammarRecognition')
        rospy.Service('GrammarRecognition/set_grammar', SetGrammar, self.set_gram_srv)
        self.pub_recogres = rospy.Publisher('AudioGrammarInfo', AudioLUResult, queue_size=10)
        rospy.Subscriber('AudioSentence', AudioSentence, self.recog_callback)

        self.gram = Grammar.Grammar()
        self.vaild_gram_id = []

        rospy.spin()

    def recog_callback(self, msg):
        if len(msg.sentences) <= 1:
            try:
                sentence = msg.sentences[0].decode('utf8')
                rospy.loginfo('-----recognition result-----')
                m = self.gram.match(sentence)
                if m:
                    res = AudioLUResult()
                    d = AudioLUData()
                    for n, nid in zip(m[1], m[2]):
                        d.noun_id.append(nid)
                        d.noun_str.append(n)
                    d.sentence = sentence
                    d.sentence_id = m[0]
                    res.sentences = msg
                    res.results.append(d)
    
                    print sentence
                    print d.sentence_id
                    print d.noun_str
    
                    if len(self.vaild_gram_id) == 0 or (m[0] in self.vaild_gram_id):
                        self.pub_recogres.publish(res)
                    else:
                        rospy.loginfo('文法マッチなし')
                else:
                    rospy.loginfo('文法マッチなし')
            except:
                print traceback.format_exc()
        
        else:
            try:
#                sentences = []
#                sentences.append(msg.sentences[i] for i in range(len(msg.sentences)))
                sentences = [ _s for _s in msg.sentences ]
                num_sentences = len(sentences)
                rospy.loginfo('-----recognition result-----')
#                for i in range(len(msg.sentences)):
                for i in range(num_sentences):
                    m = self.gram.match(sentences[i])
                    if m:
                        res = AudioLUResult()
                        d = AudioLUData()
                        for n, nid in zip(m[1], m[2]):
                            d.noun_id.append(nid)
                            d.noun_str.append(n)
                        d.sentence = sentences[i]
                        d.sentence_id = m[0]
                        res.sentences = msg
                        res.results.append(d)
    
                        if len(self.vaild_gram_id) == 0 or (m[0] in self.vaild_gram_id):
                            self.pub_recogres.publish(res)
                            break
                        else:
                            rospy.loginfo('no match grammar')
                    else:
                        rospy.loginfo('no match grammar')
            except:
                rospy.logerr(traceback.format_exc())

    def set_gram_srv(self, req):
        codecs.open('temp_gram.txt', 'w', 'utf8').write(req.grammar.decode('utf8'))
        self.vaild_gram_id = req.vaild_gram_id
        self.gram.load('temp_gram.txt')
        return SetGrammarResponse(True)

    def __enter__(self,):
        return self

    def __exit__(self, *args, **kwars):
        pass

def main():
    with GrammarBasedLU() as gram:
        gram.__init__()


if __name__ == '__main__':
    """
    """
    main()