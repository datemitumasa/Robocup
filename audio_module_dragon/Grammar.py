# encoding: utf8
from __future__ import unicode_literals
import re
import codecs

class Grammar:
    def __init__(self):
        self.nounIdFinder = re.compile( "\$noun[A-z0-9-]*" )
        self.grammars = {}
        self.noun2id = {}

    def normalize( self, s ):
        s = s.strip()

        # 不要な文字を取り除く
        s = s.replace( "\n" , "" )
        s = s.replace( "\r" , "" )

        # 句読点は全て全角にしてう白いスペースを入れる
        s = s.replace( u"，" , u"、 " )
        s = s.replace( u"、" , u"、 " )
        s = s.replace( u"。" , u"。 " )
        s = s.replace( u"．" , u"。 " )
        s = s.replace( "," , u"、 " )
        s = s.replace( "." , u"。 " )
        s = s.replace( "\t" , " " )

        while "  " in s:
            s = str.replace( "  " , " " )

        s = s.lower()

        return s


    def load(self, filename):
        lines = codecs.open( filename , "r" , "utf8" ).readlines()
        lines = [ l for l in lines ]

        self.grammars = {}
        self.noun2id = {}

        nouns = {}
        noun_re = {}


        # [NOUN]を読み込み
        start = False
        for line in lines:
            # 文法セクションの開始
            if "[NOUN]" in line:
                start = True
                continue

            if start:
                # 文法セクションの修了
                if "[" in line:
                    break
                elif "$" in line:
                    class_id = line.strip()
                    nouns[class_id] = []

                if ":" in line:
                    id,noun = line.split(":")
                    id = id.strip()

                    for n in noun.split("|"):
                        n = self.normalize(n)
                        nouns[class_id].append( n )
                        self.noun2id[n] = id
                        #print class_id ,id, n

        for id,n in nouns.items():
            noun_re[id] = "(" + "|".join(n) + ")"

        # [GRAMMAR]を読み込み
        start = False
        for line in lines:
            # 文法セクションの開始
            if "[GRAMMAR]" in line:
                start = True
                continue

            if start:
                # 文法セクションの修了
                if "[" in line:
                    break

                if ":" in line:
                    id,gram = line.split(":")
                    id = id.strip()
                    gram = self.normalize(gram)

                    # nounを正規表現に変換
                    for class_id in self.nounIdFinder.findall(gram):
                        gram = gram.replace( class_id, noun_re[class_id] )

                    # その他を正規表現に変換
                    gram = gram.replace("<" , "(?:")
                    gram = gram.replace(">" , ")")
                    gram = gram.replace("*" , ".*?")
                    gram = gram.replace( " " , "\s*?" )
                    gram = ".*?" + gram + ".*?"
                    gram = gram.lower()

                    self.grammars[id] = re.compile(gram)
                    print "RegEx:",id,"->",gram

    def match(self, s ):
        s = s.lower()
        matched_gramid = ""
        matched_noun = []
        mathced_nounid = []
        for id, g in self.grammars.items():
            m = g.match(s)

            if m:
                matched_gramid = id
                matched_noun = m.groups()
                for n in matched_noun:
                    # print n
                    mathced_nounid.append( self.noun2id[n] )
                return matched_gramid, matched_noun, mathced_nounid

        return None


def main():
    g = Grammar()

    g.load( "testgram_all.txt" )
    print g.match( "コーヒーを中村に持って行って" )
    print g.match( "人を探して" )
    print g.match( "こんにちは" )

    g.load( "testgram_all.txt" )
    print g.match( "take coffee to nakamura" )
    print g.match( "find a person" )
    print g.match( "Go to the dining table" )

    return


    g = re.compile( "(コーラ|ジュース)を.*?持ってきて" )
    s = "コーラを持って持ってきて"
    print g.match(s)
    for i in g.findall(s):
        print i

if __name__ == '__main__':
    main()