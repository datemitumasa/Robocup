# coding: utf-8
from __future__ import unicode_literals
import ctypes
import time
import codecs
import msvcrt
import sys

def count_down( sec ):

    print '5秒後に開始します'

    for i in range(sec):
        print '%d\r'%(sec-i),
        time.sleep(1)

    print

def push_key( vk ):
    ctypes.windll.user32.keybd_event(vk,0,0,0)
    time.sleep(0.1)

def release_key( vk ):
    ctypes.windll.user32.keybd_event(vk,0,2,0)
    time.sleep(0.1)


from Tkinter import Tk
_r = Tk()
_r.withdraw()
def put_clipboard(txt):
    _r.clipboard_clear()
    _r.clipboard_append(txt)


def main():
    start = False
    words = codecs.open("words.txt").readlines()

    n = 0
    if len(sys.argv)>1:
        n = int(sys.argv[1])
        words = words[n:]

    for i in range(len(words)):
        if not start:
            print "開始するにはEnterを押してください"
            raw_input()
            start = True
            count_down(5)

        w = words[i].replace("\n","").replace("\r","")
        print i+n,w, "を入力"

        put_clipboard(w)
        push_key(0x11) # ctrl
        push_key(0x56) # v
        release_key(0x56)
        release_key(0x11)

        push_key(0x0d)
        release_key(0x0d)
        time.sleep(1.0)











if __name__ == '__main__':
    main()