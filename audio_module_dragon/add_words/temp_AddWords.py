# coding: utf-8
from __future__ import unicode_literals
import ctypes
import time
import codecs
import msvcrt
import sys

import win32com.client
import win32api
import win32clipboard as CB

shell = win32com.client.Dispatch('WScript.Shell')


def count_down( sec ):

    print '5秒後に開始します'

    for i in range(sec):
        print '%d\r'%(sec-i),
        time.sleep(1)

    print


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

        CB.OpenClipboard()
        CB.EmptyClipboard()
        CB.SetClipboardText(w, CB.CF_UNICODETEXT)
        CB.CloseClipboard()
        shell.SendKeys('^v')
        win32api.Sleep(100)
        shell.SendKeys('~')
        time.sleep(1)


if __name__ == '__main__':
    main()