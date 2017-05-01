# encoding: utf8
from __future__ import unicode_literals
import Tkinter as tk
import win32gui,win32con
import time
import tkFileDialog
import os

from ctypes import *
user32 = windll.user32

def click(x, y):
    user32.SetCursorPos(x,y) #カーソル座標移動
    user32.mouse_event(0x2,0,0,0,0) #左クリックを押す
    time.sleep(0.2)                 #0.2秒待機
    user32.mouse_event(0x4,0,0,0,0) #左クリックを放す


def start_speech_recog( recog_callback=None, setdic_callback=None ):
    root = tk.Tk()

    entry = tk.Entry()
    entry.pack()

    enable_speechrecog = tk.BooleanVar()
    tk.Checkbutton(text = "音声認識有効化", variable = enable_speechrecog).pack()


    def activate():
        if not enable_speechrecog.get():
            root.after(100,activate)
            return

        hwnd = int(root.wm_frame(),0)
        win32gui.SetWindowPos(hwnd, win32con.HWND_TOPMOST, 0,0,0,0, win32con.SWP_NOMOVE | win32con.SWP_NOSIZE)

        if win32gui.GetForegroundWindow() != hwnd:
            x = root.winfo_x()
            y = root.winfo_y()
            print "ウィンドウアクティブ化"
            #print x, y
            click(x+40, y+40)



        entry.focus_set()
        root.after(200,activate)



    length = 0
    def get_recogres():
        global length

        if not enable_speechrecog.get():
            length = 0
            root.after(100,get_recogres)
            return

        recogres = entry.get()
        if len(recogres) and len(recogres)==length:
            print "認識文：",recogres
            if recog_callback:
                recog_callback( recogres )
            entry.delete(0,tk.END)
            length = 0
        elif len(recogres):
            # 一回で取れない場合があるのでもう一回
            length = len(recogres)
            print "認識文：",recogres
            root.after(200,get_recogres)
            return

        root.after(100,get_recogres)



    def set_dict():
        filename=tkFileDialog.askopenfilename(filetypes=[("テキストファイル","*.txt")],initialdir=os.curdir)
        if setdic_callback:
            setdic_callback(filename)

    tk.Button(text="辞書変更" , command=set_dict).pack()

    activate()
    get_recogres()
    root.mainloop()

def main():
    StartSpeechRecognition()

if __name__ == '__main__':
    main()