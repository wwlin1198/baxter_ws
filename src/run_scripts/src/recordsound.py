#!/usr/bin/env python
"""Create a recording with arbitrary duration.

The soundfile module (https://PySoundFile.readthedocs.io/) has to be installed!

"""
import argparse
import tempfile
import Queue as queue
import sys
import time
import thread

import sounddevice as sd
import soundfile as sf
import numpy  # Make sure NumPy is loaded before it is used in the callback
assert numpy  # avoid "imported but unused" message (W0611)


q = queue.Queue()
timer = 0
stop = False

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status)
    q.put(indata.copy())

def count(t,s):
    global timer
    global stop 
    while timer <=10:
        if timer == 10:
            stop = True 
        timer+=1
        time.sleep(1)


def record_audio():
    global stop
    try:
        # Make sure the file is opened before recording anything:
        with sf.SoundFile('test.wav', mode='x', samplerate=48000,
                          channels=1) as file:
            with sd.InputStream(samplerate=48000, 
                                channels=1, callback=callback):
                print('#' * 80)
                print('press Ctrl+C to stop the recording')
                print('#' * 80)
                while not stop:
                    file.write(q.get())
                
    except KeyboardInterrupt:
        print('\nRecording finished: ' + repr('test.wav'))
        parser.exit(0)
    except Exception as e:
        parser.exit(type(e).__name__ + ': ' + str(e))

thread.start_new_thread(record_audio, tuple())
# thread.start_new_thread(count, (4,3))
count(4,3)
# record_audio(5,5)