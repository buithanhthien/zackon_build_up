import os
import sys
from contextlib import contextmanager

@contextmanager
def suppress_stderr():
    null_fd = os.open(os.devnull, os.O_RDWR)
    save_fd = os.dup(2)
    os.dup2(null_fd, 2)
    try:
        yield
    finally:
        os.dup2(save_fd, 2)
        os.close(null_fd)
        os.close(save_fd)

import speech_recognition as sr
with suppress_stderr():
    print("Init microphone...")
    m = sr.Microphone()
    print("Microphone initialized!")
