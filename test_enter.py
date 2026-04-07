import speech_recognition as sr
import os

print("Creating object...")
m = sr.Microphone()
print("Object created.")

print("Entering context manager...")
with m as source:
    print("Inside context manager.")
