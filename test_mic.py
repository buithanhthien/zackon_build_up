import speech_recognition as sr
import os

print("Microphone test:")
r = sr.Recognizer()
mic = sr.Microphone()

print("Please make some noise for 2 seconds...")
with mic as source:
    r.adjust_for_ambient_noise(source, duration=1.0)
    print("Listening...")
    try:
        audio = r.listen(source, timeout=2.0, phrase_time_limit=3.0)
        print(f"Captured audio. Size: {len(audio.get_raw_data())} bytes")
    except sr.WaitTimeoutError:
        print("Timeout! No speech detected.")
