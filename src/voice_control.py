#!/usr/bin/env python3

import speech_recognition as sr

# obtain audio from the microphone
r = sr.Recognizer()
 
count = 0

with sr.Microphone(sample_rate=44100) as source:
    r.adjust_for_ambient_noise(source)  # here
    while True:
        count+=1
        print("Say something! ", count)

        try:
            audio = r.listen(source,timeout=5, phrase_time_limit=2)
            ok = True
        except sr.WaitTimeoutError as e:
            print("Timeout; {0}".format(e))
            ok = False

        if ok:
            try:
                print("Google Speech Recognition thinks you said " + r.recognize_google(audio, language="es-ES"))
            except sr.UnknownValueError:
                print("Google Speech Recognition could not understand audio")

