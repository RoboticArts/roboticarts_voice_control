#!/usr/bin/env python3

"""
Copyright (c) 2020 Robotic Arts Industries

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
 Author:  Robert Vasquez Zavaleta
"""

"""
---------------------------------------------------------------------
Robotic Arts Industries
All Rights Reserved 2017-2020
---------------------------------------------------------------------
Package: roboticarts_voice_control
File: voice_control.py

Description:

    This file allows the recognition of voice commands to control robots. Commands
    that want to be recognized must be defined in the voice_keywords.yaml file. When
    a command is recognized, it is published in voice_recognition/command topic. On 
    the other hand all recognized words are published in /voice_recognition/raw topic.
    
    Two speech recognition engines have been implemented:

            -   Google Speech Recognition: online, works well, various languages
            -   CMU Sphinx: offline, needs configuration to work well, only english

----------------------------------------------------------------------
		Version: 0.0.1						| Last Modification: 16/07/2020
		Author:  Robert Vasquez Zavaleta
		Contact: roboticarts1@gmail.com
"""

import speech_recognition as sr
from  std_msgs.msg import String 
import rospy
import sys
import time
import signal

class RoboticartsVoiceControl:

    def __init__(self):

        #Declare internal variables
        self.r = sr.Recognizer()

        self.isAudioAvailable = False
        self.iteration = 0
        self.aliasElapsedTime = 0
        self.alias_time_init = 0
        self.aliasFlag = False

        # Set signal to exit using terminal
        signal.signal(signal.SIGINT, self.sigint_handler)

        rospy.init_node('voice_control_node', anonymous=False)
        self.ros_voice_command = rospy.Publisher('voice_recognition/command', String, queue_size=1)
        self.ros_voice_raw = rospy.Publisher('voice_recognition/raw', String, queue_size=1)

        self.get_ros_params()

        self.checkConfiguration()
 
        rospy.loginfo(self.node_name + " already!")

    def sigint_handler(self, sig, frame):
        print("")

    def get_ros_params(self):

        # Get name of this node
        self.node_name = rospy.get_name()
        
        # Default values
        param_dict = { self.node_name + "/commands": [''],
                       self.node_name + "/engine": 'google',
                       self.node_name + "/alias": 'robot',
                       self.node_name + "/use_alias": False,
                       self.node_name + "/alias_timeout": 10,
                       self.node_name + "/language": 'en-US'
                      }

        for param in param_dict:

            if rospy.has_param(param):
                param_dict[param] = rospy.get_param(param)
            else:
                rospy.logwarn("Param " + param + " not specified, set to default value: " + str(param_dict.get(param)))

        self.keyword_list = param_dict.get(self.node_name + "/commands")
        self.engine = param_dict.get(self.node_name + "/engine")
        self.alias = param_dict.get(self.node_name + "/alias")
        self.use_alias = param_dict.get(self.node_name + "/use_alias")
        self.aliasTimeout = param_dict.get(self.node_name + "/alias_timeout")
        self.language = param_dict.get(self.node_name + "/language")

    def checkConfiguration(self):

        engine = self.engine
        language = self.language

        if engine == "google":

            if not "-" in language and len(language) != 2:
                rospy.logerr("Language " + language + " not detected by Google")
                sys.exit(1)
            

        elif engine == "sphinx":

            if not "en-" in language:
                rospy.logerr("Error: Language " + language + " not supported by Sphinx")
                sys.exit(1)

        else:
                rospy.logerr("Speech recognition engine not found")
                sys.exit(1)

        rospy.loginfo("Working with " + engine + " engine using " + language + " language" )


    def recordAudio(self, source):

        self.isAudioAvailable = False
        audio = 0

        self.iteration+=1
        rospy.loginfo("--------------------------------------")
        rospy.loginfo("Iteration: %s", self.iteration)

        try:
            audio = self.r.listen(source,timeout=5, phrase_time_limit=2)
            self.isAudioAvailable = True
            rospy.loginfo("Audio recording: successful")
        
        except sr.WaitTimeoutError as e:        
            
            rospy.loginfo("Audio recording: timeout; {0}".format(e))

        return audio


    def audioAvailable(self):
        return self.isAudioAvailable


    def findKeyword(self, rawText):

        if self.use_alias:

            if self.alias.lower() in rawText.lower():

                return self.alias

        for keyword in self.keyword_list:

            if keyword.lower() in rawText.lower():

                return keyword
    
        return ''


    def speechRecognition(self, audio):

        rawText = ''
        engine = self.engine

        if engine == "google":
            rawText = self.getGoogleVoiceCommand(audio)  

        elif engine == "sphinx":
            rawText = self.getSphinxVoiceCommand(audio)

        return rawText


    def getGoogleVoiceCommand(self, audio):

        rawText = ''

        try:
            rawText = self.r.recognize_google(audio, language=self.language)
            rospy.loginfo("Google Speech Recognition thinks you said " + rawText)
        
        except sr.UnknownValueError:
            rospy.loginfo("Google Speech Recognition could not understand audio")

        return rawText


    def getSphinxVoiceCommand(self,audio):

        rawText = ''
        
        try:
            rawText = self.r.recognize_sphinx(audio)
            rospy.loginfo("Sphinx thinks you said " + rawText)
        
        except sr.UnknownValueError:
            rospy.loginfo("Sphinx could not understand audio")
        

        return rawText

    def publishRosRaw(self, rawTextDetected):

        self.ros_voice_raw.publish(rawTextDetected)
        rospy.sleep(0.01)

    def publishRos(self, keyword):

        self.ros_voice_command.publish(keyword)
        rospy.sleep(0.01)


    def publishRosAlias(self,keyword):

        if keyword == self.alias:
            
            self.alias_time_init = time.time()
            self.publishRos(self.alias)
            self.aliasFlag = True
            rospy.loginfo("ROS voice command: activated")

        elif self.aliasFlag:

                self.publishRos(keyword)
                self.aliasFlag = False
                rospy.loginfo("ROS voice command: disabled")


    def checkAliasTiemout(self):

        if self.aliasFlag:

            self.aliasElapsedTime = time.time() - self.alias_time_init

            if self.aliasElapsedTime > self.aliasTimeout:

                self.publishRos('')
                self.aliasFlag = False
                rospy.loginfo("ROS voice command: disabled")


    def run(self):

        with sr.Microphone(sample_rate=44100) as source:

            self.r.adjust_for_ambient_noise(source) 
            
            while not rospy.is_shutdown():

                audio = self.recordAudio(source)    

                if self.audioAvailable():

                    rawText = self.speechRecognition(audio)
                    self.publishRosRaw(rawText)
                    keyword = self.findKeyword(rawText) 

                    if keyword:

                        rospy.loginfo("Keyword " + keyword + " recognized")

                        if(self.use_alias):

                            self.publishRosAlias(keyword)
                        else:

                            self.publishRos(keyword)

                    else:
                        rospy.loginfo("Keyword not recognized")


                self.checkAliasTiemout() # Used if use_alias is true

if __name__ == '__main__':

    roboticarts_voice_control = RoboticartsVoiceControl()

    try:
        roboticarts_voice_control.run()

    except rospy.ROSInterruptException:
        pass
