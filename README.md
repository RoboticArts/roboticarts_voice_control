# roboticarts_voice_control


**Autor**: Robert Vasquez Zavaleta

*Robotic Arts, All Rights Reserved 2017-2019*

Software License Agreement (BSD License)

**Description**: Package to teleop robots using voice recognition in ROS. Tested on Kinetic

**Disclaimer**: This package uses the Google and Sphinix speech recognition engine, so the quality of the recognition will depend on its current development.


## 1. Requisites

* Ubuntu with ROS Kinetic or Melodic
* Python 3.5 or higher

## 2. Dependences

roboticarts_voice_control package depends on the following python packages:

* SpeechRecognition
* PyAudio
* PocketSphinx

## 3. Installation

Install roboticarts_voice_control package

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/RoboticArts/roboticarts_voice_control.git
```

Install speech recognition library
```
$ sudo pip3 install SpeechRecognition
```

Install CMU Sphinx to offline speech recognition
```
$ sudo python3 -m pip install --upgrade pip setuptools wheel
$ sudo pip3 install --upgrade pocketsphinx
```

Install PyAudio to use the microphone 
```
$ sudo pip3 install pyaudio
$ sudo apt-get install portaudio19-dev python3-all-dev
```
IMPORTANT: this library works with ALSA driver. Make sure that you first
device detected is the microphone that you will use. Read troubleshooting for
more information

## 4. Usage

This package works online and offline.

### 4.1 Basic usage

In a terminal, launch the package.

```
$ roslaunch roboticarts_voice_control voice_control.launch
```

In other terminal, listen the topic:

```
$ rostopic echo /voice_recognition/raw 
```

Say something! ROS will publish your words. 

```
data: "hello I am speaking"
---
```

### 4.2 Select speech recognition engine

This package implements two engines. Select one of these two:

* **Google Speech Recognition API:** works very well and supports various languages. The downside is it needs internet. A
     list of available languages can be found [here](http://stackoverflow.com/a/14302134).

    ```
    $ engine:=google
    ```

* **CMU Sphinx:** works offline. The downside is that it requires configuring the voice model to work well. The language that comes when installing this library is English. All other languages ​​need to be installed manually. Available launguages [here](https://cmusphinx.github.io/wiki/faq/#q-which-languages-are-supported). 

    ```
    $ engine:=sphinx
    ```

### 4.3 Select the language

The language depends of engine used:

```
$ language:=en-US
```


### 4.4 Select the operational mode

This package can work in three modes:

* **raw mode:** the node will publish all recognized words in ```/voice_recognition/raw``` topic. This mode is always working, you just have to subscribe to the topic

* **command mode:** the node will only publish the recognized words that are defined in the ```voice_keyword.yaml```. For that, you have to subscribe to ```/voice_recognition/command``` An example of ```voice_keyword.yaml``` file is:

    ```
    commands: ['forward',
               'backward',
               'turn left',
               'turn right']
    ```

    You must create a configuration file for each language you use in the commands. In ```roboticarts_voice_control/config``` create a directory for each language,so the package can differentiate them. For example:

    English keywords

    ```
    roboticarts_voice_control/config/english/voice_control
    ```
    Spanish keywords

    ```
    roboticarts_voice_control/config/spanish/voice_control
    ```

    Now you can tell the package what config file will use it

    ```
    $ keyword_idiom:=english
    ```

* **alias mode:** this mode works just like ```command mode```. However, every time you say a command, you must first say the alias. Commands are ignored until you say the alias. This mode works just like the Google Assistant word "OK-Google". You can also configure the time that the package will wait for a command after saying the alias.

    ```
    $ alias:=robot
    $ alias_timeout:=10
    ```

To select one of these modes, do:

For command mode:
```
$ use_alias:=false
```
For alias mode:
```
$ engine:=true
```
Mode raw mode always works

### 4.5 Launch the package

If you are going to implement it in a launch file:
```
<include file="$(find roboticarts_voice_control)/launch/voice_control.launch">

    <arg name = "engine" value="google"/>
    <arg name = "language" value="en-US"/>
    <arg name = "use_alias" value="robot"/>
    <arg name = "keyword_idiom" value="english"/> 
    <arg name = "alias" value="robot"/> 
    <arg name = "alias_timeout" value="10"/>
   
</include>
```

If you are going to throw it in the terminal:

```
$ roslaunch roboticarts_voice_control voice_control.launch engine:=google language:=en-US use_alias:=false keyword_idiom:=english alias:=robot alias_timeout:=10
```

You can ommit any value, default values are:
```
$ engine = "google"
$ language = "en-US"
$ use_alias = false
$ keyword_idiom = english
$ alias = "robot"
$ alias_tiemout = 10
```


### 5. Troubleshooting

PyAudio works with ALSA driver. The first device used by this driver is the
default driver. If you get a message like this when you launch the node and **the
node does not detect you microphone**, you have to change the default device by your
microphone.

```
ALSA lib pcm_dsnoop.c:606:(snd_pcm_dsnoop_open) unable to open slave
ALSA lib pcm_dmix.c:1029:(snd_pcm_dmix_open) unable to open slave
ALSA lib pcm.c:2266:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.rear
ALSA lib pcm.c:2266:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.center_lfe
ALSA lib pcm.c:2266:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.side
ALSA lib pcm_dmix.c:1029:(snd_pcm_dmix_open) unable to open slave
```

First, check the list of sound input devices

$ aplay -l

You will get some like this. In this case the laptop microphone is the ```card 1```. If
you had other microphones, like USB, you would see them.

```
**** Lista de PLAYBACK dispositivos hardware ****
card 0: HDMI [HDA Intel HDMI], device 3: HDMI 0 [HDMI 0]
  Subdevice: 1/1

card 1: PCH [HDA Intel PCH], device 0: ALC255 Analog [ALC255 Analog]
  Subdispositivos: 0/1
  Subdevice #0: subdevice #0
```

Second, set as default you microphone. Go to ```/etc/asound.conf```  If file does not exist,
then create it. Copy and paste the following. Change ```card ``` by the card that you microphone is
using. In this example is ``` card 1 ```.

```
pcm.!default {
    type hw
    card 1
}

ctl.!default {
    type hw           
    card 1
}

```

Reboot you computer. When you launch the package again probably you will see the same errors but
you microphone will work. They are just telling something is wrong in the configuration but they 
do not cause any real problem.


### Useful links 

[Change the default input device](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=10&cad=rja&uact=8&ved=2ahUKEwi4oKGdlfjoAhWvCWMBHebKB4sQygQwCXoECAIQCg&url=https%3A%2F%2Fwiki.archlinux.org%2Findex.php%2FAdvanced_Linux_Sound_Architecture%23Set_the_default_sound_card&usg=AOvVaw1kL6Rl14pgDWRPtyshw4lv)

[Clear some errors](https://stackoverflow.com/questions/31603555/unknown-pcm-cards-pcm-rear-pyaudio)

[Fix alsa config](https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time)
