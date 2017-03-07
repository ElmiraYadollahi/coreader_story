import time
from naoqi import ALProxy
import rospy
import qi

ROBOT_IP = 'nao.local'
rospy.init_node("audio_speechrecognition")


app = qi.Application()
session = qi.Session()
session.connect("tcp://nao.local:9559")
memory = session.service("ALMemory")
asr = session.service("ALSpeechRecognition")


# Creates a proxy on the speech-recognition module
#asr = ALProxy("ALSpeechRecognition", ROBOT_IP, 9559)
#memory = ALProxy("ALMemory", ROBOT_IP, 9559)

asr.setLanguage("English")

# Example: Adds "yes", "no" and "please" to the vocabulary (without wordspotting)
vocabulary = ["yes", "no", "please"]
#asr.setVocabulary(vocabulary, False)

# Start the speech recognition engine with user Test_ASR
asr.subscribe("Test_ASR")
print 'Speech recognition engine started'

#eventName = "wordRec"
#asr.WordRecognized(eventName, )
for i in range(10):

	data = memory.getData("SpeechDetected")

	if data == True:
		print data

	data = memory.getData("WordRecognized")

	if data != None:
		print data

	#time.sleep(2)

def on_recognized(event):
	if event > 0:
		print event

#subscriber = memory.subscribe("SpeechDetected")
#subscriber.signal.connect(on_recognized)

#app.run()


time.sleep(5)
asr.unsubscribe("Test_ASR")
#rospy.spin()
isPaused = True
asr.pause(isPaused)
