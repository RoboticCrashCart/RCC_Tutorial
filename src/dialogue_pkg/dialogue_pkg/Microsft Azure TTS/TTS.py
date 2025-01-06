import os
import io
import time
from dotenv import load_dotenv

from pydub import AudioSegment
from pydub.playback import play

import azure.cognitiveservices.speech as speechsdk

def save_audio_to_file(audio_data, filename):
    with open(filename, 'wb') as file:
        file.write(audio_data)

def create_ssml(text, voice, style, styledegree="1", role=None):
    ssml = f"""
    <speak version='1.0' xmlns='http://www.w3.org/2001/10/synthesis' xmlns:mstts='https://www.w3.org/2001/mstts' xml:lang='en-US'>
        <voice name='{voice}'>
            <mstts:express-as style='{style}'>
                {text}
            </mstts:express-as>
        </voice>
    </speak>
    """
    return ssml

load_dotenv()

api_key = os.getenv('api_key')
endpoint = os.getenv('endpoint')
region = os.getenv('region')

speech_config = speechsdk.SpeechConfig(subscription=api_key, region=region)
audio_config = speechsdk.audio.AudioOutputConfig(use_default_speaker=True)

speech_sythesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config, audio_config=audio_config)

texts = [
    "Kindly, check the drawer 1",
    "Kindly, check the drawer 2",
    "Kindly, check the drawer 3",
    "Kindly, check the drawer 4",
    "Kindly, check the drawer 5",
    "Kindly, check the drawer 6"
]

voice_M = 'en-US-AndrewMultilingualNeural'
voice_F = 'en-US-AvaMultilingualNeural'
style = 'assistant'
styledegrees = ['5.1', '2', '3', '4', '2.5', '1']
roles = None

for i, text in enumerate(texts):
    ssml_M = create_ssml(text, voice_M, style)
    speech_synthesis_result = speech_sythesizer.speak_ssml_async(ssml_M).get()
    
    if speech_synthesis_result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
        print(f'Speech synthesized successfully for {text}.')
        audio_data = speech_synthesis_result.audio_data
        save_audio_to_file(audio_data, f'drawer_M{i+1}.wav')
    else:
        print(speech_synthesis_result.cancellation_details.error_details)
    
    
    ssml_F = create_ssml(text, voice_F, style)
    speech_synthesis_result = speech_sythesizer.speak_ssml_async(ssml_F).get()
    
    if speech_synthesis_result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
        print(f'Speech synthesized successfully for {text}.')
        audio_data = speech_synthesis_result.audio_data
        save_audio_to_file(audio_data, f'drawer_F{i+1}.wav')
    else:
        print(speech_synthesis_result.cancellation_details.error_details)

print("All audio files have been synthesized and saved successfully.")
