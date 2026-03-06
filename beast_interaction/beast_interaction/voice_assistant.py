#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import speech_recognition as sr
from datetime import datetime
import os
import threading
import time
from ddgs import DDGS
import re
import urllib.request
from beast_msgs.srv import SetLEDBrightness

class VoiceAssistant(Node):
    def __init__(self):
        super().__init__('voice_assistant')

        # LED service client
        self.light_client = self.create_client(SetLEDBrightness, 'ugv/set_headlights')
        self._lights_timer = None

        self.get_logger().info('Voice Assistant ready!')
        self.get_logger().info('Say "Hey Beast" to activate...')

        self.listen_for_wake_word()

    # ---------- Light helpers ----------

    def lights_on(self):
        if not self.light_client.service_is_ready():
            self.get_logger().warn('Light service not available')
            return
        req = SetLEDBrightness.Request()
        req.brightness = 255.0
        self.light_client.call_async(req)
        self.get_logger().info('Headlights ON')

    def lights_off_delayed(self, delay=3.0):
        if self._lights_timer is not None:
            self._lights_timer.cancel()
        self._lights_timer = threading.Timer(delay, self._lights_off_callback)
        self._lights_timer.start()

    def _lights_off_callback(self):
        req = SetLEDBrightness.Request()
        req.brightness = 0.0
        self.light_client.call_async(req)
        self.get_logger().info('Headlights OFF')
        self._lights_timer = None

    def breath_light(self, stop_event):
        """Breathing light effect - runs until stop_event is set"""
        while not stop_event.is_set():
            for i in range(0, 255, 10):
                if stop_event.is_set():
                    break
                req = SetLEDBrightness.Request()
                req.brightness = float(i)
                self.light_client.call_async(req)
                time.sleep(0.05)
            for i in range(255, 0, -10):
                if stop_event.is_set():
                    break
                req = SetLEDBrightness.Request()
                req.brightness = float(i)
                self.light_client.call_async(req)
                time.sleep(0.05)

    # ---------- Existing methods ----------

    def speak(self, text):
        """Use espeak to make the robot speak — blocking so we know when it's done"""
        self.get_logger().info(f'Speaking: {text}')
        subprocess.run(['espeak', '-a', '200', '-s', '130', text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def record_audio(self, duration=3):
        filename_48k = f'/tmp/voice_48k_{datetime.now().strftime("%Y%m%d_%H%M%S")}.wav'
        filename_16k = f'/tmp/voice_{datetime.now().strftime("%Y%m%d_%H%M%S")}.wav'
        cmd = ['arecord', '-D', 'hw:0,0', '-f', 'S16_LE', '-c', '1', '-r', '48000', '-d', str(duration), filename_48k]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        sox_cmd = ['sox', filename_48k, '-r', '16000', filename_16k]
        subprocess.run(sox_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        os.remove(filename_48k)
        return filename_16k

    def transcribe_audio(self, filename):
        recognizer = sr.Recognizer()
        try:
            with sr.AudioFile(filename) as source:
                audio = recognizer.record(source)
            return recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio")
            return None
        except sr.RequestError as e:
            self.get_logger().error(f"Could not request results; {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Transcription error: {e}")
            return None

    def search_and_answer(self, question):
        try:
            self.get_logger().info(f'Searching for: {question}')
            with DDGS() as ddgs:
                results = list(ddgs.text(question, max_results=3))
            if results:
                answer = results[0].get('body', 'I could not find an answer')
                answer = re.sub(r'\[\d+\]', '', answer)
                answer = re.sub(r'\(\/.*?\/.*?\)', '', answer)
                sentences = answer.split('. ')
                return '. '.join(sentences[:2]).strip()
            else:
                return "I could not find an answer to that question"
        except Exception as e:
            self.get_logger().error(f'Search error: {e}')
            return "Sorry, I had trouble searching for that"
        
    def get_weather(self, question):
        try:
            # Extract just the location
            location = question.lower()
            for phrase in ['what is the current weather in', 'what is the current weather for',
                           'what is the weather report for', 'what is the weather in',
                           'what is the temperature in', 'weather in', 'temperature in',
                           'weather for', 'temperature for', 'what is the weather',
                            'what is the current']:
                location = location.replace(phrase, '').strip()

            # Remove emojis and clean up symbols
            weather = re.sub(r'[^\x00-\x7F]+', '', weather)
            weather = re.sub(r'\+', ' ', weather)        # Replace + with space
            weather = re.sub(r'\s+', ' ', weather)       # Collapse multiple spaces
            weather = weather.strip()

            url = f'http://wttr.in/{location.replace(" ", "+")}?format="%l:+%C+%t+humidity+%h"'
            result = subprocess.run(['curl', '-s', url], capture_output=True, text=True, timeout=30)
            weather = result.stdout.strip()
            self.get_logger().info(f'Raw weather: {weather}')  

            # Remove emojis and clean up symbols
            weather = re.sub(r'[^\x00-\x7F]+', '', weather)
            weather = weather.replace('+', ' ').strip()

            return weather
        except Exception as e:
            self.get_logger().error(f'Weather error: {e}')
            return "Sorry, I could not get the weather right now"
        
    def listen_for_wake_word(self):
        while rclpy.ok():
            self.get_logger().info('Listening for wake word...')
            audio_file = self.record_audio(duration=3)
            text = self.transcribe_audio(audio_file)

            if text:
                self.get_logger().info(f'Heard: "{text}"')
                text_lower = text.lower()

                if ('beast' in text_lower or 'based' in text_lower or
                        'hey beast' in text_lower or 'hey based' in text_lower or
                        'hey b' in text_lower):

                    self.get_logger().info('Wake word detected!')
                    self.lights_on()                          # Static bright on wake word
                    self.speak("Yes?")

                    self.get_logger().info('Listening for question...')
                    question_file = self.record_audio(duration=5)
                    question = self.transcribe_audio(question_file)

                    if question:
                        self.get_logger().info(f'Question: "{question}"')

                        # Start breathing light while searching and answering
                        stop_breathing = threading.Event()
                        breath_thread = threading.Thread(target=self.breath_light, args=(stop_breathing,))
                        breath_thread.start()

                        if 'weather' in question.lower() or 'temperature' in question.lower():
                            answer = self.get_weather(question)
                        else:
                            answer = self.search_and_answer(question)
                        self.get_logger().info(f'Answer: {answer}')
                        self.speak(answer)                    # blocking — waits for speech to finish

                        # Stop breathing and turn off after delay
                        stop_breathing.set()
                        breath_thread.join()
                        self.lights_off_delayed(3.0)

                    else:
                        self.speak("I didn't catch that")
                        self.lights_off_delayed(2.0)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceAssistant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()