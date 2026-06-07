#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import speech_recognition as sr
from datetime import datetime
import os
import threading
import time
import math
from geometry_msgs.msg import Twist
from ddgs import DDGS
import re
from beast_msgs.srv import SetLEDBrightness
from beast_interaction import sound_localizer


class VoiceAssistant(Node):
    def __init__(self):
        super().__init__('voice_assistant')

        # LED service clients — one per IO channel
        self.light_client_io4 = self.create_client(SetLEDBrightness, 'ugv/set_headlights')
        self.light_client_io5 = self.create_client(SetLEDBrightness, 'ugv/set_headlights_io5')
        self._lights_timer = None

        self.get_logger().info('Voice Assistant ready!')
        self.get_logger().info('Say "Hey Beast" to activate...')

        self.listen_for_wake_word()

    # ---------- Light helpers ----------

    def _set_brightness(self, client, brightness: float):
        """Fire-and-forget brightness call on the given client."""
        req = SetLEDBrightness.Request()
        req.brightness = brightness
        client.call_async(req)

    def lights_on(self):
        """Turn both light channels on at full brightness."""
        for client in (self.light_client_io4, self.light_client_io5):
            if not client.service_is_ready():
                self.get_logger().warn('A light service is not available')
                return
        self._set_brightness(self.light_client_io4, 255.0)
        self._set_brightness(self.light_client_io5, 255.0)
        self.get_logger().info('Headlights ON')

    def lights_off_delayed(self, delay=3.0):
        """Turn both lights off after a delay."""
        if self._lights_timer is not None:
            self._lights_timer.cancel()
        self._lights_timer = threading.Timer(delay, self._lights_off_callback)
        self._lights_timer.start()

    def _lights_off_callback(self):
        self._set_brightness(self.light_client_io4, 0.0)
        self._set_brightness(self.light_client_io5, 0.0)
        self.get_logger().info('Headlights OFF')
        self._lights_timer = None

    def breath_light(self, stop_event: threading.Event):
        """
        Alternating breathing effect between IO4 and IO5.

        IO4 fades 0→255 while IO5 fades 255→0, then they swap.
        The two lights are always mirror images of each other,
        giving a smooth cross-fade / heartbeat look.

        Tune STEP and DELAY to taste:
          STEP  — bigger = choppier but fewer service calls
          DELAY — bigger = slower sweep
        """
        STEP  = 5     # brightness increment per tick
        DELAY = 0.02  # seconds between ticks

        # Both lights full-bright briefly before alternating starts
        self._set_brightness(self.light_client_io4, 255.0)
        self._set_brightness(self.light_client_io5, 255.0)
        time.sleep(0.5)

        while not stop_event.is_set():
            # Phase 1: IO4 ramps DOWN, IO5 ramps UP
            for brightness in range(255, -1, -STEP):
                if stop_event.is_set():
                    break
                self._set_brightness(self.light_client_io4, float(brightness))
                self._set_brightness(self.light_client_io5, float(255 - brightness))
                time.sleep(DELAY)

            # Phase 2: IO4 ramps UP, IO5 ramps DOWN
            for brightness in range(0, 256, STEP):
                if stop_event.is_set():
                    break
                self._set_brightness(self.light_client_io4, float(brightness))
                self._set_brightness(self.light_client_io5, float(255 - brightness))
                time.sleep(DELAY)

        # Stopped mid-cycle — zero both lights cleanly
        self._set_brightness(self.light_client_io4, 0.0)
        self._set_brightness(self.light_client_io5, 0.0)

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
            location = question.lower()
            for phrase in ['what is the current weather in', 'what is the current weather for',
                           'what is the weather report for', 'what is the weather in',
                           'what is the temperature in', 'weather in', 'temperature in',
                           'weather for', 'temperature for', 'what is the weather',
                           'what is the current']:
                location = location.replace(phrase, '').strip()

            url = f'http://wttr.in/{location.replace(" ", "+")}?format="%l:+%C+%t+humidity+%h"'
            result = subprocess.run(['curl', '-s', url], capture_output=True, text=True, timeout=30)
            weather = result.stdout.strip()
            self.get_logger().info(f'Raw weather: {weather}')

            weather = re.sub(r'[^\x00-\x7F]+', '', weather)
            weather = re.sub(r'\+', ' ', weather)
            weather = re.sub(r'\s+', ' ', weather)
            weather = weather.replace('F ', ' degrees Fahrenheit ')
            weather = weather.replace('%', ' percent')
            weather = weather.replace('"', '')
            weather = weather.strip()

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
                    self.lights_on()                          # Both lights full-bright on wake word
                    # Localize the speaker and rotate to face them
                    angle = sound_localizer.localize()
                    if angle is not None:
                        self.get_logger().info(f'Speaker at {angle:.1f}° — rotating to face')
                        self.rotate_to_angle(angle)
                    self.speak("Yes?")

                    self.get_logger().info('Listening for question...')
                    question_file = self.record_audio(duration=5)
                    question = self.transcribe_audio(question_file)

                    if question:
                        self.get_logger().info(f'Question: "{question}"')

                        # Start alternating breath while thinking / speaking
                        stop_breathing = threading.Event()
                        breath_thread = threading.Thread(
                            target=self.breath_light,
                            args=(stop_breathing,),
                            daemon=True
                        )
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

    def rotate_to_angle(self, angle_deg):
        """
        Publish a cmd_vel rotation to face the given angle.
        Positive angle = turn left, negative = turn right.

        Dead zones:
          - Near 0°   → already facing speaker, skip
          - Near 180° → ambiguous/weak signal (mics face rear, front is blind spot), skip
        """
        if not hasattr(self, '_cmd_vel_pub'):
            self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Skip rotation if already facing speaker or signal is ambiguous
        if abs(angle_deg) < 20.0:
            self.get_logger().info('Already facing speaker, no rotation needed')
            return
        if abs(abs(angle_deg) - 180.0) < 20.0:
            self.get_logger().info('Ambiguous angle (mic blind spot), no rotation')
            return

        angular_speed = 0.8   # rad/s
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad) / angular_speed

        twist = Twist()
        twist.angular.z = angular_speed if angle_rad > 0 else -angular_speed

        rate = 20   # Hz
        steps = int(duration * rate)

        for _ in range(steps):
            self._cmd_vel_pub.publish(twist)
            time.sleep(1.0 / rate)

        self._cmd_vel_pub.publish(Twist())


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