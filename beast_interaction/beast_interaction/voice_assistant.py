#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import speech_recognition as sr
from datetime import datetime
import os
from ddgs import DDGS
import re

class VoiceAssistant(Node):
    def __init__(self):
        super().__init__('voice_assistant')
        
        self.get_logger().info('Voice Assistant ready!')
        self.get_logger().info('Say "Hey Beast" to activate...')
    
        self.listen_for_wake_word() 
    
    def speak(self, text):
        """Use espeak to make the robot speak"""
        self.get_logger().info(f'Speaking: {text}')
        subprocess.Popen(['espeak', text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    def record_audio(self, duration=3):
        """Record audio and return the filename"""
        filename_48k = f'/tmp/voice_48k_{datetime.now().strftime("%Y%m%d_%H%M%S")}.wav'
        filename_16k = f'/tmp/voice_{datetime.now().strftime("%Y%m%d_%H%M%S")}.wav'
        
        # Record at 48kHz (what camera supports)
        cmd = ['arecord', '-D', 'hw:0,0', '-f', 'S16_LE', '-c', '1', '-r', '48000', '-d', str(duration), filename_48k]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        # Resample to 16kHz for Vosk
        sox_cmd = ['sox', filename_48k, '-r', '16000', filename_16k]
        subprocess.run(sox_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        # Clean up 48kHz file
        os.remove(filename_48k)
        
        return filename_16k
    
    def transcribe_audio(self, filename):
        """Convert audio file to text using Google Speech Recognition"""
        recognizer = sr.Recognizer()
        
        try:
            with sr.AudioFile(filename) as source:
                audio = recognizer.record(source)
            
            # Use Google's free API
            text = recognizer.recognize_google(audio)
            return text
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
        """Search DuckDuckGo and return a concise answer"""
        try:
            self.get_logger().info(f'Searching for: {question}')
            
            with DDGS() as ddgs:
                results = list(ddgs.text(question, max_results=3))
            
            if results:
                # Get the first result's snippet as the answer
                answer = results[0].get('body', 'I could not find an answer')
                
                # Remove citation numbers like [1], [5][6], etc.
                answer = re.sub(r'\[\d+\]', '', answer)
                
                # Limit to first 2 sentences for conciseness
                sentences = answer.split('. ')
                short_answer = '. '.join(sentences[:2])
                return short_answer.strip()
            else:
                return "I could not find an answer to that question"
        
        except Exception as e:
            self.get_logger().error(f'Search error: {e}')
            return "Sorry, I had trouble searching for that"
    
    def listen_for_wake_word(self):
        """Simple wake word detection loop"""
        while rclpy.ok():
            self.get_logger().info('Listening for wake word...')
            
            # Record 3 seconds
            audio_file = self.record_audio(duration=3)
            text = self.transcribe_audio(audio_file)
            
            if text:
                self.get_logger().info(f'Heard: "{text}"')
                
                # Check for wake word (simple string matching)
                text_lower = text.lower()
                if ('beast' in text_lower or 'based' in text_lower or 
                    'hey beast' in text_lower or 'hey based' in text_lower or
                    'hey b' in text_lower):
                    self.get_logger().info('Wake word detected!')
                    self.speak("Yes?")
                    
                    # Listen for question
                    self.get_logger().info('Listening for question...')
                    question_file = self.record_audio(duration=5)
                    question = self.transcribe_audio(question_file)
                    
                    if question:
                        self.get_logger().info(f'Question: "{question}"')
                        
                        # Search for the answer
                        answer = self.search_and_answer(question)
                        self.get_logger().info(f'Answer: {answer}')
                        
                        # Speak the answer
                        self.speak(answer)
                    else:
                        self.speak("I didn't catch that")


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