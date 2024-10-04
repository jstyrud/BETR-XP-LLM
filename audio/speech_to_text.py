#!/usr/bin/env python3
#
# Copyright (c) 2024, ABB Schweiz AG
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import speech_recognition as sr


class SpeechRecognition:
    def __init__(self, mic_device=0, sample_rate=16000, noise_threshold=None):
        self._recognizer = sr.Recognizer()
        self._microphone = sr.Microphone(device_index=mic_device, sample_rate=sample_rate)

        if not noise_threshold:
            print("Calibrating microphone background noise...")
            with self._microphone as mic:
                self._recognizer.adjust_for_ambient_noise(mic)
            print(f"Done! Threshold at {self._recognizer.energy_threshold}")
        else:
            self._recognizer.energy_threshold = noise_threshold

    def listen(self):
        text = str()
        with self._microphone as mic:
            audio = self._recognizer.listen(mic)
            try:
                text = self._recognizer.recognize_google(audio)
            except sr.UnknownValueError:
                print("Google Speech Recognition could not understand audio")
            except sr.RequestError as e:
                print(f"Could not request results from Google Speech Recognition service; {e}")
        return text

    @staticmethod
    def print_devices():
        for index, name in enumerate(sr.Microphone.list_microphone_names()):
            print(f'Microphone with name "{name}" found for `Microphone(device_index={index})`')
