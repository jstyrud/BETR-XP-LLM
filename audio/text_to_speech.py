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

from gtts import gTTS
from io import BytesIO
from pygame import mixer, time


class TTS:
    def __init__(self):
        mixer.init()

    def speak(self, text: str):
        mp3_fp = BytesIO()
        tts = gTTS(text, lang="en")
        tts.write_to_fp(mp3_fp)
        mp3_fp.seek(0)
        mixer.music.load(mp3_fp, "mp3")
        mixer.music.play()
        while mixer.music.get_busy():
            time.wait(100)  # ms

    def save_mp3(self, text, filename):

        tts = gTTS(text, lang="en")
        tts.save(filename)

