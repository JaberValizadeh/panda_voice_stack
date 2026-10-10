#!/usr/bin/env python3
"""Offline speech recognition node for the Panda block-stacking demo.

Captures microphone audio with arecord, recognizes speech with Vosk, and
publishes the recognized text on /voice_commands (std_msgs/String).

Same structure as tb3_voice_control/scripts/voice_recognizer.py; only the
grammar differs - here it covers manipulation phrases rather than driving
ones. Restricting the grammar matters even more for this demo, because the
block names are single letters, which free-form dictation gets wrong most
of the time.
"""

import audioop
import json
import os
import subprocess

import rospy
from std_msgs.msg import String

from vosk import Model, KaldiRecognizer, SetLogLevel

SAMPLE_RATE = 16000
CHUNK_SIZE = 4000  # bytes read from arecord per iteration

# Phrases the recognizer is allowed to produce. The homophones ("bee", "sea",
# "hey", ...) are deliberate: Vosk will emit them for the spoken letters, and
# voice_arm_control.py maps them back onto A/B/C.
GRAMMAR = [
    # picking
    "pick up block a", "pick up block b", "pick up block c",
    "pick up block hey", "pick up block bee", "pick up block sea",
    "pick block a", "pick block b", "pick block c",
    "grab block a", "grab block b", "grab block c",
    # stacking
    "stack block a on block b", "stack block a on block c",
    "stack block b on block a", "stack block b on block c",
    "stack block c on block a", "stack block c on block b",
    "stack it on block a", "stack it on block b", "stack it on block c",
    "stack on block a", "stack on block b", "stack on block c",
    "put it on block a", "put it on block b", "put it on block c",
    # placing back down
    "put it down", "put block down", "place it on the table",
    "put it on the table", "release", "drop it",
    # housekeeping
    "open the gripper", "open gripper", "close the gripper", "close gripper",
    "go home", "home", "ready",
    "reset the blocks", "reset blocks", "reset",
    "stop", "halt", "cancel",
    "what are you holding", "status",
    "[unk]",
]


def unmute_microphone():
    """Best-effort unmute: ALSA capture gets re-muted on some systems
    (e.g. restored by PulseAudio), which silently breaks recognition."""
    for cmd in (["amixer", "sset", "Capture", "100%", "cap"],
                ["pactl", "set-source-mute", "@DEFAULT_SOURCE@", "0"],
                ["pactl", "set-source-volume", "@DEFAULT_SOURCE@", "100%"]):
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            pass


def find_model(explicit):
    """Locate the Vosk model.

    The 68 MB model is not duplicated into this package - by default we reuse
    the copy that ships with tb3_voice_control in ~/catkin_ws. A models/
    directory inside this package wins if you do drop one in.
    """
    candidates = []
    if explicit:
        candidates.append(explicit)
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates.append(os.path.join(pkg_root, "models",
                                   "vosk-model-small-en-us-0.15"))
    candidates.append(os.path.expanduser(
        "~/catkin_ws/src/tb3_voice_control/models/vosk-model-small-en-us-0.15"))
    for path in candidates:
        if os.path.isdir(path):
            return path
    return None


def main():
    rospy.init_node("voice_recognizer")
    unmute_microphone()

    model_path = find_model(rospy.get_param("~model_path", ""))
    device = rospy.get_param("~device", "default")  # ALSA capture device
    use_grammar = rospy.get_param("~use_grammar", True)
    # Software amplification for weak microphones (1.0 = no gain).
    gain = rospy.get_param("~gain", 2.0)

    if model_path is None:
        rospy.logfatal("Vosk model not found. Set ~model_path, or install the "
                       "model under this package's models/ directory.")
        return

    SetLogLevel(-1)
    rospy.loginfo("Loading Vosk model from %s ...", model_path)
    model = Model(model_path)
    if use_grammar:
        recognizer = KaldiRecognizer(model, SAMPLE_RATE, json.dumps(GRAMMAR))
    else:
        recognizer = KaldiRecognizer(model, SAMPLE_RATE)

    pub = rospy.Publisher("voice_commands", String, queue_size=10)

    arecord_cmd = [
        "arecord", "-D", device, "-q",
        "-f", "S16_LE", "-r", str(SAMPLE_RATE), "-c", "1", "-t", "raw",
    ]
    rospy.loginfo("Starting microphone capture: %s", " ".join(arecord_cmd))
    proc = subprocess.Popen(arecord_cmd, stdout=subprocess.PIPE)

    rospy.loginfo("Listening... say e.g. 'pick up block A', "
                  "'stack it on block B', 'go home'")
    try:
        while not rospy.is_shutdown():
            data = proc.stdout.read(CHUNK_SIZE)
            if not data:
                rospy.logerr("Microphone stream ended unexpectedly")
                break
            if gain != 1.0:
                data = audioop.mul(data, 2, gain)
            if recognizer.AcceptWaveform(data):
                text = json.loads(recognizer.Result()).get("text", "")
                text = text.replace("[unk]", "").strip()
                if text:
                    rospy.loginfo("Heard: '%s'", text)
                    pub.publish(text)
    finally:
        proc.terminate()
        proc.wait()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
