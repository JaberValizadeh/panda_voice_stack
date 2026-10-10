#!/usr/bin/env python3
"""Type commands instead of speaking them.

Publishes on the same /voice_commands topic the recognizer uses, so the whole
pipeline downstream of speech recognition can be exercised without a
microphone - useful over SSH, and the quickest way to tell a parsing bug apart
from a recognition one.

    rosrun panda_voice_stack say_command.py                 # interactive
    rosrun panda_voice_stack say_command.py pick up block a # one-shot
"""

import sys

import rospy
from std_msgs.msg import String

EXAMPLES = """Examples:
  pick up block a
  stack it on block b
  stack block c on block a
  put it down
  open the gripper / close the gripper
  go home
  reset the blocks
  stop
Type 'quit' to exit."""


def main():
    rospy.init_node("say_command", anonymous=True)
    pub = rospy.Publisher("voice_commands", String, queue_size=10, latch=True)

    if len(sys.argv) > 1:
        # Strip the __name:= / __log:= arguments roslaunch appends.
        words = [a for a in sys.argv[1:] if not a.startswith("__")]
        text = " ".join(words)
        rospy.sleep(0.5)          # let the connection establish before sending
        pub.publish(text)
        rospy.loginfo("Sent: '%s'", text)
        rospy.sleep(0.5)
        return

    print(EXAMPLES)
    while not rospy.is_shutdown():
        try:
            text = input("command> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not text:
            continue
        if text.lower() in ("quit", "exit"):
            break
        pub.publish(text)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
