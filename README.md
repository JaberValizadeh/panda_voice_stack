# panda_voice_stack

Talk to a Franka Emika **Panda** arm and it picks up and stacks blocks.

Spoken commands are recognized **offline** with Vosk (same recognizer pattern as
`tb3_voice_control` in `~/catkin_ws`), parsed into manipulation commands, and
executed with MoveIt plus the `franka_gripper` actions. The Gazebo scene - table,
blocks A/B/C, the Panda with `gazebo_grasp_fix` - is reused as-is from
[`panda_block_stack`](../panda_block_stack), so there is exactly one copy of the
world, the URDF wrapper and the grasp-plugin setup.

Where `panda_block_stack` runs one fixed script (A onto B, every time), this
package lets you drive the arm one command at a time: any block onto any other
block, in any order, plus put-down, home, gripper and reset.

## Running it

Load the Panda environment first, in **every** terminal - `~/.bashrc` defines a
`panda` function for exactly this:

```bash
panda
```

source ~/moveit_ws/setup_env.bash


(It wraps `source ~/moveit_ws/setup_env.bash`. It is not
automatic on purpose: this workspace displaces `~/catkin_ws` on
`ROS_PACKAGE_PATH`, so new terminals stay on TurtleBot3 until you ask.)

Then, all in one:

```bash
roslaunch panda_voice_stack voice_stack.launch
```

Wait for `You can start planning now!` and then `Ready. Say 'pick up block A' ...`,
and start talking.

| arg | default | meaning |
|---|---|---|
| `headless` | `false` | run Gazebo without its GUI |
| `rviz` | `true` | show the MoveIt motion-planning display |
| `keyboard` | `false` | `true` = no microphone, type commands instead |
| `device` | `default` | ALSA capture device, e.g. `hw:0,0` |
| `gain` | `2.0` | software mic amplification; raise it if your mic is quiet |
| `model_path` | *(auto)* | Vosk model dir; see [The Vosk model](#the-vosk-model) |

### Against a scene that is already up

If you already have `panda_block_stack environment.launch` running, add just the
voice pipeline:

```bash
roslaunch panda_voice_stack voice_control.launch
```

### No microphone

Over SSH, or to tell a parsing bug apart from a recognition one, type the
commands instead - they go onto the same topic:

```bash
roslaunch panda_voice_stack voice_stack.launch keyboard:=true
# then, in another terminal:
rosrun panda_voice_stack say_command.py            # interactive prompt
rosrun panda_voice_stack say_command.py pick up block a   # one-shot
```

## What you can say

| say | what happens |
|---|---|
| "pick up block A" / "grab block C" | hover, descend, grasp, lift |
| "stack it on block B" | place what is held on top of B |
| "stack block A on block B" | pick **and** stack, in one command |
| "put it down" / "place it on the table" | put the held block back on the table |
| "open the gripper" / "close the gripper" | direct gripper control |
| "go home" | back to the `ready` pose |
| "reset the blocks" | teleport all three blocks back to their start poses |
| "stop" | abort the motion in progress and drop anything queued |
| "what are you holding" | report the held block |

Any block can go on any other, and stacks compose - "pick up block A", "stack it
on block B", "pick up block C", "stack it on block A" builds a three-high tower,
because block poses are read back from Gazebo rather than assumed.

Feedback is logged and also published on `/voice_arm_control/feedback`
(`std_msgs/String`), so a TTS node or a GUI can subscribe to it.

## How it fits together

```
microphone -> voice_recognizer.py --/voice_commands--> voice_arm_control.py -> MoveIt -> Gazebo
              (Vosk, offline)        (std_msgs/String)  (parse + execute)
                    ^
       say_command.py publishes here too (keyboard fallback)
```

| path | what it is |
|---|---|
| `scripts/voice_recognizer.py` | mic capture + Vosk, publishes recognized text |
| `scripts/voice_arm_control.py` | parses commands and runs the arm |
| `scripts/say_command.py` | type commands instead of speaking them |
| `launch/voice_control.launch` | the voice pipeline alone |
| `launch/voice_stack.launch` | all-in-one: `panda_block_stack` scene + voice |

## The Vosk model

The 68 MB `vosk-model-small-en-us-0.15` is **not** duplicated into this package.
`voice_recognizer.py` looks for it in this order:

1. the `~model_path` parameter, if set;
2. `panda_voice_stack/models/vosk-model-small-en-us-0.15`;
3. `~/catkin_ws/src/tb3_voice_control/models/vosk-model-small-en-us-0.15` ← used here.

So this package depends on `~/catkin_ws` being present for speech. Nothing else
does: with `keyboard:=true` the recognizer never starts and neither Vosk nor the
model is needed.

## Building

`~/moveit_ws` is a **catkin_tools** workspace:

```bash
cd ~/moveit_ws
catkin build
```

`--no-deps` is no longer required: the MoveIt source checkout moved to
`~/ridgeback_ws` when the workspace was split, so a plain `catkin build` here
only builds the Panda demos and `gazebo-pkgs`. It remains harmless, and MoveIt
still comes from the `~/arm_underlay` binaries.

## Design notes

Things that are easy to get wrong here, on top of the five gotchas already
documented in `panda_block_stack/README.md` (all of which still apply, since
that package supplies the scene):

1. **Recognition, then execution, are on different clocks.** Speech arrives in
   callbacks; arm motion takes ~15 s. Commands are therefore queued and run by
   one worker thread - a subscriber callback must never block on `arm.go()`, or
   the next utterance is dropped. `stop` is the exception: it is handled inline
   in the callback, since it has to interrupt a move already in flight.

2. **Single letters are the hardest thing to recognize.** Vosk hears "bee",
   "sea", "hey" for B, C, A. Two defences: the grammar is restricted to the
   phrases in `GRAMMAR` (free-form dictation gets block names wrong most of the
   time), and `LETTER_WORDS` maps the homophones back.

3. **Only a word right after "block" counts as a block name.** Otherwise the
   "a" in "a block" and a stray "be" become block references, and the arm picks
   up the wrong cube.

4. **Block poses come from `/gazebo/model_states`, not from constants.** After
   the first stack the constants are stale; ground truth keeps multi-step towers
   and post-reset runs correct. The held block is skipped while it is attached -
   its Gazebo pose is mid-flight and would fight the planning scene.

5. **The planning scene is re-synced before every command,** for the same
   reason. `panda_block_stack` can build it once because its world only changes
   in ways its script already knows about.

6. **`arm.go()` returning False does not mean the arm missed.** Under
   franka_gazebo the position controller regularly ends a trajectory with
   `GOAL_TOLERANCE_VIOLATED` over an error of ~0.15 mrad. Raising on that
   aborted commands that had in fact arrived, so `move_to` now measures the
   real distance to the goal and only fails past `REACHED_TOLERANCE` (10 mm) -
   still well under half a block, so a genuine miss is not papered over.

7. **The voice node is deliberately not `required="true"`.** Bringing the scene
   up costs about a minute; you want to restart the controller after editing it
   without losing Gazebo, MoveIt and the controllers with it.

## Verified

Run headless end to end with typed commands (`keyboard:=true`):

```
stack block a on block b
stack block c on block a
```

Gazebo ground truth afterwards - a three-high tower, each block exactly one
45 mm edge above the last:

```
block_B: x=+0.4500 y=+0.2001 z=0.4225   <- base, unmoved
block_A: x=+0.4459 y=+0.2009 z=0.4675   <- on B
block_C: x=+0.4454 y=+0.2064 z=0.5125   <- on A
```

The second command is the one that proves the design: block A's pose at that
point exists nowhere in the source, only in Gazebo.

Non-motion paths checked in the same session - "what are you holding",
"put it down" with an empty gripper, "stop", and an unrecognized phrase, which
answers `I did not understand '...'` rather than moving.
