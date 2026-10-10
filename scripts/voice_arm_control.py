#!/usr/bin/env python3
"""Turn spoken commands into Panda pick-and-place motions.

Subscribes to /voice_commands (std_msgs/String), parses each utterance into a
manipulation command, and executes it with MoveIt plus the franka_gripper
actions - the same motion recipe as panda_block_stack/scripts/stack_blocks.py,
but driven one command at a time instead of as a fixed script.

    "pick up block A"          -> grasp A and lift it
    "stack it on block B"      -> place what is held on top of B
    "stack block A on block B" -> both steps in one go
    "put it down"              -> place what is held back on the table
    "open/close the gripper"   -> direct gripper control
    "go home"                  -> back to the 'ready' pose
    "reset the blocks"         -> teleport the blocks back in Gazebo
    "stop"                     -> abort the motion in progress

Speech recognition is asynchronous but arm motion is not, so commands are
queued and run by a single worker thread; 'stop' is handled inline in the
subscriber callback so it can interrupt a move that is already executing.

Block poses are tracked from Gazebo's /gazebo/model_states rather than
hard-coded, so the node stays correct after a reset, after a manual drag in
the Gazebo GUI, or after several stacks in a row.
"""

import copy
import queue
import re
import sys
import threading

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler

import actionlib
from franka_gripper.msg import (MoveAction, MoveGoal, GraspAction, GraspGoal,
                                StopAction, StopGoal)

# --- Scene constants (must match panda_block_stack/worlds/blocks.world) ----
BLOCK = 0.045                 # cube edge [m]
TABLE_TOP = 0.40              # table surface height [m]
BLOCK_Z = TABLE_TOP + BLOCK / 2.0

TABLE_SIZE = (0.60, 0.90, 0.40)
TABLE_CENTER = (0.55, 0.0, 0.20)

# Where each block starts, and where "put it down" returns it to.
HOME_XY = {"a": (0.45, -0.20), "b": (0.45, 0.20), "c": (0.62, 0.00)}

APPROACH = 0.12               # how far above a block to hover before descending
# How close to the goal still counts as "arrived" when the controller reports
# a tolerance violation. Well under half a block, so a genuine miss still fails.
REACHED_TOLERANCE = 0.01      # [m]
GRIPPER_OPEN = 0.075          # finger separation when open [m]
GRIPPER_WIDTH = BLOCK         # target width when grasping [m]
GRASP_FORCE = 25.0            # [N]

# Panda's tool frame (panda_link8) points +Z along the flange axis. Pointing
# the hand straight down at the table means rotating pi about X; the extra
# -pi/4 about Z aligns the fingers with the world Y axis, matching how the
# hand is mounted 45 degrees off link8.
DOWNWARD_RPY = (3.14159265, 0.0, -0.78539816)

# Vosk hears the spoken letters as ordinary words; map them back. Anything not
# in here is simply not a block name.
LETTER_WORDS = {
    "a": "a", "hey": "a", "eight": "a", "ay": "a", "alpha": "a",
    "b": "b", "bee": "b", "be": "b", "p": "b", "bravo": "b",
    "c": "c", "see": "c", "sea": "c", "si": "c", "charlie": "c",
}


class VoiceArmControl(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("voice_arm_control", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")

        self.arm.set_end_effector_link("panda_hand_tcp")
        self.arm.set_planning_time(10.0)
        self.arm.set_num_planning_attempts(10)
        # The Panda's datasheet limits are aggressive for a stacking demo;
        # slowing down keeps the block from being shaken loose in Gazebo.
        self.arm.set_max_velocity_scaling_factor(0.25)
        self.arm.set_max_acceleration_scaling_factor(0.25)
        self.arm.set_goal_position_tolerance(0.005)
        self.arm.set_goal_orientation_tolerance(0.02)
        self.planning_frame = self.arm.get_planning_frame()

        # franka_gazebo exposes the hand as actions rather than a MoveIt group.
        self.gripper_move = actionlib.SimpleActionClient(
            "franka_gripper/move", MoveAction)
        self.gripper_grasp = actionlib.SimpleActionClient(
            "franka_gripper/grasp", GraspAction)
        self.gripper_stop = actionlib.SimpleActionClient(
            "franka_gripper/stop", StopAction)
        rospy.loginfo("Waiting for franka_gripper action servers...")
        self.gripper_move.wait_for_server()
        self.gripper_grasp.wait_for_server()

        # Ground-truth block poses, kept fresh from Gazebo.
        self.block_poses = {k: (x, y, BLOCK_Z) for k, (x, y) in HOME_XY.items()}
        self.poses_lock = threading.Lock()
        self.held = None            # letter of the block currently grasped
        self.abort = False          # set by 'stop', cleared before each command

        self.commands = queue.Queue()
        self.feedback = rospy.Publisher("~feedback", String, queue_size=10)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_models,
                         queue_size=1)
        rospy.Subscriber("voice_commands", String, self.on_voice, queue_size=10)

        rospy.loginfo("Planning frame: %s", self.planning_frame)
        rospy.loginfo("Ready. Say 'pick up block A', 'stack it on block B', "
                      "'put it down', 'go home', 'reset the blocks', 'stop'.")

    # -- world state -------------------------------------------------------
    def on_models(self, msg):
        with self.poses_lock:
            for name, pose in zip(msg.name, msg.pose):
                letter = name.lower().replace("block_", "")
                if letter in self.block_poses and letter != self.held:
                    self.block_poses[letter] = (pose.position.x,
                                                pose.position.y,
                                                pose.position.z)

    def block_pose(self, letter):
        with self.poses_lock:
            return self.block_poses[letter]

    def say(self, text, warn=False):
        (rospy.logwarn if warn else rospy.loginfo)("%s", text)
        self.feedback.publish(text)

    # -- planning scene ----------------------------------------------------
    def _box_pose(self, x, y, z):
        p = PoseStamped()
        p.header.frame_id = self.planning_frame
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation.w = 1.0
        return p

    def sync_planning_scene(self):
        """Mirror the current Gazebo world into MoveIt so plans avoid it.

        Re-run before every command: unlike the scripted demo, the blocks may
        have been moved by an earlier spoken command since the last sync.
        """
        self.scene.add_box("table", self._box_pose(*TABLE_CENTER),
                           size=TABLE_SIZE)
        cube = (BLOCK, BLOCK, BLOCK)
        for letter in HOME_XY:
            name = "block_%s" % letter.upper()
            if letter == self.held:
                continue        # attached to the hand, not a world object
            x, y, z = self.block_pose(letter)
            self.scene.add_box(name, self._box_pose(x, y, z), size=cube)
        rospy.sleep(0.5)

    # -- gripper -----------------------------------------------------------
    def open_gripper(self):
        goal = MoveGoal(width=GRIPPER_OPEN, speed=0.1)
        self.gripper_move.send_goal_and_wait(goal, rospy.Duration(5.0))
        rospy.loginfo("Gripper opened")

    def close_gripper(self):
        goal = GraspGoal()
        goal.width = GRIPPER_WIDTH
        goal.speed = 0.05
        goal.force = GRASP_FORCE
        # Generous tolerances: the block is squeezed slightly, and the grasp
        # action reports failure if the final width falls outside this band.
        goal.epsilon.inner = 0.02
        goal.epsilon.outer = 0.02
        self.gripper_grasp.send_goal_and_wait(goal, rospy.Duration(8.0))
        rospy.loginfo("Gripper closed")
        # Give gazebo_grasp_fix its few update cycles to latch the block.
        rospy.sleep(1.5)

    # -- motion ------------------------------------------------------------
    def check_abort(self):
        if self.abort:
            raise RuntimeError("aborted by 'stop'")

    def move_to(self, x, y, z, label):
        self.check_abort()
        q = quaternion_from_euler(*DOWNWARD_RPY)
        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.arm.set_pose_target(pose)
        rospy.loginfo("Moving to %s (%.3f, %.3f, %.3f)", label, x, y, z)
        ok = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        self.check_abort()
        if not ok:
            # go() also returns False when the trajectory finished but the
            # controller declared GOAL_TOLERANCE_VIOLATED - routinely by a
            # fraction of a millimetre under franka_gazebo. Treating that as a
            # failure aborts commands that in fact arrived, so ask where the
            # arm actually is before giving up.
            error = self.position_error(x, y, z)
            if error <= REACHED_TOLERANCE:
                rospy.logwarn("%s: planner reported failure but the arm is "
                              "%.1f mm from the goal - continuing",
                              label, error * 1000.0)
                return True
            raise RuntimeError("failed to reach %s (off by %.0f mm)"
                               % (label, error * 1000.0))
        return ok

    def position_error(self, x, y, z):
        """Distance from the end effector to (x, y, z), in metres."""
        p = self.arm.get_current_pose().pose.position
        return ((p.x - x) ** 2 + (p.y - y) ** 2 + (p.z - z) ** 2) ** 0.5

    def move_straight(self, dx, dy, dz, label, avoid_collisions=True):
        """Cartesian straight-line motion - used for approach and retreat so
        the hand never swings sideways while it is near a block.

        Approach and place segments deliberately drive the payload into
        contact with the scene, so they pass avoid_collisions=False; otherwise
        the planner refuses to generate the last few centimetres.

        Note the signature: MoveIt 1.1's compute_cartesian_path takes
        (waypoints, eef_step, avoid_collisions) - the old jump_threshold
        positional argument is gone.
        """
        self.check_abort()
        start = self.arm.get_current_pose().pose
        target = copy.deepcopy(start)
        target.position.x += dx
        target.position.y += dy
        target.position.z += dz

        plan, fraction = self.arm.compute_cartesian_path(
            [target], 0.005, avoid_collisions)
        if fraction < 0.9:
            rospy.logwarn("%s: only %.0f%% of the straight path was planned, "
                          "falling back to a joint-space move",
                          label, fraction * 100)
            return self.move_to(target.position.x, target.position.y,
                                target.position.z, label)
        rospy.loginfo("Cartesian move: %s", label)
        self.arm.execute(plan, wait=True)
        self.arm.stop()
        self.check_abort()
        return True

    def go_home(self):
        self.check_abort()
        self.arm.set_named_target("ready")
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        rospy.loginfo("At home ('ready') pose")

    # -- the manipulation primitives ---------------------------------------
    def do_pick(self, letter):
        if self.held is not None:
            self.say("I am already holding block %s - stack it or put it down "
                     "first." % self.held.upper(), warn=True)
            return
        name = "block_%s" % letter.upper()
        x, y, z = self.block_pose(letter)
        self.say("Picking up block %s" % letter.upper())

        self.sync_planning_scene()
        self.open_gripper()
        # Hover first, then drop block X from the planning scene: the gripper
        # is about to occupy the same space on purpose.
        self.move_to(x, y, z + APPROACH, "above block %s" % letter.upper())
        self.scene.remove_world_object(name)
        rospy.sleep(0.5)

        self.move_straight(0, 0, -APPROACH,
                           "descend onto block %s" % letter.upper(),
                           avoid_collisions=False)
        self.close_gripper()
        self.held = letter

        # Tell MoveIt the block travels with the hand, so it is accounted for
        # in collision checking on the way to wherever it is going.
        self.scene.add_box(name, self._box_pose(x, y, z),
                           size=(BLOCK, BLOCK, BLOCK))
        rospy.sleep(0.5)
        self.arm.attach_object(
            name, "panda_hand",
            touch_links=self.robot.get_link_names(group="panda_hand"))
        rospy.sleep(0.5)

        self.move_straight(0, 0, APPROACH, "lift block %s" % letter.upper())
        self.say("Holding block %s" % letter.upper())

    def do_place(self, x, y, z, description):
        """Release the held block so its centre lands at (x, y, z)."""
        letter = self.held
        name = "block_%s" % letter.upper()
        self.say("Placing block %s %s" % (letter.upper(), description))

        self.move_to(x, y, z + APPROACH, "above the drop point")
        # Release 3 mm high so the block drops the last bit instead of being
        # jammed into whatever is underneath it.
        self.move_straight(0, 0, -(APPROACH - 0.003), "descend to the drop point",
                           avoid_collisions=False)

        self.open_gripper()
        self.arm.detach_object(name)
        rospy.sleep(0.5)
        self.held = None
        with self.poses_lock:
            self.block_poses[letter] = (x, y, z)
        self.scene.remove_world_object(name)
        self.scene.add_box(name, self._box_pose(x, y, z),
                           size=(BLOCK, BLOCK, BLOCK))

        self.move_straight(0, 0, APPROACH, "retreat")
        self.say("Block %s is %s" % (letter.upper(), description))

    def do_stack(self, target):
        if self.held is None:
            self.say("I am not holding anything - say 'pick up block A' first.",
                     warn=True)
            return
        if target == self.held:
            self.say("I cannot stack block %s on itself." % target.upper(),
                     warn=True)
            return
        tx, ty, tz = self.block_pose(target)
        # Target block's top face, plus half of the carried block.
        place_z = tz + BLOCK / 2.0 + BLOCK / 2.0
        self.sync_planning_scene()
        self.do_place(tx, ty, place_z, "on top of block %s" % target.upper())

    def do_put_down(self):
        if self.held is None:
            self.say("I am not holding anything.", warn=True)
            return
        letter = self.held
        x, y = HOME_XY[letter]
        occupied = [k for k in HOME_XY
                    if k != letter and self._near(self.block_pose(k), (x, y))]
        if occupied:
            # Its home slot is taken (something got stacked there); drop it in
            # front of the table centre instead.
            x, y = TABLE_CENTER[0] - 0.10, 0.0
        self.sync_planning_scene()
        self.do_place(x, y, BLOCK_Z, "on the table")

    @staticmethod
    def _near(pose, xy, tol=0.05):
        return abs(pose[0] - xy[0]) < tol and abs(pose[1] - xy[1]) < tol

    def do_reset(self):
        if self.held is not None:
            self.say("Put the block down before resetting.", warn=True)
            return
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
            set_state = rospy.ServiceProxy("/gazebo/set_model_state",
                                           SetModelState)
        except rospy.ROSException:
            self.say("Gazebo's set_model_state service is not available.",
                     warn=True)
            return
        for letter, (x, y) in HOME_XY.items():
            state = ModelState()
            state.model_name = "block_%s" % letter.upper()
            state.reference_frame = "world"
            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = BLOCK_Z
            state.pose.orientation.w = 1.0
            set_state(state)
        rospy.sleep(1.0)
        self.sync_planning_scene()
        self.say("Blocks are back at their starting poses.")

    def do_status(self):
        if self.held is None:
            self.say("I am not holding anything.")
        else:
            self.say("I am holding block %s." % self.held.upper())

    # -- parsing -----------------------------------------------------------
    @staticmethod
    def letters_in(text):
        """Block letters mentioned, in the order they were spoken.

        Only words that directly follow 'block' count, so the 'a' in
        'a block' or a stray 'be' cannot be mistaken for a block name.
        """
        words = text.split()
        found = []
        for i, word in enumerate(words[:-1]):
            if word == "block" and words[i + 1] in LETTER_WORDS:
                found.append(LETTER_WORDS[words[i + 1]])
        return found

    def parse(self, text):
        """Map an utterance onto (handler, args), or None if unrecognized."""
        cmd = re.sub(r"[^a-z ]", "", text.lower()).strip()
        words = set(cmd.split())
        letters = self.letters_in(cmd)

        # Order matters: 'stop' first so it is never shadowed, and 'stack'
        # before 'pick' so "pick up A and stack it" would still stack.
        if words & {"stop", "halt", "cancel"}:
            return (self.do_stop_sync, ())
        if words & {"reset"}:
            return (self.do_reset, ())
        if words & {"status", "holding"}:
            return (self.do_status, ())
        if words & {"home", "ready"}:
            return (self.go_home, ())
        if "gripper" in words or "hand" in words:
            if "open" in words:
                return (self.open_gripper, ())
            if "close" in words:
                return (self.close_gripper, ())
        if "stack" in words:
            if len(letters) >= 2:
                return (self.do_pick_and_stack, (letters[0], letters[1]))
            if len(letters) == 1:
                return (self.do_stack, (letters[0],))
            return None
        if words & {"pick", "grab", "lift"}:
            if letters:
                return (self.do_pick, (letters[0],))
            return None
        # "put it on block B" is a stack; "put it down" / "on the table" is not.
        if words & {"put", "place"}:
            if letters and "down" not in words and "table" not in words:
                return (self.do_stack, (letters[0],))
            return (self.do_put_down, ())
        if words & {"release", "drop"}:
            return (self.do_put_down, ())
        return None

    def do_pick_and_stack(self, source, target):
        self.do_pick(source)
        if self.held == source:
            self.do_stack(target)

    def do_stop_sync(self):
        pass  # 'stop' is fully handled in the callback; nothing left to do

    # -- plumbing ----------------------------------------------------------
    def on_voice(self, msg):
        text = msg.data.strip()
        if not text:
            return
        parsed = self.parse(text)
        if parsed is None:
            self.say("I did not understand '%s'" % text, warn=True)
            return

        handler, args = parsed
        if handler is self.do_stop_sync:
            # Interrupt whatever is running and throw away anything queued.
            self.abort = True
            self.arm.stop()
            if self.gripper_stop.wait_for_server(rospy.Duration(0.5)):
                self.gripper_stop.send_goal(StopGoal())
            while not self.commands.empty():
                try:
                    self.commands.get_nowait()
                except queue.Empty:
                    break
            self.say("Stopping.")
            return

        self.commands.put((text, handler, args))

    def spin(self):
        while not rospy.is_shutdown():
            try:
                text, handler, args = self.commands.get(timeout=0.2)
            except queue.Empty:
                continue
            self.abort = False
            rospy.loginfo("Executing: '%s'", text)
            try:
                handler(*args)
            except Exception as exc:      # keep listening after a failed move
                self.say("Command '%s' failed: %s" % (text, exc), warn=True)
                self.arm.stop()
                self.arm.clear_pose_targets()


def main():
    node = VoiceArmControl()
    try:
        node.spin()
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
