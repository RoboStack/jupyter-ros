import sys
import signal
from math import pi
from threading import Thread
import rclpy
from rclpy.node import Node
import xml.dom.minidom
from sensor_msgs.msg import JointState

RANGE = 10000


class JointStatePublisher(Node):
    def get_param(self, name, value=None):
        private = "~%s" % name
        if self.has_parameter(private):
            return self.get_parameter(private)

        if self.has_parameter(name):
            return self.get_parameter(name)

        return value

    def init_collada(self, robot):
        robot = robot.getElementsByTagName('kinematics_model')[0].getElementsByTagName('technique_common')[0]
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue

            if child.localName == 'joint':
                name = child.getAttribute('name')

                if child.getElementsByTagName('revolute'):
                    joint = child.getElementsByTagName('revolute')[0]
                else:
                    self.get_logger().warn("Unknown joint type %s" % child)
                    continue

                if joint:
                    limit = joint.getElementsByTagName('limits')[0]
                    minval = float(limit.getElementsByTagName('min')[0].childNodes[0].nodeValue)
                    maxval = float(limit.getElementsByTagName('max')[0].childNodes[0].nodeValue)

                if minval == maxval:  # this is fixed joint
                    continue

                self.joint_list.append(name)
                joint = {
                    'min': minval*pi/180.0,
                    'max': maxval*pi/180.0,
                    'zero': 0,
                    'position': 0,
                    'velocity': 0,
                    'effort':0,
                    }
                self.free_joints[name] = joint

    def init_urdf(self, robot):
        robot = robot.getElementsByTagName('robot')[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue

            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed' or jtype == 'floating':
                    continue

                name = child.getAttribute('name')
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi

                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        self.get_logger().warn(f"{name} is not fixed, nor continuous, but limits are not specified!")
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if self.use_small and len(safety_tags) == 1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))

                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if self.use_mimic and len(mimic_tags) == 1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))

                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval) / 2
                else:
                    zeroval = 0

                joint = {'min': minval, 'max': maxval, 'zero': zeroval}
                if self.pub_def_positions:
                    joint['position'] = zeroval

                if self.pub_def_vels:
                    joint['velocity'] = 0.0

                if self.pub_def_efforts:
                    joint['effort'] = 0.0

                if jtype == 'continuous':
                    joint['continuous'] = True

                self.free_joints[name] = joint

    def __init__(self):
        super().__init__('joint_state_publisher')
        description = self.get_param('robot_description')

        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = self.get_param("dependent_joints", {})
        self.use_mimic = self.get_param('use_mimic_tags', True)
        self.use_small = self.get_param('use_smallest_joint_limits', True)

        self.zeros = self.get_param("zeros")

        self.pub_def_positions = self.get_param("publish_default_positions", True)
        self.pub_def_vels = self.get_param("publish_default_velocities", False)
        self.pub_def_efforts = self.get_param("publish_default_efforts", False)

        # ros2 loop timer and params
        hz = self.get_param("rate", 10)  # 10hz
        self.delta = self.get_param("delta", 0.0)
        self.timer = self.create_timer(1/hz, self.loop)

        # TODO: `robot` returns as NoneType
        # robot = xml.dom.minidom.parseString(description)
        # if robot.getElementsByTagName('COLLADA'):
        #     self.init_collada(robot)
        # else:
        #     self.init_urdf(robot)

        use_gui = self.get_param("use_gui", False)

        if use_gui:
            num_rows = self.get_param("num_rows", 0)
            self.app = QApplication(sys.argv)
            self.gui = JointStatePublisherGui("Joint State Publisher", self, num_rows)
            self.gui.show()
        else:
            self.gui = None

        source_list = self.get_param("source_list", [])
        self.sources = []
        for source in source_list:
            self.sources.append(self.create_subscription(JointState, source, self.source_cb))

        self.pub = self.create_publisher(JointState, 'joint_states', 10)

    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            if name not in self.free_joints:
                continue

            if msg.position:
                position = msg.position[i]
            else:
                position = None

            if msg.velocity:
                velocity = msg.velocity[i]
            else:
                velocity = None

            if msg.effort:
                effort = msg.effort[i]
            else:
                effort = None

            joint = self.free_joints[name]
            if position is not None:
                joint['position'] = position

            if velocity is not None:
                joint['velocity'] = velocity

            if effort is not None:
                joint['effort'] = effort

        if self.gui is not None:
            # signal instead of directly calling the update_sliders method, to switch to the QThread
            self.gui.sliderUpdateTrigger.emit()

    def loop(self):

        # Publish Joint States
        msg = JointState()
        msg.header.stamp = self.get_clock().now()

        if self.delta > 0:
            self.update(self.delta)

        # Initialize msg.position, msg.velocity, and msg.effort.
        has_position = len(self.dependent_joints.items()) > 0
        has_velocity = False
        has_effort = False
        for name, joint in self.free_joints.items():
            if not has_position and 'position' in joint:
                has_position = True

            if not has_velocity and 'velocity' in joint:
                has_velocity = True

            if not has_effort and 'effort' in joint:
                has_effort = True

        num_joints = (len(self.free_joints.items()) +
                      len(self.dependent_joints.items()))
        if has_position:
            msg.position = num_joints * [0.0]

        if has_velocity:
            msg.velocity = num_joints * [0.0]

        if has_effort:
            msg.effort = num_joints * [0.0]

        for i, name in enumerate(self.joint_list):
            msg.name.append(str(name))
            joint = None

            # Add Free Joint
            if name in self.free_joints:
                joint = self.free_joints[name]
                factor = 1
                offset = 0
            # Add Dependent Joint
            elif name in self.dependent_joints:
                param = self.dependent_joints[name]
                parent = param['parent']
                factor = param.get('factor', 1)
                offset = param.get('offset', 0)
                # Handle recursive mimic chain
                recursive_mimic_chain_joints = [name]
                while parent in self.dependent_joints:
                    if parent in recursive_mimic_chain_joints:
                        error_message = "Found an infinite recursive mimic chain"
                        self.get_logger().error("{}: [{}, {}]".format(error_message, ', '.join(recursive_mimic_chain_joints), parent))
                        sys.exit(-1)

                    recursive_mimic_chain_joints.append(parent)
                    param = self.dependent_joints[parent]
                    parent = param['parent']
                    offset += factor * param.get('offset', 0)
                    factor *= param.get('factor', 1)

                joint = self.free_joints[parent]

            if has_position and 'position' in joint:
                msg.position[i] = joint['position'] * factor + offset

            if has_velocity and 'velocity' in joint:
                msg.velocity[i] = joint['velocity'] * factor

            if has_effort and 'effort' in joint:
                msg.effort[i] = joint['effort']

        if msg.name or msg.position or msg.velocity or msg.effort:
            # Only publish non-empty messages
            self.pub.publish(msg)

    def update(self, delta):
        for name, joint in self.free_joints.items():
            forward = joint.get('forward', True)
            if forward:
                joint['position'] += delta
                if joint['position'] > joint['max']:
                    if joint.get('continuous', False):
                        joint['position'] = joint['min']
                    else:
                        joint['position'] = joint['max']
                        joint['forward'] = not forward

            else:
                joint['position'] -= delta
                if joint['position'] < joint['min']:
                    joint['position'] = joint['min']
                    joint['forward'] = not forward


if __name__ == '__main__':
    rclpy.init()
    JSP = JointStatePublisher()

    if JSP.gui is None:
        rclpy.spin(JSP)
    else:
        Thread(target=rclpy.spin(JSP)).start()
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        sys.exit(JSP.app.exec_())

    # Destroy the node explicitly
    JSP.destroy_node()
    rclpy.shutdown()
