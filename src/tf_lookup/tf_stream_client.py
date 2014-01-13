import actionlib
import rospy
import tf_lookup.msg as TLM
import tf_lookup.tf_common as TLC
import tf.msg as TM

class TfStreamClient:
    def __init__(self):
        self.al_client = actionlib.SimpleActionClient("tf_stream", TLM.TfStreamAction)
        self.transforms = {}
        self.cb = None
        self.sub_id = None
        self.last_update = 0

    def add_transform(self, target, source, cb):
        target = TLC.sanitize_frame(target)
        source = TLC.sanitize_frame(source)
        key = TLC.key_from_transform(target, source)
        if key in self.transforms:
            if rospy.get_time() - self.last_update < 1.0:
                return
        self.transforms[key] = {'cb': cb, 'sub': TLM.Subscription(target, source)}
        self.last_update = rospy.get_time()
        self._update_transforms()

    def _update_transforms(self):
        goal = TLM.TfStreamGoal()
        if self.sub_id is not None:
            goal.update = True
            goal.subscription_id = self.sub_id
        goal.transforms = [x['sub'] for x in self.transforms.itervalues()]
        self.al_client.send_goal(goal, done_cb=self._al_cb)

    def _al_cb(self, status, result):
        if status != actionlib.GoalStatus.SUCCEEDED:
            return
        if self.sub_id is None:
            self.sub = rospy.Subscriber(result.topic, TM.tfMessage, self._main_cb)
            self.sub_id = result.subscription_id

    def _main_cb(self, data):
        for t in data.transforms:
            key = TLC.key_from_transform(t.header.frame_id, t.child_frame_id)
            if key not in self.transforms:
                rospy.logwarn("we have received an unsollicited transform: [{}]->[{}]".format(
                    t.header.frame_id, t.child_frame_id))
                continue
            self.transforms[key]['cb'](t)
