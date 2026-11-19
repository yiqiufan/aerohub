"""Workstation task proxy: sends context/stop requests; never samples motion references."""
import json
import time
import uuid
from types import SimpleNamespace
from std_msgs.msg import String

PREFIX = '/racer_short/adapter'


class RemoteAdapter:
    def __init__(self, mission, dry_run, prefix=None):
        prefix = PREFIX if prefix is None else prefix
        self.mission, self.dry_run = mission, dry_run
        self.snapshot = {}
        self.received = 0.0
        self.stop_id = None
        self.request = mission.create_publisher(String, prefix + '/command', 10)
        mission.create_subscription(String, prefix + '/state', self.state, 10)
        mission.create_subscription(String, prefix + '/event', self.event, 100)

    def state(self, msg):
        self.snapshot = json.loads(msg.data)
        self.received = time.monotonic()

    def event(self, msg):
        self.mission.trace('onboard_adapter_event', source=json.loads(msg.data))

    def fresh(self):
        return time.monotonic() - self.received < 1.25

    def ready(self):
        return self.fresh() and self.snapshot.get('ready', False) and self.snapshot.get('dry_run') == self.dry_run

    def sync_context(self):
        identity = uuid.uuid4().hex
        m = self.mission
        request = dict(command='context', identity=identity, phase=m.phase, anchor=m.anchor,
                       goal=m.goal, yaw=m.yaw)
        self.request.publish(String(data=json.dumps(request)))
        m.wait(lambda: self.fresh() and self.snapshot.get('context_id') == identity,
               2, 'onboard adapter did not acknowledge task context')

    def begin_stop(self, detail, operator):
        if self.stop_id is not None:
            return
        self.stop_id = uuid.uuid4().hex
        self.request.publish(String(data=json.dumps(dict(command='stop', identity=self.stop_id,
                                                         detail=detail, operator=operator))))

    @property
    def stopping(self):
        return (not self.fresh() or self.snapshot.get('stop_id') != self.stop_id
                or self.snapshot.get('stopping', True))

    @property
    def stop_failure(self):
        return self.snapshot.get('stop_failure', '')

    @property
    def active(self):
        return self.snapshot.get('active', 0) if self.fresh() else 0

    @property
    def goal_handle(self):
        return self.snapshot.get('goal_handle', False)

    @property
    def pending(self):
        return self.snapshot.get('pending', False)

    @property
    def accepted_plans(self):
        return {int(k): SimpleNamespace(**v) for k, v in self.snapshot.get('accepted_plans', {}).items()}

    @property
    def candidate(self):
        value = self.snapshot.get('candidate')
        return SimpleNamespace(id=value) if value is not None else None
