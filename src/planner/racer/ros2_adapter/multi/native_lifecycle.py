"""Read lifecycle evidence from the locked RACER execution log; never command motion."""
import re
import uuid

class NativeLifecycle:
    def __init__(self, identity):
        self.identity=identity
        self.session=uuid.uuid4().hex
        self.generation=0
        self.reason=None
        self.status=None
        self.completion=None
        self.fault=None
    def feed(self, line):
        if 'Fail to solve ATSP.' in line:self.fault='grid_solver_failed'
        stop=re.search(r'handoff stop requested: (.*)',line)
        if stop:self.reason=stop.group(1).strip()
        applied=re.search(r'handoff status applied: status=(\d+)',line)
        if applied:self.status=int(applied.group(1))
        transition=re.search(r'Drone '+str(self.identity)+r' from (\w+) to (\w+)',line)
        if transition:
            old,new=transition.groups()
            if old=='WAIT_TRIGGER' and new in ('PLAN_TRAJ','FINISH'):
                self.generation+=1;self.reason=None;self.status=None;self.fault=None
            self.completion=None
            if new=='FINISH':
                if self.status==4:self.completion='operator_stopped'
                elif old=='WAIT_TRIGGER':self.completion='initial_no_frontier'
                elif self.reason=='return finished' and self.status==3:self.completion='native_return_finished'
                else:self.completion='unclassified_finish'
        return dict(native_session=self.session,trigger_generation=self.generation,
                    completion_reason=self.completion,last_native_stop_reason=self.reason,completion_fault=self.fault)
