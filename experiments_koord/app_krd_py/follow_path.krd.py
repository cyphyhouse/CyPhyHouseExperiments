from src.harness.agentThread import AgentThread


class Follow_path(AgentThread):

    def __init__(self, config, motion_config):
        super(Follow_path, self).__init__(config, motion_config)

    def initialize_vars(self):
        self.locals = {}
        self.locals['tries'] = 1

    def loop_body(self):
        if self.locals['tries'] == 1:
            self.locals['tries'] = 2
            self.write_to_actuator('Motion.path', self.toList(self.pos3d( -2 * self.pid(), 2 * self.pid(), 1.0), self.pos3d(2 * self.pid(), 2 * self.pid(), 1.0)))
            return
        if self.locals['tries'] == 2 and self.read_from_sensor('Motion.reached'):
            self.trystop()
            return
