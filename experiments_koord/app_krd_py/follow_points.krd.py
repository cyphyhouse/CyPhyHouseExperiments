from src.harness.agentThread import AgentThread


class Follow_points(AgentThread):

    def __init__(self, config, motion_config):
        super(Follow_points, self).__init__(config, motion_config)

    def initialize_vars(self):
        self.locals = {}
        self.locals['tries'] = 1
        self.locals['p'] = 9 - 2 * self.pid()
        self.write_to_actuator('Motion.target', self.read_from_sensor('Motion.position') + self.pos3d(0.0, 0.0, 1.0))

    def loop_body(self):
        if self.locals['tries'] == 1:
            self.write_to_actuator('Motion.target', self.pos3d( -self.locals['p'], self.locals['p'], 1.0))
            self.locals['tries'] = 2
            return
        if self.locals['tries'] == 2 and self.read_from_sensor('Motion.reached'):
            self.write_to_actuator('Motion.target', self.pos3d(self.locals['p'], self.locals['p'], 1.0))
            self.locals['tries'] = 3
            return
        if self.locals['tries'] == 3 and self.read_from_sensor('Motion.reached'):
            self.write_to_actuator('Motion.target', self.pos3d(self.locals['p'],  -self.locals['p'], 1.0))
            self.locals['tries'] = 4
            return
        if self.locals['tries'] == 4 and self.read_from_sensor('Motion.reached'):
            self.write_to_actuator('Motion.target', self.pos3d( -self.locals['p'],  -self.locals['p'], 1.0))
            self.locals['tries'] = 5
            return
        if self.locals['tries'] == 5 and self.read_from_sensor('Motion.reached'):
            self.trystop()
            return
