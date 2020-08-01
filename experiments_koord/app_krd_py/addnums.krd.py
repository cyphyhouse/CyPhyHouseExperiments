from src.harness.agentThread import AgentThread


class Addnums(AgentThread):

    def __init__(self, config, motion_config):
        super(Addnums, self).__init__(config, motion_config)

    def initialize_vars(self):
        self.locals = {}
        self.locals['added'] = False
        self.locals['finalsum'] = None
        self.create_aw_var('sum', int, 0)
        self.create_aw_var('numadded', int, 0)
        self.initialize_lock('adding')

    def loop_body(self):
        if  not self.locals['added']:
            if not self.lock('adding'):
                return
            self.write_to_shared('sum', None, self.read_from_shared('sum', None) + self.pid() * 2)
            self.write_to_shared('numadded', None, self.read_from_shared('numadded', None) + 1)
            self.locals['added'] = True
            self.unlock('adding')
            return
        if self.read_from_shared('numadded', None) == self.num_agents():
            self.locals['finalsum'] = self.read_from_shared('sum', None)
            self.log("Agent ")
            self.log(self.pid())
            self.log(": finalsum = ")
            self.log(self.locals['finalsum'])
            self.log("\n")

            self.trystop()
            return
