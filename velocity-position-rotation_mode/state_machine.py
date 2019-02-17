

class State_Machine:

    def __init__(self, states = {}, sm_name = ""):
        self.state_dict = states
        self.current_state = None
        self.sm_name = sm_name

    def add_state(self, name, state_class):
        self.state_dict[name] = state_class

    def set_state(self, name):
        if self.current_state != name:
            print(f"{self.sm_name}: transitioning to {name}")

        self.current_state = name


    def get_state(self):
        return self.current_state

    def run(self):
        state_class  = self.state_dict.get(self.current_state)
        if not state_class:
            raise Exception(f"state {self.current_state} not found", self.state_dict)
        self.set_state(state_class.run())