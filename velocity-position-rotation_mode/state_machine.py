

class State_Machine:

    def __init__(self, states = {}):
        self.state_dict = states
        self.current_state = None

    def add_state(self, name, state_class):
        self.state_dict[name] = state_class

    def set_state(self, name):
        if self.current_state != name:
            print(f"transitioning to {name}")

        self.current_state = name


    def get_state(self):
        return self.current_state

    def run(self):
        state_class  = self.state_dict.get(self.current_state)
        if not state_class:
            raise Exception(f"state {self.current_state} not found", self.state_dict)
        self.set_state(state_class.run())