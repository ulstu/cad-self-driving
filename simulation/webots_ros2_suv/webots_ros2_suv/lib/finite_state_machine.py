import yaml
import importlib.util
import os
import sys


class FiniteStateMachine:
    def __init__(self, config_file, node):
        self.node = node
        self.config = self.load_config(config_file)
        self.states = self.load_states(self.config['states'])
        self.current_states = [self.states['start']()]
        self.transitions = self.config['transitions']

    def load_config(self, config_file):
        with open(config_file, 'r') as file:
            return yaml.safe_load(file)

    def detect_path(self, py_filename):
        for p in sys.path:
            full_path = f'{p}/webots_ros2_suv/states/'
            # self.node._logger.info(full_path)
            if os.path.exists(f'{full_path}{py_filename}'):
                if os.path.dirname(full_path) not in sys.path:
                    sys.path.append(full_path)
                return full_path
        return ''

    def load_states(self, state_paths):
        states = {}
        module_path = None
        for state_name, path in state_paths.items():
            if not module_path:
                module_path = self.detect_path(path)
            module_name, class_name = os.path.splitext(os.path.basename(f'{module_path}{path}'))
            spec = importlib.util.spec_from_file_location(f'{module_name}', f'{module_path}{path}')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            states[state_name] = getattr(module, module_name)
        return states
    
    def spin(self, scene_model):
        for state in self.current_states:
            res = state.spin(scene_model)

    def on_event(self, event):
        new_states = []
        for state in self.current_states:
            next_states = state.on_event(event)
            # Проверка, если next_states не определены в состоянии, 
            # используем таблицу переходов автомата
            if next_states is None:
                state_name = type(state).__name__.lower().replace('state', '')
                if event in self.transitions[state_name]:
                    new_states.extend([self.states[s]() for s in self.transitions[state_name][event]])
        if new_states:
            self.current_states = new_states
            self.node._logger.info(f"Transitioned to {[type(s).__name__ for s in self.current_states]}")
        else:
            self.node._logger.info("No transition available for this event")

