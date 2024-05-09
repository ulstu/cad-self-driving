import yaml
import importlib.util
import os
import sys
import itertools


class FiniteStateMachine:
    def __init__(self, config_file, node):
        self.node = node
        self.config = self.load_config(config_file)
        self.states_dir = self.config['states_dir']
        self.states = self.load_states(self.config['states'])
        self.workers = self.load_workers(self.config["workers"])
        self.workerstates = self.config['workerstates']
        self.current_states = {'start': self.states['start']} #[self.states['start']()]
        self.current_workers = {}
        for state_key, state in self.current_states.items():
            for ws in self.workerstates[state_key]:
                if not ws in self.current_workers:
                    self.current_workers[ws] = self.workers[ws]
        self.transitions = self.config['transitions']
        

    def load_config(self, config_file):
        with open(config_file, 'r') as file:
            return yaml.safe_load(file)

    def detect_path(self, py_filename, full_path):
        for p in sys.path:
            fp = f'{p}{full_path}'
            if os.path.exists(f'{fp}{py_filename}'):
                if os.path.dirname(fp) not in sys.path:
                    sys.path.append(fp)
                return fp
        return ''

    def load_states(self, state_paths):
        states = {}
        module_path = None
        for state_name, path in state_paths.items():
            if not module_path:
                module_path = self.detect_path(path, full_path = f'/webots_ros2_suv/states/{self.states_dir}')
            module_name, class_name = os.path.splitext(os.path.basename(f'{module_path}{path}'))
            spec = importlib.util.spec_from_file_location(f'{module_name}', f'{module_path}{path}')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            states[state_name] = getattr(module, module_name)(self.node)
        return states

    def load_workers(self, confworkers):
        workers = {}
        module_path = None
        for worker_name, path in confworkers.items():
            if not module_path:
                module_path = self.detect_path(path, full_path = '/webots_ros2_suv/workers/')
            module_name, class_name = os.path.splitext(os.path.basename(f'{module_path}{path}'))
            spec = importlib.util.spec_from_file_location(f'{module_name}', f'{module_path}{path}')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self.node._logger.info(f'LOAD INFO: {module} {module_name}')
            workers[worker_name] = getattr(module, module_name)(self.node)
        return workers
    
    def on_event(self, event, world_model=None):
        new_states = {}
        for key, state in self.current_states.items():
            new_event = state.on_event(event, world_model)
            if new_event:
                event = new_event
            if (new_states is None or len(new_states) == 0) and event:
                #state_name = state.__qualname__.lower().replace('state', '')
                state_name = type(state).__name__.lower().replace('state', '')
                if event in self.transitions[state_name]:
                    for s in self.transitions[state_name][event]:
                        new_states[s] = self.states[s] 
        if len(new_states) > 0:
            self.current_states = new_states
            self.current_workers = {}
            for state_key, state in self.current_states.items():
                for ws in self.workerstates[state_key]:
                    if not ws in self.current_workers:
                        self.current_workers[ws] = self.workers[ws]
                self.node._logger.info(f"Transitioned to {[type(state).__name__.lower().replace('state', '') for s in self.current_states]}")
            
            self.node._logger.info(f'Current workers: {str(self.current_workers)}')
            self.node._logger.info(f'Current states: {str(self.current_states)}')

    def on_data(self, world_model, source="general"):
        for key, worker in self.current_workers.items():
            #self.node._logger.info(f'Invoking on_data event from {source} on worker {str(worker)} ')
            world_model = worker.on_data(world_model)
        return world_model


    def find_worker(self, worker_state):
        for state_key, state in self.current_states.items():
            if worker_state in self.workerstates[state_key]: 
                return self.workerstates[state_key][worker_state]
        return None