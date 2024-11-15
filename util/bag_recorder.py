import subprocess
import os
from pathlib import Path

class BagRecorderConfig:
    def __init__(self, topics=[], duration=None, output_file=None):
        self.topic = topics
        self.duration = duration
        self.output_file = output_file


class BagRecorder:
    def __init__(self, config):
        self.config = config

    def start_recording(self):
        args = ['rosbag record']
        if self.config.output_file:
            dirname = os.path.dirname(self.config.output_file)
            if dirname:
                Path(dirname).mkdir(parents=True, exist_ok=True)
            args.append(f'--output-name={self.config.output_file}')
        if self.config.duration:
            args.append(f'--duration={self.config.duration}')

        args.append(' '.join(self.config.topic))
        subprocess.run(' '.join(args), shell=True)

