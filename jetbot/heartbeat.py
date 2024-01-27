import enum
import traitlets
from traitlets.config.configurable import Configurable
import time
import threading


class Heartbeat(Configurable):
    class Status(enum.Enum):
        dead = 0
        alive = 1

    status = traitlets.UseEnum(Status, default_value=Status.dead)
    running = traitlets.Bool(default_value=False)
    
    # config
    period = traitlets.Float(default_value=0.5).tag(config=True)

    def __init__(self, *args, **kwargs):
        super(Heartbeat, self).__init__(*args,
                                        **kwargs)  # initializes traitlets

        self.pulseout = time.time()
        self.pulsein = time.time()
        self.start()

    def _run(self):
        while True:
            if not self.running:
                break
            if self.pulseout - self.pulsein >= self.period:
                self.status = Heartbeat.Status.dead
            else:
                self.status = Heartbeat.Status.alive
            self.pulseout = time.time()
            time.sleep(self.period)

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run)
        self.thread.start()

    def stop(self):
        self.running = False
