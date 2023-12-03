import csv
import queue
import threading
import datetime as dt
from typing import List

class TelemetryThread(threading.Thread):
    def __init__(self, queue: queue.Queue):
        super().__init__()
        self.queue = queue
        self.running = False
        self.filename = f'telemetry_{dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.csv'

    def run(self):
        self.running = True
        while self.running:
            # Get data from the queue
            data = self.queue.get()

            # Check if the data is a stop signal
            if data is None:
                break

            # Write the data to the CSV file
            self.write_to_csv(data)

    def stop(self):
        self.running = False

    def write_to_csv(self, data: List[float]):
        # Open the CSV file in append mode
        with open(self.filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # Check if the file is empty
            if csvfile.tell() == 0:
                # construct the header row
                header = ['time']
                for i in range(int((len(data) - 1) / 5)):
                    header.append(f'agent_{i}_x')
                    header.append(f'agent_{i}_y')
                    header.append(f'agent_{i}_z')
                    header.append(f'agent_{i}_yaw')
                    header.append(f'agent_{i}_pitch')
                writer.writerow(header)

            # Write the data to the CSV file
            writer.writerow(data)

class Telemetry:
    def __init__(self, queue: queue.Queue):
        self.queue = queue
        self.telemetry_thread = TelemetryThread(queue)

    def start(self):
        self.telemetry_thread.start()

    def stop(self):
        # Send a stop signal to the thread
        self.queue.put(None)
        self.telemetry_thread.stop()
        self.telemetry_thread.join()

    def get_telemetry_data(self):
        # Return the telemetry data here
        pass

    def get_telemetry_data(self):
        # Return the telemetry data here
        pass
