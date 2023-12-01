import csv
import queue
import threading

class TelemetryThread(threading.Thread):
    def __init__(self, queue: queue.Queue):
        super().__init__()
        self.queue = queue
        self.running = False

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

    def write_to_csv(self, data):
        # Open the CSV file in append mode
        with open('telemetry.csv', 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)

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
