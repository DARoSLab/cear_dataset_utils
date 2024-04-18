import dv_processing as dv
import sys
sys.path.append("..")
from src.my_utils import getargs

args = getargs()

root_path = args.input_dir
# Replace 'path/to/your/aedat4/file.aedat' with the actual path to your AEDAT4 file
aedat4_file_path = root_path+'event.aedat4'
txt_path = root_path+'event.txt'

# Open any camera
reader = dv.io.MonoCameraRecording(aedat4_file_path)


with open(txt_path, 'w') as f:
    # Run the loop while camera is still connected
    while reader.isRunning():
        # Read batch of events
        events = reader.getNextEventBatch()
        if events is not None:
            # Print received packet time range
            # events: <class 'dv_processing.EventStore'>
            for ev in events:
                print(f"Sliced event [{ev.timestamp()}, {ev.x()}, {ev.y()}, {ev.polarity()}]")
                f.write(f"{ev.timestamp()} {ev.x()} {ev.y()} {ev.polarity()}\n")
