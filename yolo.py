import cv2
import serial
import numpy as np
from mss import mss
from ultralytics import YOLO

# --- CONFIG ---
# Ensure this is the correct port in Device Manager
PORT = 'COM12' 
try:
    ser = serial.Serial(PORT, 115200, timeout=0.01)
except Exception as e:
    print(f"Error opening serial port {PORT}: {e}")
    exit()

model = YOLO("emergency.pt")
# Mapping for on-screen labels
DIR_LABELS = {b'N': "NORTH", b'S': "SOUTH", b'E': "EAST", b'W': "WEST", b'C': ""}

with mss() as sct:
    monitor = sct.monitors[1]
    print("AI Traffic System Running. Press 'q' to exit.")

    while True:
        # 1. Capture Screen
        img = np.array(sct.grab(monitor))
        frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        h, w, _ = frame.shape

        # --- AI DETECTION ---
        results = model.predict(frame, conf=0.5, verbose=False)
        cmd = b'C'
        
        for r in results:
            for box in r.boxes:
                if "ambulance" in model.names[int(box.cls[0])].lower():
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cx, cy = (x1+x2)/2, (y1+y2)/2
                    
                    # Determine Direction based on center point
                    if cy < h/3: cmd = b'N'
                    elif cy > 2*h/3: cmd = b'S'
                    elif cx < w/3: cmd = b'W'
                    elif cx > 2*w/3: cmd = b'E'
                    
                    # Draw Bounding Box (Red)
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
                    
                    # Add Direction Label onto the box
                    label_text = f"EMG: {DIR_LABELS[cmd]}"
                    if cmd != b'C':
                        cv2.putText(frame, label_text, (int(x1), int(y1)-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Send command to FPGA
        ser.write(cmd)

        # --- UI OVERLAY (Grid Lines) ---
        grid_color = (100, 100, 100) # Grey
        thickness = 2
        # Horizontal lines at 1/3 and 2/3 height
        cv2.line(frame, (0, int(h/3)), (w, int(h/3)), grid_color, thickness)
        cv2.line(frame, (0, int(2*h/3)), (w, int(2*h/3)), grid_color, thickness)
        # Vertical lines at 1/3 and 2/3 width
        cv2.line(frame, (int(w/3), 0), (int(w/3), h), grid_color, thickness)
        cv2.line(frame, (int(2*w/3), 0), (int(2*w/3), h), grid_color, thickness)

        # Resize and Display
        cv2.imshow("IEEE CASS AI TRAFFIC VIEW", cv2.resize(frame, (1280, 720)))
        if cv2.waitKey(1) & 0xFF == ord('q'): break

ser.close()
cv2.destroyAllWindows()