import cv2
from ultralytics import YOLO

# Load the Pose Model
model = YOLO('yolov8n-pose.pt')

cap = cv2.VideoCapture(0)

# SETTINGS
CONF_THRESHOLD = 0.5 
TRIGGER_MARGIN = 40      # Pixels for Left/Right
STOP_MARGIN = 50         # Pixels ABOVE shoulder to trigger Stop

print("Press 'q' to quit.")

while True:
    success, frame = cap.read()
    if not success: break
    
    # --- IMPORTANT FOR ROBOTS ---
    # Remove this line for the actual car!
    frame = cv2.flip(frame, 1) 
    
    results = model(frame, verbose=False)
    annotated_frame = frame.copy()

    # Check if at least one person is detected
    if results[0].keypoints is not None and results[0].keypoints.data.shape[0] > 0:
        
        # Get data for the first person [0]
        kpts = results[0].keypoints.data.cpu().numpy()[0]
        
        # Ensure we have all keypoints
        if len(kpts) >= 17:
            # Extract points
            # 5: Left Shoulder, 6: Right Shoulder
            # 9: Left Wrist,   10: Right Wrist
            l_shldr = kpts[5]
            r_shldr = kpts[6]
            l_wrist = kpts[9]
            r_wrist = kpts[10]

            # Draw visual debug dots
            # Shoulders (Blue)
            if l_shldr[2] > CONF_THRESHOLD:
                cv2.circle(annotated_frame, (int(l_shldr[0]), int(l_shldr[1])), 8, (255, 0, 0), -1)
            if r_shldr[2] > CONF_THRESHOLD:
                cv2.circle(annotated_frame, (int(r_shldr[0]), int(r_shldr[1])), 8, (255, 0, 0), -1)
            # Wrists (Yellow)
            if l_wrist[2] > CONF_THRESHOLD:
                cv2.circle(annotated_frame, (int(l_wrist[0]), int(l_wrist[1])), 8, (0, 255, 255), -1)
            if r_wrist[2] > CONF_THRESHOLD:
                cv2.circle(annotated_frame, (int(r_wrist[0]), int(r_wrist[1])), 8, (0, 255, 255), -1)

            message = "WAITING..."
            color = (200, 200, 200) # Gray

            # --- LOGIC START ---

            stop_detected = False

            # 1. CHECK STOP (Highest Priority)
            # Logic: Wrist Y is LESS than Shoulder Y (Remember: Y=0 is top of screen)
            
            # Check Left Arm Up
            if l_wrist[2] > CONF_THRESHOLD and l_shldr[2] > CONF_THRESHOLD:
                if l_wrist[1] < (l_shldr[1] - STOP_MARGIN):
                    message = "!!! STOP !!!"
                    color = (0, 0, 255) # Red
                    stop_detected = True
            
            # Check Right Arm Up
            if r_wrist[2] > CONF_THRESHOLD and r_shldr[2] > CONF_THRESHOLD:
                if r_wrist[1] < (r_shldr[1] - STOP_MARGIN):
                    message = "!!! STOP !!!"
                    color = (0, 0, 255) # Red
                    stop_detected = True

            # 2. IF NOT STOP, CHECK LEFT/RIGHT
            if not stop_detected:
                
                # Check Pointing LEFT (Left Wrist < Left Shoulder X)
                if l_wrist[2] > CONF_THRESHOLD and l_shldr[2] > CONF_THRESHOLD:
                    dist = l_shldr[0] - l_wrist[0] 
                    if dist > TRIGGER_MARGIN:
                        message = "<<< GO LEFT"
                        color = (0, 255, 0) # Green
                        cv2.line(annotated_frame, (int(l_shldr[0]), int(l_shldr[1])), 
                                 (int(l_wrist[0]), int(l_wrist[1])), color, 4)

                # Check Pointing RIGHT (Right Wrist > Right Shoulder X)
                if r_wrist[2] > CONF_THRESHOLD and r_shldr[2] > CONF_THRESHOLD:
                    dist = r_wrist[0] - r_shldr[0] 
                    if dist > TRIGGER_MARGIN:
                        message = "GO RIGHT >>>"
                        color = (0, 255, 0) # Green
                        cv2.line(annotated_frame, (int(r_shldr[0]), int(r_shldr[1])), 
                                 (int(r_wrist[0]), int(r_wrist[1])), color, 4)

            # Display Status Box
            cv2.rectangle(annotated_frame, (0, 0), (640, 80), (0,0,0), -1)
            cv2.putText(annotated_frame, message, (50, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 2, color, 3)

    cv2.imshow("Cop Gesture Logic", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()
