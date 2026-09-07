# hand_capture_node
# priča s 3D kamerom i hand-tracking modelom 
# publisha /hand_pose koji drugi čvor prima i pretvara u brzine za ASTRO-a
# /hand_pose ima poziciju (za odrediti brzinu), ali ima i gestu i confidence kao safety feature

# neki početni parametri
CONFIG:
    camera_fps               = 30            # ovisi o kameri
    publish_rate             = 20            # Hz, dobiveno iz fps
    gesture_debounce_frames  = 5             # kolko frameova da se potvrdi gesta (safety za krivo ocitanje)
    min_landmark_confidence  = 0.6           # koliko mora biti siguran u gestu da šalje info dalje
    fist_curl_threshold      = 0.6           # početni parametar --> odredi trial and error

STATE:
    confirmed_gesture  = "OPEN"              # proizlazi iz "deadman switcha"
    pending_gesture    = "OPEN"
    pending_count      = 0
    latest_position    = None
    latest_confidence   = 0.0


# početak
ON_START:
    init_3d_camera()
    init_hand_landmark_model()
    create publisher on /hand_pose            # custom msg --> treba imati position(x,y,z), gesture, confidence, header
    create timer at publish_rate -> publish_latest_state()


# main dio programa
MAIN LOOP (runs at camera_fps):
    color_frame, depth_frame = camera.get_frames()
    landmarks_2d, confidence = hand_model.detect(color_frame)

    IF landmarks_2d is None OR confidence < min_landmark_confidence:
        raw_gesture  = "OPEN"          # "deadman" za sve što nije šaka ili ako je premali confidence 
        raw_position = None
    ELSE:
        landmarks_3d = deproject_to_3d(landmarks_2d, depth_frame, camera_intrinsics)
        raw_position = compute_palm_center(landmarks_3d)
        raw_gesture  = classify_gesture(landmarks_3d)

    debounce_gesture(raw_gesture)
    latest_position   = raw_position
    latest_confidence = confidence


FUNCTION classify_gesture(landmarks_3d):
    curled_count = 0
    FOR finger IN [index, middle, ring, pinky]:         # vjerojatno ću koristiti MediaPipe koji bi trebao imati
        tip_dist = distance(finger.tip, palm_center)    # već ugrađenu detekciju osnovnih gesti pa ovo možda neće biti potrebno
        pip_dist = distance(finger.pip, palm_center)
        IF tip_dist < pip_dist * fist_curl_threshold:
            curled_count += 1

    IF curled_count >= 4:
        RETURN "FIST"
    ELSE IF curled_count == 0:
        RETURN "OPEN"
    ELSE:
        RETURN "UNKNOWN"     # sve što nije 100% šaka ne vraća True


FUNCTION debounce_gesture(raw_gesture):
    IF raw_gesture == pending_gesture:
        pending_count += 1
    ELSE:
        pending_gesture = raw_gesture
        pending_count = 1

    IF pending_count >= gesture_debounce_frames:
        confirmed_gesture = pending_gesture


FUNCTION publish_latest_state():   # timer poziva (ne ovisi o fps kamere)
    msg.header.stamp = now()
    msg.gesture       = confirmed_gesture
    msg.confidence    = latest_confidence
    msg.position      = latest_position IF latest_position is not None ELSE last_known_position
    publish(msg)   # publisha /hand_pose