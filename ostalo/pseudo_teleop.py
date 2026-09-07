# hand_teleop_node
# subscribea se na /hand_pose i publisha geometry_msgs/Twist na /cmd_vel_hand koji se dalje
# feeda na ASTRO twist_mux --> treba definirati prioritet nad joy i nav (joy ostaviti kao najveći prioritet)
# "deadman switch" je to da se info šalje samo kada je ruka u šaci

#neki pocetni parametri 
CONFIG: 
    deadzone_radius          = 0.03     # m --> zona oko početne točke (podesiti jednom kad će sve radit)
    max_linear_speed          = 1.0     # m/s --> izvučeno iz astro_diff_drive_controller limit
    max_angular_speed         = 1.0     # rad/s --> izvučeno iz astro_diff_drive_controller limit
    map_range_linear          = 0.20    # m --> izmjeriti koliki range of motion imam s rukama naprijed - nazad
    map_range_angular         = 0.20    # m --> izmjeriti koliki range of motion imam s rukama lijevo - desno
    smoothing_alpha            = 0.3
    tracking_timeout           = 0.4    # s  (otprilike jer je twist_mux 0.5s)
    publish_rate               = 20      # Hz

    # parametri kalibracije - izvučeni iz open source algoritma za detekciju ruku
    calibration_gesture        = "FIST"  # možda promijeniti u neku drugu gestu??
    stability_window_size      = 15
    stability_threshold        = 0.015   # m
    stability_hold_duration    = 1.0     # s
    averaging_sample_count     = 30
    calibration_timeout        = 15.0    # s

STATE:
    neutral_position    = None
    last_msg_time         = None
    latest_hand_pos       = None
    latest_gesture         = "OPEN"  # na početku uvijek open da ne šalje info odmah (prvo kalibracija)
    filtered_linear_x     = 0.0
    filtered_angular_z    = 0.0

    calibration_active    = False
    position_buffer        = []
    stable_since            = None
    averaging_samples      = []
    calibration_start_time = None


# početak 
ON_START:
    subscribe to /hand_pose -> on_hand_pose_received
    create publisher on /cmd_vel_hand          # geometry_msgs/Twist
    create timer at publish_rate -> publish_velocity()
    start_calibration()


# kalibrajica
FUNCTION start_calibration():
    calibration_active = True
    position_buffer = []
    averaging_samples = []
    stable_since = None
    calibration_start_time = now()
    announce_to_operator("Hold a still fist at your resting position to calibrate")


FUNCTION run_calibration_step(msg):
    IF (now() - calibration_start_time) > calibration_timeout:
        calibration_active = False
        announce_to_operator("Calibration timed out, retry")
        RETURN

    IF msg.gesture != calibration_gesture OR msg.confidence < min_landmark_confidence:
        position_buffer = []
        stable_since = None
        RETURN

    position_buffer.append(msg.position)
    IF length(position_buffer) > stability_window_size:
        position_buffer.pop_oldest()
    IF length(position_buffer) < stability_window_size:
        RETURN

    spread = max_distance_between_any_two_points(position_buffer)
    IF spread > stability_threshold:
        stable_since = None
        RETURN

    IF stable_since is None:
        stable_since = now()

    IF (now() - stable_since) >= stability_hold_duration:
        averaging_samples.append(msg.position)
        IF length(averaging_samples) >= averaging_sample_count:
            finish_calibration()


FUNCTION finish_calibration():
    neutral_position = average(averaging_samples)
    calibration_active = False
    announce_to_operator("Calibration complete, neutral position set")



# main dio programa (detekcija i procesiranje)
FUNCTION on_hand_pose_received(msg):
    last_msg_time   = now()
    latest_hand_pos = msg.position
    latest_gesture  = msg.gesture

    IF calibration_active:
        run_calibration_step(msg)


FUNCTION publish_velocity():   # pozivat periodicno ili na svaki hand_pose msgs?
    IF calibration_active OR neutral_position is None:
        publish_zero_twist()
        RETURN

    IF (now() - last_msg_time) > tracking_timeout:
        publish_zero_twist()        # ne detektira ruku stop!
        RETURN

    IF latest_gesture != "FIST":
        publish_zero_twist()        # deadman switch --> sve osim šake stop!
        RETURN

    #podijeljeno na 5 lvl 
    # 1. offset od kalibrirane nulte pozicije
    dx = latest_hand_pos.x - neutral_position.x          # desno +, lijevo - (ovisi o kameri, provjeri specs kamere)
    dz = neutral_position.z - latest_hand_pos.z          # blize +, dalje - 

    # 2. radijalna deadzona
    IF magnitude(dx, dz) < deadzone_radius:
        dx, dz = 0, 0

    # 3. skaliranje na ASTRO limite
    norm_forward = clamp(dz / map_range_linear, -1, 1)
    norm_turn    = clamp(dx / map_range_angular, -1, 1)

    target_linear_x  = norm_forward * max_linear_speed
    target_angular_z = norm_turn    * max_angular_speed

    # 4. smoothing
    filtered_linear_x  = smoothing_alpha * target_linear_x  + (1 - smoothing_alpha) * filtered_linear_x
    filtered_angular_z = smoothing_alpha * target_angular_z + (1 - smoothing_alpha) * filtered_angular_z

    # 5. publishanje
    twist.linear.x  = filtered_linear_x
    twist.angular.z = filtered_angular_z
    publish(twist)   # publishaj /cmd_vel_hand


FUNCTION publish_zero_twist():
    filtered_linear_x  = 0.0    # resetiraj filter state za smooth ponovno pokretanje
    filtered_angular_z = 0.0
    publish(Twist(0,0,0,0,0,0))   # publishat na /cmd_vel_hand


# na kraju dodat u ASTRO-v twist_mux.yaml
#hand:
  #topic   : cmd_vel_hand
  #timeout : 0.5
  #priority: 50

# na ovaj način sve na ASTRO-u ostaje isto i ovo se lako ugasi/ukloni