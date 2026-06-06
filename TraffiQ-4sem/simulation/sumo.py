import os
import time
import threading
import subprocess
import heapq

try:
    import traci
    TRACI_AVAILABLE = True
except ImportError:
    TRACI_AVAILABLE = False
    print("traci not available.")

SUMO_CFG    = os.path.join(os.path.dirname(__file__), "final.sumocfg")
SUMO_BINARY = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"
TRACI_PORT  = 8813
TL_ID       = "clusterJ2_clusterJ2_J4_J5"

_sumo_started = False
_sumo_process = None
_sim_step     = 0
_lock         = threading.Lock()

_lane_state: dict = {}


_active_lanes: set = set()

_current_green_lane = None
_green_steps_left   = 0
_yellow_phase       = False
_yellow_steps_left  = 0
_emergency_active   = False


BOOST_FACTOR       = 2.5          
MIN_GREEN_TIME     = 5
MAX_GREEN_TIME     = 20
TOTAL_CYCLE        = 40
YELLOW_TIME        = 1           
EMERGENCY_GREEN    = 18


MAX_WAIT_THRESHOLD = 25          


LANE_INDEX = {
    "-E2_0": 0,  
    "-E0_0": 1,   
    "-E1_0": 2,   
    "E0_0":  3,   
}
ALL_LANES = list(LANE_INDEX.keys())


def dp_allocate_green_times(lane_vehicle_counts: dict) -> dict:
    
    active = {lid: cnt for lid, cnt in lane_vehicle_counts.items() if cnt > 0}
    lanes   = list(active.keys())
    n       = len(lanes)
    total_v = sum(active.values())

    if n == 0:
        return {}          

    if total_v == 0:
        return {lid: MIN_GREEN_TIME for lid in lanes}

    T   = TOTAL_CYCLE
    INF = float('-inf')

  
    ideal = {lid: (active[lid] / total_v) * T for lid in lanes}

    dp     = [[INF] * (T + 1) for _ in range(n + 1)]
    choice = [[0]   * (T + 1) for _ in range(n + 1)]
    dp[0][0] = 0.0

    for i in range(1, n + 1):
        lid  = lanes[i - 1]
        id_t = ideal[lid]
        for t in range(T + 1):
            for alloc in range(MIN_GREEN_TIME, MAX_GREEN_TIME + 1):
                if t - alloc < 0:
                    break
                prev = dp[i - 1][t - alloc]
                if prev == INF:
                    continue
                score = prev + min(alloc / id_t, 1.0) if id_t > 0 else prev + 1.0
                if score > dp[i][t]:
                    dp[i][t]     = score
                    choice[i][t] = alloc

    best_t = max(range(T + 1), key=lambda t: dp[n][t])

    result = {}
    rem = best_t
    for i in range(n, 0, -1):
        alloc = choice[i][rem]
        result[lanes[i - 1]] = alloc
        rem -= alloc

    return result


def calculate_priority(lane_data: dict) -> float:
    
    vehicles = lane_data.get("vehicles", 0)
    wait     = lane_data.get("wait_time", 0.0)


    if vehicles == 0 and wait == 0.0:
        return 0.0

    return vehicles + wait * BOOST_FACTOR


def build_state(green_lane: str = None, yellow_lane: str = None) -> str:
    state = ['r', 'r', 'r', 'r']
    if green_lane and green_lane in LANE_INDEX:
        state[LANE_INDEX[green_lane]] = 'G'
    if yellow_lane and yellow_lane in LANE_INDEX:
        state[LANE_INDEX[yellow_lane]] = 'y'
    return ''.join(state)


def start_sumo():
    global _sumo_started, _sumo_process
    if not TRACI_AVAILABLE:
        print("TraCI unavailable.")
        return False
    if _sumo_started:
        return True
    if not os.path.exists(SUMO_CFG):
        print("Config file not found.")
        return False
    try:
        print(f"[TraffiQ] Launching sumo-gui: {SUMO_CFG}")
        os.environ["SUMO_HOME"] = r"C:\Program Files (x86)\Eclipse\Sumo"
        cmd = [
            SUMO_BINARY, "-c", SUMO_CFG,
            "--remote-port", str(TRACI_PORT),
            "--no-step-log", "true",
            "--collision.action", "none",
            "--start", "--quit-on-end", "false"
        ]
        _sumo_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[TraffiQ] PID: {_sumo_process.pid} — waiting for TraCI...")
        for _ in range(30):
            time.sleep(1)
            try:
                traci.init(port=TRACI_PORT, numRetries=1)
                _sumo_started = True
                print("[TraffiQ] TraCI connected.")
                traci.trafficlight.setProgram(TL_ID, "0")
                return True
            except Exception:
                pass
        print("[TraffiQ] TraCI timed out.")
        return False
    except Exception as e:
        print(f"[TraffiQ] Error: {e}")
        return False


def stop_sumo():
    global _sumo_started, _sumo_process
    if TRACI_AVAILABLE and _sumo_started:
        try: traci.close()
        except: pass
    if _sumo_process:
        try: _sumo_process.terminate()
        except: pass
    _sumo_started = False
    _sumo_process = None


def get_lane_data():
    global _sim_step, _lane_state, _active_lanes
    global _current_green_lane, _green_steps_left
    global _yellow_phase, _yellow_steps_left, _emergency_active
    global _sumo_started

    with _lock:
        if not (TRACI_AVAILABLE and _sumo_started):
            return {"error": "SUMO not connected", "lanes": {}, "source": "none"}

        try:
            traci.simulationStep()
            _sim_step += 1
        except Exception as e:
            return {"error": str(e), "lanes": {}, "source": "none"}

        lanes = {}

        try:
          
            for lane_id in ALL_LANES:
                try:
                    vehicle_ids   = traci.lane.getLastStepVehicleIDs(lane_id)
                    vehicle_count = len(vehicle_ids)
                    wait_time     = round(traci.lane.getWaitingTime(lane_id), 1)

                   
                    if vehicle_count > 0:
                        _active_lanes.add(lane_id)
                    else:
                        _active_lanes.discard(lane_id)

                   
                    emergency = False
                    for vid in vehicle_ids:
                        try:
                            vtype = traci.vehicle.getTypeID(vid).lower()
                            if any(k in vtype for k in ["emergency", "ambulance", "police", "fire", "firetruck"]):
                                emergency = True
                                break
                        except: pass

                  
                    if lane_id not in _lane_state:
                        _lane_state[lane_id] = {
                            "last_served": 0,
                            "green_time":  MIN_GREEN_TIME,
                            "wait_time":   0.0,
                        }

                   
                    
                    if lane_id != _current_green_lane:
                        _lane_state[lane_id]["wait_time"] = wait_time

                    _lane_state[lane_id]["vehicles"]  = vehicle_count
                    _lane_state[lane_id]["emergency"] = emergency

                    lanes[lane_id] = {
                        "vehicles":       vehicle_count,
                        "emergency":      emergency,
                        "wait_time":      _lane_state[lane_id]["wait_time"],
                        "phase":          "red",
                        "time_remaining": 0,
                    }
                except Exception as e:
                    print(f"[TraffiQ] Lane {lane_id}: {e}")

            total_vehicles = sum(d["vehicles"] for d in lanes.values())

          
         
            emg_lane = next((lid for lid in lanes if lanes[lid]["emergency"]), None)

            if emg_lane:
                if not _emergency_active:
                    print(f"[TraffiQ]  EMERGENCY OVERRIDE → {emg_lane} | All other lanes STOPPED")
                    _emergency_active   = True
                    _current_green_lane = emg_lane
                    _green_steps_left   = EMERGENCY_GREEN
                    _yellow_phase       = False
                    if emg_lane in _lane_state:
                        _lane_state[emg_lane]["wait_time"] = 0.0
                _green_steps_left = max(0, _green_steps_left - 1)   # ← ADDED
                try:
                    traci.trafficlight.setRedYellowGreenState(TL_ID, build_state(green_lane=emg_lane))
                except Exception as e:
                    print(f"[TraffiQ] TL error: {e}")

            else:
                if _emergency_active:                                
                    _emergency_active  = False                      
                    _yellow_phase      = True                    
                    _yellow_steps_left = YELLOW_TIME           
                else:                                               
                    _emergency_active  = False                       

                if _yellow_phase:
                    _yellow_steps_left -= 1
                    try:
                        traci.trafficlight.setRedYellowGreenState(
                            TL_ID, build_state(yellow_lane=_current_green_lane)
                        )
                    except: pass
                    if _yellow_steps_left <= 0:
                        _yellow_phase     = False
                        _green_steps_left = 0
                elif _green_steps_left <= 0:

                    # Only consider lanes with ≥1 vehicle — empty lanes get 0 time
                    candidate_lanes = [lid for lid in _active_lanes if
                                       lanes.get(lid, {}).get("vehicles", 0) > 0]

                    if not candidate_lanes:
                        pass
                    else:
                        starved_lane = None
                        max_wait     = 0.0
                        for lid in candidate_lanes:
                            wt = _lane_state.get(lid, {}).get("wait_time", 0.0)
                            if wt >= MAX_WAIT_THRESHOLD and wt > max_wait:
                                max_wait     = wt
                                starved_lane = lid

                        if starved_lane:
                            best_lane = starved_lane
                            print(f"[TraffiQ] ⚠ STARVATION OVERRIDE → {best_lane} | "
                                  f"waited {max_wait:.1f}s — granted green despite lower vehicle count")
                        else:
                            heap = []
                            for lane_id in candidate_lanes:
                                score = calculate_priority(lanes.get(lane_id, {}))
                                heapq.heappush(heap, (-score, lane_id))

                            _, best_lane = heapq.heappop(heap)

                        active_counts = {
                            lid: lanes.get(lid, {}).get("vehicles", 0)
                            for lid in candidate_lanes
                        }
                        dp_times   = dp_allocate_green_times(active_counts)
                        green_secs = dp_times.get(best_lane, MIN_GREEN_TIME)

                        _current_green_lane = best_lane
                        _green_steps_left   = green_secs

                        if best_lane in _lane_state:
                            _lane_state[best_lane]["last_served"] = _sim_step
                            _lane_state[best_lane]["green_time"]  = green_secs
                            _lane_state[best_lane]["wait_time"]   = 0.0

                        score_val = calculate_priority(lanes.get(best_lane, {}))
                        print(f"[TraffiQ]  GREEN → {best_lane} | "
                              f"score={score_val:.1f} | DP time={green_secs}s | "
                              f"vehicles={active_counts.get(best_lane, 0)}")

                        try:
                            traci.trafficlight.setRedYellowGreenState(
                                TL_ID, build_state(green_lane=best_lane)
                            )
                        except Exception as e:
                            print(f"[TraffiQ] TL error: {e}")

               
                else:

                    _green_steps_left -= 1
                    if _green_steps_left == 0:
                        _yellow_phase      = True
                        _yellow_steps_left = YELLOW_TIME   # 1 second
                    try:
                        traci.trafficlight.setRedYellowGreenState(
                            TL_ID, build_state(green_lane=_current_green_lane)
                        )
                    except: pass

            try:
                actual_state = traci.trafficlight.getRedYellowGreenState(TL_ID)
                for lane_id, idx in LANE_INDEX.items():
                    if lane_id in lanes and idx < len(actual_state):
                        c = actual_state[idx].lower()
                        lanes[lane_id]["phase"]          = "green" if c == 'g' else "amber" if c == 'y' else "red"
                        lanes[lane_id]["time_remaining"] = _green_steps_left if c == 'g' else (
                            _yellow_steps_left if c == 'y' else 0
                        )
            except Exception as e:
                print(f"[TraffiQ] State read error: {e}")

            heap_display = []
            for lane_id, data in lanes.items():
                heapq.heappush(heap_display, (-calculate_priority(data), lane_id))

            priority_order = [lane for _, lane in sorted(heap_display)]   

            for rank, lane_id in enumerate(priority_order):
                if lane_id in lanes:
                    lanes[lane_id]["priority_rank"]  = rank + 1
                    _score_data = {                                           
                        "vehicles":  lanes[lane_id]["vehicles"],           
                        "wait_time": _lane_state.get(lane_id, {}).get("wait_time", 0.0) 
                    }                                                        
                    lanes[lane_id]["priority_score"] = round(calculate_priority(_score_data), 1) 
                    lanes[lane_id]["green_time"]     = _lane_state.get(lane_id, {}).get("green_time", MIN_GREEN_TIME)
                    lanes[lane_id]["is_active"]      = lane_id in _active_lanes
                    lanes[lane_id]["wait_stored"]    = round(_lane_state.get(lane_id, {}).get("wait_time", 0.0), 1)
                    if _emergency_active:
                        lanes[lane_id]["override"] = "emergency_green" if lanes[lane_id]["phase"] == "green" else "stopped"

            return {
                "lanes":              lanes,
                "sim_step":           _sim_step,
                "source":             "traci",
                "priority_order":     priority_order,
                "top_priority_lane":  priority_order[0] if priority_order else None,
                "emergency_lane":     emg_lane,
                "active_lanes":       list(_active_lanes),
                "total_vehicles":     total_vehicles,
                "current_green_lane": _current_green_lane,
                "green_steps_left":   _green_steps_left,
            }

        except Exception as e:
            _sumo_started = False
            print(f"[TraffiQ] Fatal TraCI error: {e}")
            return {"error": str(e), "lanes": {}, "source": "none"}