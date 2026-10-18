#!/usr/bin/env python3
import subprocess
import time
import json
import os
import sys

CONTAINER_NAME = "benchmark_test"
WORKSPACE = "/home/rosdev/gazebo_workspace"
OUTPUT_DIR = "/home/thevindu/SocialGym2/gazebo_workspace/evaluation_results"

EPISODES = [
    # Corridor variations
    {"scenario": "corridor", "run_id": "corridor_j00", "jitter": 0.0, "timeout": 135, "seed": 42},
    {"scenario": "corridor", "run_id": "corridor_j05", "jitter": 0.5, "timeout": 135, "seed": 42},
    {"scenario": "corridor", "run_id": "corridor_j10", "jitter": 1.0, "timeout": 135, "seed": 42},
    {"scenario": "corridor", "run_id": "corridor_j15", "jitter": 1.5, "timeout": 135, "seed": 42},
    {"scenario": "corridor", "run_id": "corridor_j20", "jitter": 2.0, "timeout": 135, "seed": 42},
    # Doorway variations
    {"scenario": "doorway", "run_id": "doorway_j00", "jitter": 0.0, "timeout": 45, "seed": 42},
    {"scenario": "doorway", "run_id": "doorway_j05", "jitter": 0.5, "timeout": 45, "seed": 42},
    {"scenario": "doorway", "run_id": "doorway_j10", "jitter": 1.0, "timeout": 45, "seed": 42},
    {"scenario": "doorway", "run_id": "doorway_j15", "jitter": 1.5, "timeout": 45, "seed": 42},
    {"scenario": "doorway", "run_id": "doorway_j20", "jitter": 2.0, "timeout": 45, "seed": 42},
]

def check_container_health():
    """Checks container processes and memory."""
    ps_out = subprocess.check_output(["docker", "exec", CONTAINER_NAME, "ps", "aux"], text=True)
    mem_out = subprocess.check_output(["docker", "exec", CONTAINER_NAME, "free", "-m"], text=True)
    return ps_out, mem_out

def restart_container():
    print(f"--> Restarting container '{CONTAINER_NAME}' cleanly...", flush=True)
    subprocess.run(["docker", "restart", CONTAINER_NAME], check=True)
    time.sleep(2)

def run_episode(ep_idx: int, total_eps: int, ep: dict):
    scenario = ep['scenario']
    run_id = ep['run_id']
    jitter = ep['jitter']
    timeout = ep['timeout']
    seed = ep['seed']

    print(f"\n=======================================================", flush=True)
    print(f"=== BATCH EPISODE {ep_idx}/{total_eps}: {run_id} ({scenario}, jitter={jitter}s) ===", flush=True)
    print(f"=======================================================", flush=True)

    # 1. Clean container restart & pre-check
    restart_container()
    ps_pre, mem_pre = check_container_health()
    mem_line = [l for l in mem_pre.splitlines() if l.startswith("Mem:")][0]
    print(f"Pre-run memory: {mem_line}", flush=True)

    # 2. Execute episode script
    cmd = [
        "docker", "exec", CONTAINER_NAME,
        f"{WORKSPACE}/execute_episode.sh",
        scenario,
        "robot_present",
        run_id,
        str(seed),
        str(jitter),
        str(timeout)
    ]
    t_start = time.time()
    wall_timeout = int(timeout / 0.20) + 90
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=wall_timeout)
        wall_time = time.time() - t_start
        print(f"Episode execution finished in {wall_time:.1f}s (exit code {proc.returncode})", flush=True)
        # Print tail of output
        lines = proc.stdout.splitlines()
        summary_lines = [l for l in lines if any(k in l for k in ["EPISODE SUMMARY", "Reason:", "RTF:", "Min Robot-Human", "Contact Events", "Nav2 final status", "Trajectory saved"])]
        for sl in summary_lines:
            print(f"  {sl}", flush=True)
    except subprocess.TimeoutExpired:
        print(f"ERROR: Episode {run_id} hit hard python timeout ({timeout + 60}s)!", flush=True)
        restart_container()

    # 3. Post-run process check
    ps_post, mem_post = check_container_health()
    mem_post_line = [l for l in mem_post.splitlines() if l.startswith("Mem:")][0]
    print(f"Post-run memory: {mem_post_line}", flush=True)

    # 4. Verify evaluation file exists
    res_file = os.path.join(OUTPUT_DIR, f"evaluation_{scenario}_{run_id}.json")
    if os.path.exists(res_file):
        print(f"SUCCESS: Evaluation file exists: {res_file}", flush=True)
    else:
        print(f"WARNING: Evaluation file NOT FOUND: {res_file}", flush=True)

def main():
    total = len(EPISODES)
    print(f"Starting Batch Evaluation of {total} episodes...", flush=True)
    t0_batch = time.time()

    for idx, ep in enumerate(EPISODES, 1):
        run_episode(idx, total, ep)

    batch_duration = time.time() - t0_batch
    print(f"\n=======================================================", flush=True)
    print(f"=== BATCH EVALUATION COMPLETED in {batch_duration/60:.2f} minutes ===", flush=True)
    print(f"=======================================================", flush=True)

if __name__ == '__main__':
    main()
