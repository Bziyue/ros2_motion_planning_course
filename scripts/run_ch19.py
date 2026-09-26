#!/usr/bin/env python3
"""Run isolated, sequential A/B/C trials. Source ROS and the workspace first."""
import argparse
import csv
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys


def main():
    p=argparse.ArgumentParser();p.add_argument('--seeds',nargs='+',type=int,default=[42,43,44]);p.add_argument('--routes',nargs='+',choices=['A','B','C'],default=['A','B','C']);p.add_argument('--seconds',type=float,default=65.)
    p.add_argument('--domain',type=int,default=78);p.add_argument('--output',default='tmp/ch19_batch');args=p.parse_args()
    root=Path(__file__).resolve().parent.parent;out=(root/args.output).resolve();out.mkdir(parents=True,exist_ok=True);results=[]
    for seed in args.seeds:
        for route in args.routes:
            trial=out/f'{route}_{seed}';trial.mkdir(exist_ok=True)
            env=os.environ.copy();env['ROS_DOMAIN_ID']=str(args.domain)
            with (trial/'launch.log').open('w') as log:
                launch=subprocess.Popen(['ros2','launch','motion2d_bringup','ch19.launch.py',f'route:={route}',f'seed:={seed}','rviz:=false'],stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True,cwd=root)
                try:
                    subprocess.run([sys.executable,str(root/'scripts/check_ch19.py'),'--route',route,'--seed',str(seed),'--seconds',str(args.seconds),'--output',str(trial)],check=True,env=env,cwd=root)
                finally:
                    if launch.poll() is None:
                        os.killpg(launch.pid,signal.SIGINT)
                        try:launch.wait(timeout=12)
                        except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=5)
            result=json.loads((trial/'summary.json').read_text())
            evaluation=subprocess.run(['ros2','run','motion2d','navigation_clearance',str(seed),str(trial/'samples.csv')],text=True,capture_output=True,check=True,env=env,cwd=root)
            (trial/'clearance.csv').write_text(evaluation.stdout)
            clearances=[float(row['clearance_m']) for row in csv.DictReader(io.StringIO(evaluation.stdout))]
            result['min_true_disk_clearance_m']=min(clearances);(trial/'summary.json').write_text(json.dumps(result,indent=2));results.append(result)
            (out/'summary.json').write_text(json.dumps(results,indent=2));print(f"Finished {route}/{seed}: {result['reason']}, clearance {min(clearances):.4f} m",flush=True)
    with (out/'summary.csv').open('w') as file:
        w=csv.DictWriter(file,fieldnames=list(results[0]));w.writeheader();w.writerows(results)


if __name__=='__main__':main()
