#!/usr/bin/env python3
"""Sequential live ROS CPU/CUDA paired replay, no concurrent timing workloads."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import numpy as np


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--beams',type=int,nargs='+',default=[90,720,16384]);parser.add_argument('--domain',type=int,default=80);parser.add_argument('--output',default='tmp/ch20_ros');args=parser.parse_args()
    root=Path(__file__).resolve().parent.parent;out=(root/args.output).resolve();out.mkdir(parents=True,exist_ok=True);results=[]
    env=os.environ.copy();env['ROS_DOMAIN_ID']=str(args.domain)
    for beams in args.beams:
        for backend in ['cpu','cuda']:
            trial=out/f'{backend}_{beams}';trial.mkdir(exist_ok=True)
            with (trial/'launch.log').open('w') as log:
                proc=subprocess.Popen(['ros2','launch','motion2d_bringup','ch20.launch.py',f'backend:={backend}',f'beams:={beams}','rviz:=false'],stdout=log,stderr=subprocess.STDOUT,env=env,cwd=root,start_new_session=True)
                try:subprocess.run([sys.executable,str(root/'scripts/check_ch20.py'),'--backend',backend,'--beams',str(beams),'--output',str(trial)],check=True,env=env,cwd=root)
                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid,signal.SIGINT)
                        try:proc.wait(timeout=12)
                        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
            results.append(json.loads((trial/'summary.json').read_text()))
        a=np.load(out/f'cpu_{beams}'/'scans.npz');b=np.load(out/f'cuda_{beams}'/'scans.npz')
        np.testing.assert_array_equal(a['stamps'],b['stamps']);np.testing.assert_array_equal(np.isnan(a['ranges']),np.isnan(b['ranges']));np.testing.assert_array_equal(np.isinf(a['ranges']),np.isinf(b['ranges']))
        np.testing.assert_allclose(a['ranges'],b['ranges'],atol=2e-5,rtol=0,equal_nan=True)
        valid=np.isfinite(a['ranges']);error=float(np.max(np.abs(a['ranges'][valid]-b['ranges'][valid])))
        for r in results[-2:]:r['paired_max_error_m']=error
        print(f'PASS {beams} beams: 101 aligned noisy scans, max error {error:.3g} m',flush=True)
        (out/'summary.json').write_text(json.dumps(results,indent=2))
    with (out/'summary.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(results[0]));w.writeheader();w.writerows(results)


if __name__=='__main__':main()
