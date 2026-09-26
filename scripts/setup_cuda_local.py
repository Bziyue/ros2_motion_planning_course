#!/usr/bin/env python3
"""Fetch a pinned minimal CUDA 13.2.1 toolchain into tmp; no driver/system changes.

Linux x86_64 only. Official NVIDIA archives keep their component licenses.
CPU-only learners do not need this script. Downloads are SHA-256 checked before
extraction; the default output is ignored by Git.
"""
import argparse
import hashlib
import filecmp
import platform
from pathlib import Path
import shutil
import tarfile
import urllib.request

BASE = 'https://developer.download.nvidia.com/compute/cuda/redist/'
COMPONENTS = {'cuda_nvcc': {'relative_path': 'cuda_nvcc/linux-x86_64/cuda_nvcc-linux-x86_64-13.2.78-archive.tar.xz',
               'sha256': '1dc73b03f1d74081866986ca406f7b5981d14306ccbb03127ba45401f36e1862',
               'md5': '51386c95804af58783dcba465ae0cb10',
               'size': '31106340'},
 'cuda_cudart': {'relative_path': 'cuda_cudart/linux-x86_64/cuda_cudart-linux-x86_64-13.2.75-archive.tar.xz',
                 'sha256': '9502ab2c7824e5ef3b554d290b9f564c5026994f469b99c72451a53578f386b9',
                 'md5': '3d67f208d3904c1d27eb6258a4667f1a',
                 'size': '1545956'},
 'cuda_cccl': {'relative_path': 'cuda_cccl/linux-x86_64/cuda_cccl-linux-x86_64-13.2.75-archive.tar.xz',
               'sha256': '1801f304d92085a327ab46db58264d7f7f48cce80f5825637c405c3a2410dadd',
               'md5': '8f96687e430ec4414e96e64837f0614f',
               'size': '1213540'},
 'cuda_crt': {'relative_path': 'cuda_crt/linux-x86_64/cuda_crt-linux-x86_64-13.2.78-archive.tar.xz',
              'sha256': '17e731ba749765c2e2e325e3bffecee94226c866cf59f13ba3792aa4f2b1bb31',
              'md5': '0ad56a3a66495a8e8665e96637a23893',
              'size': '80256'},
 'libnvvm': {'relative_path': 'libnvvm/linux-x86_64/libnvvm-linux-x86_64-13.2.78-archive.tar.xz',
             'sha256': '9c665ebec40d0dec4df1858d5a0129972b00351dd674e783d30405cf712d925d',
             'md5': '83dfc5eeffc3f156196e733fb47131cb',
             'size': '45456248'}}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='tmp/cuda-13.2.1');args=parser.parse_args()
    if platform.system()!='Linux' or platform.machine()!='x86_64':raise RuntimeError('These pinned archives target Linux x86_64')
    root=Path(args.output).resolve();root.mkdir(parents=True,exist_ok=True);dest=root/'toolkit';dest.mkdir(exist_ok=True)
    for name,item in COMPONENTS.items():
        archive=root/Path(item['relative_path']).name
        if not archive.exists():urllib.request.urlretrieve(BASE+item['relative_path'],archive)
        if hashlib.sha256(archive.read_bytes()).hexdigest()!=item['sha256']:raise RuntimeError(f'Checksum mismatch: {archive}; remove that download before retrying')
        with tarfile.open(archive) as tar:tar.extractall(root/'parts',filter='data')
        part=root/'parts'/archive.name.removesuffix('.tar.xz')
        for source in part.rglob('*'):
            if source.name=='LICENSE':continue
            target=dest/source.relative_to(part)
            if source.is_symlink():
                link=source.readlink()
                if target.is_symlink() and target.readlink()==link:continue
                if target.exists() or target.is_symlink():target.unlink()
                target.symlink_to(link)
            elif source.is_dir():target.mkdir(parents=True,exist_ok=True)
            elif not target.exists() or not filecmp.cmp(source,target,shallow=False):
                target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
        licenses=dest/'licenses';licenses.mkdir(exist_ok=True);shutil.copyfile(part/'LICENSE',licenses/(name+'.txt'))
        print(f'{name}: verified and extracted',flush=True)
    if not (dest/'lib64').exists():(dest/'lib64').symlink_to('lib',target_is_directory=True)
    print(f'CUDA compiler: {dest / "bin/nvcc"}')


if __name__=='__main__':main()
