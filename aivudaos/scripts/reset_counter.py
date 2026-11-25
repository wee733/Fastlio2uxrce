#!/usr/bin/env python3
"""Advance the PX4 reset counter for the current estimator."""
import fcntl
import os
from pathlib import Path


def next_counter(directory):
    root = Path(directory)
    if not root.is_absolute():
        raise ValueError('The shared reset counter directory must be an absolute path')
    try:
        root.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise ValueError(f'Cannot create shared counter directory {root}; configure a writable directory shared by FAST-LIO and mocap') from exc
    counter = root / 'reset_counter'
    with (root / 'counter.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if counter.is_symlink():
            raise ValueError(f'Reset counter must not be a symbolic link: {counter}')
        if counter.exists():
            try:
                old = int(counter.read_text().strip())
            except ValueError as exc:
                raise ValueError(f'Invalid reset counter in {counter}') from exc
        else:
            # The legacy service assumes a previous manual run used counter 1.
            old = 1
        if not 0 <= old <= 255:
            raise ValueError('Stored reset_counter must be within uint8 range')
        new = (old + 1) % 256
        temp = root / 'reset_counter.new'
        with temp.open('w') as out:
            out.write(f'{new}\n')
            out.flush()
            os.fsync(out.fileno())
        temp.replace(counter)
        fd = os.open(root, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
        return new
