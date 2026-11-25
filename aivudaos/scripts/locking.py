"""Cooperative per-UID locks shared by FAST-LIO instances."""
import fcntl
import os
from pathlib import Path
import stat


def acquire(names):
    """Take exclusive nonblocking locks and keep descriptors across exec."""
    uid = os.getuid()
    root = Path('/tmp') / f'fastlio-px4-{uid}'
    try:
        root.mkdir(mode=0o700)
    except FileExistsError:
        pass
    info = root.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != uid or (stat.S_IMODE(info.st_mode) != 0o700):
        raise RuntimeError(f'Unsafe lock directory: {root}; must be owned by uid {uid} with mode 0700')
    descriptors = []
    try:
        for name in names:
            fd = os.open(root / name, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            descriptors.append(fd)
            meta = os.fstat(fd)
            if not stat.S_ISREG(meta.st_mode) or meta.st_uid != uid or meta.st_nlink != 1:
                raise RuntimeError(f'Unsafe lock file: {root / name}')
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.set_inheritable(fd, True)
    except Exception:
        for fd in descriptors: os.close(fd)
        raise
    return descriptors
