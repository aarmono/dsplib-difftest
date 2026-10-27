"""blobpath.py - where the blob oracle (slmodemd's closed-source i386 dsplibs.o) is, for the python tools.

The single definition is `BLOB_O_DEFAULT` in test/blob.sh (which the shell scripts source); the environment variable BLOB_O overrides it, exactly as
there.  Import:  sys.path.insert(0, <dsplibs-port>/tools); from blobpath import BLOB_O.  A relative default is resolved against the dsplibs-port
directory (the shell scripts resolve it against their working directory, which is that directory too); a relative BLOB_O from the environment is
used as given (relative to the working directory), as in the shell.
"""
import os
import re

PORTROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))     # slmodemd/dsplibs-port


def _default():
    with open(os.path.join(PORTROOT, 'test', 'blob.sh')) as f:
        m = re.search(r'^BLOB_O_DEFAULT=(\S+)\s*$', f.read(), re.M)
    if not m:
        raise RuntimeError('test/blob.sh has no BLOB_O_DEFAULT= line')
    d = m.group(1)
    return d if os.path.isabs(d) else os.path.normpath(os.path.join(PORTROOT, d))


BLOB_O = os.environ.get('BLOB_O') or _default()
