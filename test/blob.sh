# test/blob.sh - the ONE place that says where the blob oracle is.  Sourced (". test/blob.sh", from the repository root) by every test script and tool
# that needs the closed-source i386 object the library was decompiled from; python tools get the same answer from tools/blobpath.py.
#
#   BLOB_O   the blob (an i386 relocatable object, dsplibs.o).  REQUIRED, no default: the blob is not part of any repository.  Use an ABSOLUTE
#            path, e.g.  BLOB_O=/path/to/dsplibs.o sh tools/run_changed.sh ...  (it must not contain spaces).
#
# The blob is only the ORACLE of the differential tests; the library itself neither links nor reads it.
if [ -z "$BLOB_O" ]; then echo "test/blob.sh: set BLOB_O to the absolute path of the blob (dsplibs.o)" >&2; exit 2; fi
case $BLOB_O in /*) ;; *) echo "test/blob.sh: BLOB_O must be an absolute path: $BLOB_O" >&2; exit 2;; esac
[ -f "$BLOB_O" ] || { echo "test/blob.sh: BLOB_O does not exist: $BLOB_O" >&2; exit 2; }
export BLOB_O
