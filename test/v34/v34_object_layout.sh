#!/bin/sh
# X3: layout agreement of the two spellings of the V.34 module object (C V34Obj / C++ tagV34Object) at -m32 and -m64 (see test/x3_v34layout.py).  No blob involved.
set -e
cd "$(dirname "$0")/../.."
python3 test/v34/v34_object_layout.check.py
