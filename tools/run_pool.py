#!/usr/bin/env python3
"""Parallel job pool of tools/run_changed.sh -j N  (not meant to be called by hand).

usage: run_pool.py -j N --root DIR(tree) --sandbox DIR --logs DIR JOBFILE

JOBFILE: one tab-separated line per job: id weight excl key script args g nopin seeds
  weight = cores the job uses (the scheduler never runs more than N cores' worth at once), excl=1 -> exclusive lane: runs alone, after the pool drained.
  cross/z01_hybrid is chained behind vpcm/dp_vpcm (the old z01; same sandbox) when both are in the file.
Every job runs in its own private tree (tools/job_sandbox.sh) with TMPDIR inside it; its output goes to LOGS/<fid>.log, its exit code is LOGS/<fid>.rc,
wall / cpu seconds are appended to LOGS/timing.tsv (fid = the file-safe form of the job id: every '/' becomes '__', TESTREORG stage 10).  The stamp (test/.stamps/<fid> = key) is written HERE, by this single process, atomically (tmp + rename),
only when the job's exit code is 0.  Exit status 0 always (the caller counts the .rc files).
"""
import os, sys, subprocess, time, signal, shutil, argparse

ap = argparse.ArgumentParser()
ap.add_argument('-j', type=int, default=1)
ap.add_argument('--root', required=True)
ap.add_argument('--sandbox', required=True)
ap.add_argument('--logs', required=True)
ap.add_argument('jobfile')
o = ap.parse_args()
N = max(1, o.j)

def fid(i): return i.replace('/', '__')   # file-safe form of a job id (ids are planned names with '/' and '.'; no id contains '__': tools/jobs_check.sh)
Z01, Z01H = 'vpcm/dp_vpcm', 'cross/z01_hybrid'

jobs = []
for ln in open(o.jobfile):
    ln = ln.rstrip('\n')
    if not ln: continue
    f = (ln.split('\t') + [''] * 9)[:9]
    jobs.append(dict(id=f[0], w=max(1, int(f[1] or 1)), excl=f[2] == '1', key=f[3], script=f[4], args=f[5], g=f[6], nopin=f[7], seeds=f[8].split(), est=0))
# longest-first order helps the makespan: the hint is the previous wall time in the timing history (jobclass.txt column est, filled by the caller via the 10th field)
tasks = []   # a task = list of jobs run in sequence in ONE sandbox
byid = {j['id']: j for j in jobs}
for j in jobs:
    if j['id'] == Z01H and Z01 in byid: continue
    t = [j]
    if j['id'] == Z01 and Z01H in byid: t.append(byid[Z01H])
    tasks.append(t)
def tw(t): return min(N, max(j['w'] for j in t))
pool = [t for t in tasks if not any(j['excl'] for j in t)]
excl = [t for t in tasks if any(j['excl'] for j in t)]

os.makedirs(o.logs, exist_ok=True); os.makedirs(o.sandbox, exist_ok=True)
timing = open(os.path.join(o.logs, 'timing.tsv'), 'a')
running = {}   # pid -> (task, ji, t0, procinfo)
stop = False

def write_stamp(j):
    p = os.path.join(o.root, 'test', '.stamps', fid(j['id']))
    os.makedirs(os.path.dirname(p), exist_ok=True)
    tmp = p + '.tmp.%d' % os.getpid()
    with open(tmp, 'w') as f: f.write(j['key'] + '\n')
    os.replace(tmp, p)

def drop_stamp(j):
    try: os.unlink(os.path.join(o.root, 'test', '.stamps', fid(j['id'])))
    except OSError: pass

def start(task, idx):
    j = task[idx]
    log = open(os.path.join(o.logs, fid(j['id']) + '.log'), 'wb')
    sbtree = os.path.join(o.sandbox, fid(task[0]['id']), 'tree')
    env = dict(os.environ, JOB_ID=j['id'], JOB_SCRIPT=j['script'], JOB_ARGS=j['args'], JOB_G=j['g'], NOPIN=j['nopin'],
               TMPDIR=os.path.join(o.sandbox, fid(task[0]['id']), 'tmp'))
    if idx == 0:
        cmd = 'sh tools/job_sandbox.sh "$SBROOT" "$SBID" $SEEDS >/dev/null && cd "$SBROOT/$SBID/tree" && exec sh tools/run_job.sh'
        env.update(SBROOT=o.sandbox, SBID=fid(task[0]['id']), SEEDS=' '.join(j['seeds']))
        cwd = o.root
    else:
        cmd = 'exec sh tools/run_job.sh'; cwd = sbtree
    p = subprocess.Popen(['sh', '-c', cmd], cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return p.pid, (task, idx, time.time(), log, p)   # keep p alive: a garbage-collected Popen would reap the child behind our back (subprocess._cleanup)

def finish(pid, status, ru):
    task, idx, t0, log, _p = running.pop(pid)
    _p.returncode = 0   # reaped here, not by subprocess
    log.close()
    j = task[idx]
    rc = os.waitstatus_to_exitcode(status)
    wall = time.time() - t0; cpu = ru.ru_utime + ru.ru_stime
    open(os.path.join(o.logs, fid(j['id']) + '.rc'), 'w').write('%d\n' % rc)
    timing.write('%s\t%.1f\t%.1f\t%.2f\t%d\n' % (j['id'], wall, cpu, cpu / wall if wall > 0 else 0, j['w'])); timing.flush()
    if rc == 0: write_stamp(j)
    else: drop_stamp(j)
    sys.stderr.write('[%s] %s rc=%d wall=%.0fs cpu=%.0fs\n' % ('ok' if rc == 0 else 'FAIL', j['id'], rc, wall, cpu)); sys.stderr.flush()
    if rc == 0 and idx + 1 < len(task):
        npid, info = start(task, idx + 1); running[npid] = info; return
    if rc != 0:
        for k in task[idx + 1:]:   # chained job not run: say so
            open(os.path.join(o.logs, fid(k['id']) + '.log'), 'w').write('%s: FAIL (not run: %s failed)\n' % (k['id'], j['id']))
            open(os.path.join(o.logs, fid(k['id']) + '.rc'), 'w').write('1\n'); drop_stamp(k)
    if rc == 0 and not os.environ.get('KEEP_SANDBOX'):
        shutil.rmtree(os.path.join(o.sandbox, fid(task[0]['id'])), ignore_errors=True)

def killall(*_):
    for pid in list(running):
        try: os.killpg(pid, signal.SIGTERM)
        except OSError: pass
    time.sleep(1)
    for pid in list(running):
        try: os.killpg(pid, signal.SIGKILL)
        except OSError: pass
    sys.exit(130)
signal.signal(signal.SIGINT, killall); signal.signal(signal.SIGTERM, killall)

def used(): return sum(tw(r[0]) for r in running.values())
def reap(block):
    pid, status, ru = os.wait4(-1, 0 if block else os.WNOHANG)
    if pid and pid in running: finish(pid, status, ru); return True
    return False

def run_queue(q, serial):
    q = list(q)
    while q or running:
        while q and (not running if serial else (used() + tw(q[0]) <= N or not running)):
            t = q.pop(0); pid, info = start(t, 0); running[pid] = info
            if serial: break
        if running: reap(True)

# longest first, when the est hint exists (jobclass.txt, passed through env POOL_EST: "id=seconds,...")
est = dict(kv.split('=') for kv in os.environ.get('POOL_EST', '').split(',') if '=' in kv)
pool.sort(key=lambda t: -sum(float(est.get(j['id'], 0)) for j in t))
run_queue(pool, False)
run_queue(excl, True)
