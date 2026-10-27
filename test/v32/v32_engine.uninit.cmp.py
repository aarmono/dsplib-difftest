"""N02: compare which bytes of the heap blocks of V32FP_create stay uninitialised in the blob vs the port (dumps u_<blob|port>_<fill>.txt of v32_engine.uninit.main.c).
Exit 1 if the PORT leaves bytes uninitialised that the blob initialises (or vice versa).  Usage: v32_engine.uninit.cmp.py DIR"""
import re,sys,os
os.chdir(sys.argv[1] if len(sys.argv)>1 else ".")
def load(f):
    d={}; cur=None
    for l in open(f):
        m=re.match(r'role (\d):',l)
        if m: role=int(m.group(1)); continue
        m=re.match(r'A(\d+) size (\d+):',l)
        if m: cur=(role,int(m.group(1))); d[cur]={'size':int(m.group(2)),'b':[]}; continue
        m=re.match(r'([0-9a-f]{4}):(.*)',l)
        if m: d[cur]['b']+= [int(x,16) for x in m.group(2).split()]
    return d
def unin(b):
    A=load('u_%s_0.txt'%b); B=load('u_%s_165.txt'%b); s={}
    for k in A:
        s[k]=set(i for i,(x,y) in enumerate(zip(A[k]['b'],B[k]['b'])) if x!=y)
    return s,A
sb,Ab=unin('blob'); sp,Ap=unin('port')
def rng(l):
    r=[]
    for i in l:
        if r and i==r[-1][1]+1: r[-1][1]=i
        else: r.append([i,i])
    return ' '.join('%x-%x'%(a,b) if a!=b else '%x'%a for a,b in r)
bad=0
for k in sorted(sb):
    only_p=sorted(sp.get(k,set())-sb[k]); only_b=sorted(sb[k]-sp.get(k,set()))
    if only_p or only_b:
        bad=1; print(k,'size',Ab[k]['size'],'uninit only in PORT:',rng(only_p),'| only in BLOB:',rng(only_b))
print('blocks compared: %d (blob) %d (port); blocks with a blob-vs-port uninitialised-byte difference: %s' % (len(sb), len(sp), 'SOME' if bad else 'none'))
sys.exit(bad)
