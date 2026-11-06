"""E6: closed-loop kinematic simulation of the singularity guard, old (cut on q') vs new (cut on the twist).

Run from the package root with ROS sourced:
    python3 thesis/planning/analysis/E6_singularity/e6_guard_sim.py

The loop copies update_motion(): feedforward + capped gain x error, guard, 2-cycle delay, pinv, joint cap.
No Servo, command filter or arm dynamics, so use it to compare guards, not to predict the arm.
"""
import sys,time,importlib.util,math,numpy as np
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0,'.')
from macro_micro_robot.ur16e_common import SingularityGuard, jacobian_derivatives
spec=importlib.util.spec_from_file_location("si","scripts/singularity_input.py"); si=importlib.util.module_from_spec(spec); spec.loader.exec_module(si)
def pose(q):
    T=np.eye(4)
    for i in range(6):
        c,s,ca,sa=math.cos(q[i]),math.sin(q[i]),math.cos(si.ALPHA[i]),math.sin(si.ALPHA[i])
        T=T@np.array([[c,-s*ca,s*sa,si.A[i]*c],[s,c*ca,-c*sa,si.A[i]*s],[0,sa,ca,si.D[i]],[0,0,0,1]])
    return T[:3,3],Rot.from_matrix(T[:3,:3])
Jf=lambda q: si.jacobian(q)[0]
class JointSpaceGuard(SingularityGuard):
    """The guard before 2026-11-06."""
    def __call__(self, twist, positions):
        """Filtered copy of twist (6: linear, angular; same frame and point as the Jacobian)."""
        positions = np.asarray(positions, dtype=float)
        jacobian = self.jacobian_fn(positions)
        u, s, vt = np.linalg.svd(jacobian)
        self.condition = s[0] / s[-1] if s[-1] > 0.0 else np.inf
        self.limiting, self.escaping, self.twist_gradient = 0.0, False, None
        if self.condition < self.slow:
            return twist

        # Damped least squares: damping from 0 at c = slow to `damping` at s_min = 0
        threshold = s[0] / self.slow
        damping_sq = self.damping ** 2 * (1.0 - (s[-1] / threshold) ** 2)
        joint_velocity = vt.T @ ((s / (s ** 2 + damping_sq)) * (u.T @ twist))

        # Joint-space direction away from the singularity: the gradient of
        # m = (sum s_i^-2)^-1/2 (ds_i = u_i^T dJ v_i). m ~ s_min, but stays smooth where
        # the two smallest s_i cross (two singularities at once, e.g. elbow + shoulder);
        # the gradient of s_min itself flips between them every cycle there.
        derivatives = jacobian_derivatives(jacobian)
        ds = np.diagonal(u.T @ derivatives @ vt.T, axis1=1, axis2=2)     # [k, i] = ds_i / dq_k
        safe = np.maximum(s, 1e-12)
        gradient = self.measure(s) ** 3 * (ds @ safe ** -3)
        norm_sq = gradient @ gradient
        if not norm_sq > 1e-12 and self.condition > self.stop:
            # Exactly on a singularity (e.g. mock hardware's all-zero start) m is like |x|
            # and the analytic gradient can cancel to 0: step each joint one way instead.
            # Scored by log(s_1 ... s_6), which rises when any one of two singularities
            # at once is relieved (m stays 0 until both are).
            def score(values):
                return np.sum(np.log(np.maximum(values, 1e-12)))
            base = score(s)
            gradient = np.array([
                score(np.linalg.svd(self.jacobian_fn(positions + self.PROBE_STEP * step), compute_uv=False))
                - base for step in np.eye(len(positions))]) / self.PROBE_STEP
            norm_sq = gradient @ gradient
        if norm_sq > 1e-12:
            # (J_dls^+)^T g: q' = J_dls^+ v, so dm/dt = g . q' = this . v
            self.twist_gradient = u @ ((s / (s ** 2 + damping_sq)) * (vt @ gradient))
            self.limiting = min(1.0, (self.condition - self.slow) / (self.stop - self.slow))
            towards = gradient @ joint_velocity
            if towards < 0.0:
                joint_velocity -= self.limiting * towards / norm_sq * gradient
            if self.condition > self.stop:
                speed = min(self.escape_speed, self.escape_gain * (self.condition / self.stop - 1.0))
                joint_velocity += speed * gradient / np.sqrt(norm_sq)
                self.escaping = speed > 0.0
        return jacobian @ joint_velocity

def run(cls,q0,target,T,dt=0.004,delay=3,qcap=2.0,guardon=True):
    G=cls(Jf,30,60,0.05,1.0,0.2); q=np.array(q0,float); p0,R0=pose(q)
    log=[];buf=[np.zeros(6)]*delay; pprev=target(0)
    for k in range(int(T/dt)):
        t=k*dt; tp=target(t); v_ff=(target(t+dt)-tp)/dt
        p,R=pose(q); e=tp-p; lin=v_ff+(lambda x:x*min(1,.05/max(np.linalg.norm(x),1e-12)))(8*e)
        rv=(R0*R.inv()).as_rotvec(); ang=(lambda x:x*min(1,.5/max(np.linalg.norm(x),1e-12)))(8*rv)
        cmd=np.r_[lin,ang]
        out=G(cmd,q) if guardon else cmd
        out=np.r_[out[:3]*min(1,1.2/max(np.linalg.norm(out[:3]),1e-12)),out[3:]*min(1,3/max(np.linalg.norm(out[3:]),1e-12))]
        buf.append(out); o=buf.pop(0)
        qd=np.linalg.lstsq(Jf(q),o,rcond=1e-4)[0]
        f=min(1,qcap/max(abs(qd).max(),1e-9)); q=q+qd*f*dt
        log.append((t,np.linalg.cond(Jf(q)),*p,np.degrees(abs(qd*f).max()),np.linalg.norm(e),np.linalg.norm(rv)))
    return np.array(log)
def stroke(p0,d,L=.22,move=12,hold=3):
    def f(t):
        t2=t-1
        if t2<0: s=0
        elif t2<move: s=si.blend(t2/move)
        elif t2<move+hold: s=1
        else: s=si.blend(max(0,1-(t2-move-hold)/move))
        return p0+L*s*d
    return f
rng=np.random.default_rng(11); N=30
cases=[]
while len(cases)<N:
    kind='wrist' if len(cases)%2==0 else 'elbow'
    q=rng.uniform(np.radians([-120,-140,40,-200,40,-170]),np.radians([120,-30,150,0,140,170]))
    q[2]*=rng.choice([-1,1]); q[4]*=rng.choice([-1,1])
    if kind=='wrist': q[4]=np.radians(rng.uniform(12,25))*rng.choice([-1,1])
    else: q[2]=np.radians(rng.uniform(14,30))*rng.choice([-1,1])
    c=np.linalg.cond(Jf(q)); p,_=pose(q)
    if not 8<c<30 or p[2]<.2 or np.hypot(p[0],p[1])<.3: continue
    row=np.linalg.inv(Jf(q))[4 if kind=='wrist' else 2,:3]; d=-np.sign(q[4] if kind=='wrist' else q[2])*row/np.linalg.norm(row)
    d2=np.cross(d,rng.normal(size=3)); d2/=np.linalg.norm(d2); cases.append((q,d,d2,kind))
t0=time.time(); res={}
for name,cl in (('old',JointSpaceGuard),('new',SingularityGuard)):
    R=[]
    for q,d,d2,k in cases:
        p0,_=pose(q); L=.30
        def tgt(t,p0=p0,d=d,d2=d2):
            a=si.blend(np.clip((t-.5)/6,0,1)); b=si.blend(np.clip((t-6.5)/4,0,1)); c=si.blend(np.clip((t-11)/6,0,1))
            return p0+(1-c)*(L*a*d+.08*b*d2)
        o=run(cl,q,tgt,19,dt=.008,delay=2)
        R.append((o[:,1].max(),o[:,5].max(),o[:,6].max()*1000,np.degrees(o[:,7].max()),o[-1,6]*1000,np.degrees(o[-1,7]),k,o[:,4].min()-p0[2]))
    res[name]=R
print('secs',round(time.time()-t0))
for name,R in res.items():
  for k in ('wrist','elbow'):
    a=np.array([r[:6]+(r[7],) for r in R if r[6]==k]); h=a[:,0]>29
    print(f"{name:5s}{k:6s} n {len(a)} limit {h.sum()} peak_c {a[:,0].max():.1f} qd med/max {np.median(a[:,1]):.0f}/{a[:,1].max():.0f} err p50/p95/worst {np.median(a[:,2]):.0f}/{np.percentile(a[:,2],95):.0f}/{a[:,2].max():.0f} rot worst {a[:,3].max():.1f} end err worst {a[:,4].max():.1f} (>5mm {(a[:,4]>5).sum()}) z drop worst {-a[:,6].min()*1000:.0f}")
# start past the limit, stylus still
for name,cl in (('old',JointSpaceGuard),('new',SingularityGuard)):
    q=np.radians([-3,-70,70,-180,-1.0,0]); p0,_=pose(q); o=run(cl,q,lambda t:p0,10,dt=.008,delay=2)
    print(name,'start c',round(o[0,1]),'end c',round(o[-1,1]),'tool moved mm',round(o[:,6].max()*1000,1),'qd max',round(o[:,5].max(),1),'rot',round(np.degrees(o[:,7].max()),1))
