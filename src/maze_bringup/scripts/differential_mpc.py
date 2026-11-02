"""OSQP LTV-MPC: differential kinematics, measured actuator states and wheel limits.

Adapted algorithmically from Zythyra/ifly2_2026 my_planner C2/C3/C4,
commit 4c876c9e551c4ad161344b9efa5a625f48a981e3. No lateral control or drift.
"""
import math
import time

import numpy as np
import osqp
from scipy import sparse

from polyline_core import Frame2D
from motion_reference import arc_length, project



def limit_wheel_command(command, previous, dt=.05, acceleration=.9, speed=.95):
    def wheels(u):return np.array([u[0]-.08*u[1],u[0]+.08*u[1]])
    prior=wheels(previous);desired=np.clip(wheels(command),-speed,speed)
    limited=prior+np.clip(desired-prior,-acceleration*dt,acceleration*dt)
    return float(np.mean(limited)),float((limited[1]-limited[0])/.16)

def model(state, command, dt=.05, tau_v=.06, tau_w=.09):
    x,y,theta,v,w=state;vc,wc=command
    av,aw=math.exp(-dt/tau_v),math.exp(-dt/tau_w)
    vn=av*v+(1-av)*vc;wn=aw*w+(1-aw)*wc
    vm,wm=.5*(v+vn),.5*(w+wn)
    middle=theta+.5*wm*dt
    return np.array([x+vm*math.cos(middle)*dt,y+vm*math.sin(middle)*dt,
                     theta+wm*dt,vn,wn])


def linearize(state, command, dt=.05):
    theta,v,w=state[2:];vc,wc=command
    av,aw=math.exp(-dt/.06),math.exp(-dt/.09);bv,bw=1-av,1-aw
    vm=.5*((1+av)*v+bv*vc);wm=.5*((1+aw)*w+bw*wc)
    c,s=math.cos(theta+.5*wm*dt),math.sin(theta+.5*wm*dt)
    A=np.eye(5);B=np.zeros((5,2))
    A[0,2]=-vm*s*dt;A[1,2]=vm*c*dt
    A[0,3]=.5*(1+av)*c*dt;A[1,3]=.5*(1+av)*s*dt
    A[0,4]=-vm*s*dt*.25*(1+aw)*dt
    A[1,4]=vm*c*dt*.25*(1+aw)*dt
    A[2,4]=.5*(1+aw)*dt
    A[3,3]=av;A[4,4]=aw
    B[0,0]=.5*bv*c*dt;B[1,0]=.5*bv*s*dt
    B[0,1]=-vm*s*dt*.25*bw*dt;B[1,1]=vm*c*dt*.25*bw*dt
    B[2,1]=.5*bw*dt;B[3,0]=bv;B[4,1]=bw
    return A,B,model(state,command,dt)-A@state-B@command


class DifferentialMPC:
    def __init__(self,horizon=20,dt=.05,max_speed=.53,max_omega=4.3,
                 wheel_speed=.95,wheel_acceleration=.9):
        self.N=int(horizon);self.dt=float(dt);self.max_speed=max_speed
        self.max_omega=max_omega;self.wheel_speed=wheel_speed
        self.wheel_acceleration=wheel_acceleration
        self.previous_controls=None;self.last_call=None
        self.successes=self.failures=0
        self.accepted=0

    def reset(self):
        self.previous_controls=None;self.last_call=None

    def solve(self, reference, measured, previous_command, speed_caps=None, widths=None, control_dt=None):
        started=time.monotonic();N=self.N;nx=5;nu=2;dt=self.dt
        reference=np.asarray(reference,dtype=float)
        if reference.shape!=(N+1,nx):raise ValueError('MPC reference must be (horizon+1, 5)')
        ns=nx*(N+1);nc=2*N;nv=ns+nc+N
        P=np.eye(nv)*1e-7;q=np.zeros(nv)
        previous=np.asarray(previous_command,dtype=float)
        uref=reference[:-1,3:5].copy()
        uref[:,0]+=.06*np.diff(reference[:,3])/dt
        uref[:,1]+=.09*np.diff(reference[:,4])/dt
        for k,ref in enumerate(reference):
            c,s=math.cos(ref[2]),math.sin(ref[2]);rotation=np.array([[c,s],[-s,c]])
            Q=np.diag([3.,110.,10.,5.,.6 if abs(ref[4])>.1 else 2.])
            Q[:2,:2]=rotation.T@np.diag([3.,110.])@rotation
            if k==N:Q*=2.
            sl=slice(k*nx,(k+1)*nx)
            P[sl,sl]+=2*Q;q[sl]-=2*Q@ref
        D=np.zeros((nc,nv));R=np.tile([4.,.6],N)
        for k in range(N):
            u=ns+2*k;sl=slice(u,u+2)
            weights=np.array([.3,.12])
            P[sl,sl]+=np.diag(2*weights);q[sl]-=2*weights*uref[k]
            q[u]-=.2  # restrained forward-progress preference
            D[2*k:2*k+2,sl]=np.eye(2)
            if k:D[2*k:2*k+2,u-2:u]=-np.eye(2)
            P[ns+nc+k,ns+nc+k]+=2*20000.
        initial_rate=np.zeros(nc);initial_rate[:2]=previous
        P+=2*(D.T*R)@D;q-=2*D.T@(R*initial_rate)
        rows=[];lower=[];upper=[]
        def constraint(values,lo,hi):
            rows.append(values);lower.append(lo);upper.append(hi)
        for j in range(nx):
            row=np.zeros(nv);row[j]=1;constraint(row,measured[j],measured[j])
        for k in range(N):
            A,B,affine=linearize(reference[k],uref[k],dt)
            for j in range(nx):
                row=np.zeros(nv);row[nx*(k+1)+j]=1
                row[nx*k:nx*(k+1)]-=A[j];row[ns+2*k:ns+2*k+2]-=B[j]
                constraint(row,affine[j],affine[j])
            # Input bounds and differential wheel velocities (m/s at rim).
            cap=self.max_speed if speed_caps is None else max(float(speed_caps[k]),previous[0]-.7*dt*(k+1))
            for j,lo,hi in [(0,0.,min(self.max_speed,cap)),(1,-self.max_omega,self.max_omega)]:
                row=np.zeros(nv);row[ns+2*k+j]=1;constraint(row,lo,hi)
            for sign in (-1,1):
                row=np.zeros(nv);row[ns+2*k]=1;row[ns+2*k+1]=sign*.08
                constraint(row,-self.wheel_speed,self.wheel_speed)
            step=dt if k else (dt if control_dt is None else float(np.clip(control_dt,.02,.1)))
            for j,limit in [(0,.9),(1,7.)]:
                row=D[2*k+j].copy();offset=previous[j] if k==0 else 0.
                constraint(row,offset-limit*step,offset+limit*step)
            for sign in (-1,1):
                row=D[2*k]+sign*.08*D[2*k+1]
                offset=previous[0]+sign*.08*previous[1] if k==0 else 0.
                constraint(row,offset-self.wheel_acceleration*step,offset+self.wheel_acceleration*step)
            # Soft normal corridor for recovery from existing lateral error.
            normal=np.array([-math.sin(reference[k+1,2]),math.cos(reference[k+1,2])])
            half=.10 if widths is None else float(widths[k+1])
            for side in (-1,1):
                row=np.zeros(nv);row[(k+1)*nx:(k+1)*nx+2]=side*normal
                row[ns+nc+k]=-1
                constraint(row,-np.inf,side*float(normal@reference[k+1,:2])+half)
            row=np.zeros(nv);row[ns+nc+k]=1;constraint(row,0.,.18)
        matrix=sparse.csc_matrix(np.asarray(rows));lo=np.array(lower);hi=np.array(upper)
        solver=osqp.OSQP()
        solver.setup(P=sparse.triu(sparse.csc_matrix(P),format='csc'),q=q,A=matrix,l=lo,u=hi,
                     verbose=False,warm_starting=True,polishing=False,max_iter=1500,
                     eps_abs=5e-4,eps_rel=5e-4,time_limit=.025,check_termination=10)
        if self.previous_controls is not None:
            controls=np.vstack((self.previous_controls[1:],self.previous_controls[-1]))
            warm=np.zeros(nv);state=np.asarray(measured).copy();warm[:nx]=state
            for k in range(N):
                warm[ns+2*k:ns+2*k+2]=controls[k]
                state=model(state,controls[k],dt);warm[nx*(k+1):nx*(k+2)]=state
            solver.warm_start(x=warm)
        result=solver.solve(raise_error=False)
        report=dict(solver_status=result.info.status,iterations=int(result.info.iter),
                    solve_ms=float(result.info.solve_time*1000),qp_ms=(time.monotonic()-started)*1000)
        if result.info.status_val not in (1,2) or result.x is None or not np.all(np.isfinite(result.x)):
            self.failures+=1;report.update(mpc_failures=self.failures);return None,None,report
        residual=matrix@result.x
        violation=float(max(np.max(np.maximum(lo-residual,0)),np.max(np.maximum(residual-hi,0))))
        report['max_constraint_violation']=violation
        if violation>.003:
            self.failures+=1;report.update(solver_status='REJECT_CONSTRAINT_RESIDUAL',mpc_failures=self.failures)
            return None,None,report
        controls=result.x[ns:ns+nc].reshape(N,2)
        controls[:,0]=np.clip(controls[:,0],0.,self.max_speed)
        controls[:,1]=np.clip(controls[:,1],-self.max_omega,self.max_omega)
        self.previous_controls=controls.copy();self.successes+=1
        states=[np.asarray(measured).copy()]
        for command in controls:states.append(model(states[-1],command,dt))
        report.update(qp_successes=self.successes,mpc_failures=self.failures)
        return controls[0],np.array(states),report

    def compute(self, reference, map_position, odom_position, odom_heading, map_from_odom,
                measured_speed, measured_omega, previous_command, grid=None):
        started=time.monotonic()
        control_dt=self.dt if self.last_call is None else started-self.last_call
        self.last_call=started
        horizon=reference.horizon(map_position,measured_speed,self.N,self.dt)
        s0,_=project(reference.xy,np.asarray(map_position))
        caps=np.interp(s0+arc_length(horizon[:,:2]),reference.s,reference.speed)
        body=Frame2D(np.asarray(odom_position),odom_heading)
        local=horizon.copy()
        for i,point in enumerate(horizon[:,:2]):local[i,:2]=body.inverse(map_from_odom.inverse(point))
        local[:,2]=np.unwrap(horizon[:,2]-map_from_odom.yaw-odom_heading)
        local[:,2]-=round(local[0,2]/(2*math.pi))*2*math.pi
        measured=np.array([0.,0.,0.,measured_speed,measured_omega])
        measured=model(measured,previous_command,.03)
        widths=np.full(self.N+1,.10)
        if grid is not None:
            for i,point in enumerate(horizon[:,:2]):
                cell=grid.cell(point)
                if grid.allowed(cell):widths[i]=max(.035,min(.12,float(grid.distance[cell])-.14))
        command,states,report=self.solve(local,measured,previous_command,caps,widths,control_dt)
        ref_path=[p.copy() for p in horizon[:,:2]]
        if command is None:
            report.update(mpc_accepted=self.accepted,mpc_failures=self.failures)
            return None,[],ref_path,report
        predicted=[map_from_odom.apply(body.apply(point)) for point in states[:,:2]]
        if grid is not None:
            for a,b in zip(predicted[:-1],predicted[1:]):
                for point in np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/.012)+1)):
                    r,c=grid.cell(point)
                    if not (0<=r<grid.cells.shape[0] and 0<=c<grid.cells.shape[1]
                            and 0<=grid.cells[r,c]<65 and grid.distance[r,c]>=.14):
                        report['solver_status']='REJECT_NONLINEAR_ROLLOUT_CLEARANCE'
                        self.previous_controls=None;self.failures+=1
                        report.update(mpc_accepted=self.accepted,mpc_failures=self.failures)
                        return None,predicted,ref_path,report
        self.accepted+=1
        report.update(mpc_accepted=self.accepted,mpc_failures=self.failures)
        report['mpc_total_ms']=(time.monotonic()-started)*1000
        return tuple(float(v) for v in command),predicted,ref_path,report
