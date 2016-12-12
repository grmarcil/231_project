include("mpc_functions.jl")
include("path.jl")
#= using Gallium # julia debugger =#
#= Gallium.breakpoint(generateRef) =#

#Car dimensions
lr  = 1.738
dr = 0.712
lf = 1.738
df = 0.712
width = 1.85

#paramters not to touch
L  = lr+lf
Lf = lf+df
Lr = lr+dr
car_size = [lr,Lf,Lr,width]

# Import path planned in Matlab
# First run Julia_setup.m
path = importPath()

N = 5 # MPC horizon
Lsim = 150 # Simulation duration
dt=0.1

# Simulation Data
v = 4.5; # m/s constant velocity
# States [x;y;psi;v]
z0 = [path.x[1]; path.y[1]; path.psi[1]; v];

zmax=[35;35;2*pi;10]
zmin=[-35;-35;-2*pi;-10]
umax=[0.6;1.5*dt]

# Num of states and inputs
nz = 4;
nu = 2;

# Closed Loop MPC
u_cl, z_cl = simulateCarMPC(car_size,nz,nu,N,Lsim,dt,z0,path,zmin,zmax,umax)
writecsv("u_cl.csv", u_cl)
writecsv("z_cl.csv", z_cl)
