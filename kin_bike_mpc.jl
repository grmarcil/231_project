using HDF5
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
lw = 3;
ll = 30;
# y coordinate for stop sign, approaching from below
y_stop = -lw;

N = 5 # MPC horizon
Lsim = 150 # Simulation duration
dt=0.1

# Simulation Data
v = 4.5; # m/s constant velocity
# States [x;y;psi;v]
z0 = [path.x[1]; path.y[1]; path.psi[1]; v];

zmax=[35;35;2*pi;10]
zmin=[-35;-35;-2*pi;-10]
umax=[0.6;4.5] # 6.0s 0-60mph time
umin=[-0.6;-8.0] # normal sedan braking limit
#= umax=[0.6;1.5*dt] =#

# Num of states and inputs
nz = 4;
nu = 2;

# Closed Loop MPC
u_cl, z_cl, u_ref_cl, z_ref_cl = simulateCarMPC(car_size,nz,nu,N,Lsim,dt,z0,path,zmin,zmax,umin,umax,y_stop)
print("Enter a filename (without .h5 extension)\n")
print("Press Enter to use default name: mpc_sim\n")
filename = strip(readline(STDIN))
if filename == ""
    filename = "mpc_sim"
end
print("Exporting Data Files\n")
h5open("$filename.h5", "w") do file
    write(file, "u_cl", u_cl)
    write(file, "z_cl", z_cl)
    write(file, "u_ref_cl", u_ref_cl)
    write(file, "z_ref_cl", z_ref_cl)
end
print("Continue by running Julia_after.m in Matlab\n")
