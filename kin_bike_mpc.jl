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
lw = 5;
ll = 30;
# y coordinate for stop sign, approaching from below
y_stop = -lw-(lf+df)-0.1;

N = 5 # MPC horizon
Lsim = 150 # Simulation duration
dt=0.1

# Simulation Data
v = 6; # m/s constant velocity
# States [x;y;psi;v]
z0 = [path.x[1]; path.y[1]; path.psi[1]; v];

# Modify target vehicle starting state if desired
use_target = true;
# Use this condition to make the ego vehicle wait
ztarg0 = [lw+ll+33; lw/2; pi; v];
# Use this condition to make the ego vehicle go ahead
ztarg0 = [lw+ll+35; lw/2; pi; v];

zmax=[35;35;2*pi;15]
zmin=[-50;-35;-2*pi;-10]
umax=[0.6;4.5] # 6.0s 0-60mph time
umin=[-0.6;-8.0] # normal sedan braking limit
#= umax=[0.6;1.5*dt] =#

# Num of states and inputs
nz = 4;
nu = 2;

# Closed Loop MPC
if use_target
    u_cl, z_cl, u_ol, z_ol, u_ref_ol, z_ref_ol, z_targ = simulateCarMPCTarget(car_size,nz,nu,N,Lsim,dt,z0,ztarg0,path,zmin,zmax,umin,umax,y_stop)
else
    u_cl, z_cl, u_ol, z_ol, u_ref_ol, z_ref_ol = simulateCarMPC(car_size,nz,nu,N,Lsim,dt,z0,path,zmin,zmax,umin,umax,y_stop)
end

print("\n")
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
    write(file, "u_ol", u_ol)
    write(file, "z_ol", z_ol)
    write(file, "u_ref_ol", u_ref_ol)
    write(file, "z_ref_ol", z_ref_ol)
    if use_target
        write(file, "z_targ", z_targ)
    end
end
print("Continue by running Julia_after.m in Matlab\n")
