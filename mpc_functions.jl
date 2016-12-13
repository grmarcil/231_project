using JuMP, Ipopt, Interpolations

function simulateCarMPC(car_size,nz,nu,N,Lsim,dt,z0,path,zmin,zmax,umin,umax,y_stop)
  # Track variable history for export
  z_cl_hist = zeros(nz,Lsim)
  u_cl_hist = zeros(nu,Lsim-1)
  z_ol_hist = Array{Float64}(nz,N+1,Lsim-1)
  u_ol_hist = Array{Float64}(nu,N,Lsim-1)
  z_ref_hist = Array{Float64}(nz,N+1,Lsim-1)
  u_ref_hist = Array{Float64}(nu,N,Lsim-1)
  z_t = z0[:]
  # Auxiliary var used for path reference tracking
  path_dist = 0;
  predicted_dist = 0;
  # Starting control mode
  mode = 1;

  print("-Starting MPC solver loop-\n")
  for t = 1:Lsim-1
    print("$t,")
    # Solve MPC problem
    u_vec, z_vec, u_ref, z_ref, path_dist, mode = solveMPC(car_size,nz,nu,N,dt,z_t,path,zmin,zmax,umin,umax,predicted_dist,y_stop,mode)
    # Update history vectors
    z_cl_hist[:,t] = z_t[:]
    u_cl_hist[:,t] = u_vec[:,1]
    z_ol_hist[:,:,t] = z_vec
    u_ol_hist[:,:,t] = u_vec
    z_ref_hist[:,:,t] = z_ref
    u_ref_hist[:,:,t] = u_ref
    # Update model based on dynamics with first MPC inputs
    u_t = u_vec[:,1]
    predicted_dist = path_dist + z_t[4]*dt;
    z_t = z_t + dt * zdot_fun(z_t,u_t,car_size[1])
  end
  z_cl_hist[:,Lsim] = z_t[:]
  return u_cl_hist, z_cl_hist, u_ol_hist, z_ol_hist, u_ref_hist, z_ref_hist
end

function detectMode(z_t, y_stop, last_mode)
    # should convert this to real state machine package
    # eg: https://github.com/tinybike/FiniteStateMachine.jl
    stopping_dist = 20
    epsilon_y = 0.3
    epsilon_v = 0.01

    y_t = z_t[2]
    v_t = z_t[4]

    if (y_t >= y_stop - stopping_dist) && (y_t <= (y_stop - epsilon_y))
        # if the car is within stopping_dist of the intersection but not yet within
        # epsilon_y of the stop sign
        # "approaching stop sign"
        mode = 2
        print("mode 2\n")
    # real mode 3 condition
    #= elseif (last_mode == 2 || last_mode == 3) && (abs(y_stop - y_t) < epsilon_y) && (v_t >= epsilon_v) =#
    # use this condition to test just full stop
    elseif (last_mode == 2 || last_mode == 3) && (abs(y_stop - y_t) < epsilon_y)
        # within epsilon of the stopping point and v_t still positive
        # "at stop sign, apply full brakes"
        mode = 3
        print("mode 3\n")
    elseif (abs(y_stop - y_t) < epsilon_y) && (v_t < epsilon_v)
        # v_t ~= 0 and at stop sign
        # "enter merge decision mode"
        mode = 4
        print("mode 4\n")
    elseif last_mode == 4 || last_mode == 5
        # finished deciding on merge strategy
        # "execute merge strategy"
        mode = 5
        print("mode 5\n")
    else
        mode = 1
        print("mode 1\n")
    end
    return mode
end

function solveMPC(car_size,nz,nu,N,dt,z_t,path,zmin,zmax,umin,umax,predicted_dist,y_stop,last_mode)
    lr=car_size[1]
    Lf=car_size[2]
    Lr=car_size[3]
    w=car_size[4]

    # Generate path-following references
    # Idea for full stop: create mode 3, triggered within 0.5m of y_stop
    # This mode will provide a reference of v=0 and y=y_stop for all horizon
    # steps. Need anything else? Change costs in this mode? Try second.
    # Mode 4 will be enterIntersection, triggered by mode=3or4 and v w/in
    # epsilon of 0. This will run the split front/behind merge controller in
    # Georg's paper
    mode = detectMode(z_t, y_stop, last_mode)
    if mode == 1
        u_ref, z_ref, path_dist = generateRef(z_t,path,predicted_dist,N,dt,lr);
    elseif mode == 2
        u_ref, z_ref, path_dist = generateRefStopping(z_t,path,predicted_dist,N,dt,lr,y_stop);
    elseif mode == 3
        u_ref, z_ref, path_dist = generateRefStop(z_t,path,predicted_dist,N,dt,lr,y_stop);
    elseif mode == 4
        u_ref, z_ref, path_dist = generateRefStopping(z_t,path,predicted_dist,N,dt,lr,y_stop);
    elseif mode == 4
        u_ref, z_ref, path_dist = generateRefStopping(z_t,path,predicted_dist,N,dt,lr,y_stop);
    end

    # Create model
    mpc = Model(solver=IpoptSolver(print_level=0))
    @variable(mpc,  zmin[i] <= z[i=1:nz,t=1:N+1] <= zmax[i])
    @variable(mpc,  umin[i] <= u[i=1:nu,t=1:N] <= umax[i])

    # Determine cost weights according to control mode
    if mode == 3
        # Full stop mode, penalize deviation from y=y_stop and v=0 heavily
        wz = [1;100;1;100];
        wu = [1;1];
    else
        wz = [1;1;1;1];
        wu = [1;1];
    end
    # Cost
    @objective(mpc, Min, sum(wz[1]*(z[1,t]-z_ref[1,t])^2 + wz[2]*(z[2,t]-z_ref[2,t])^2 +
        wz[3]*(z[3,t]-z_ref[3,t])^2 + wz[4]*(z[4,t]-z_ref[4,t])^2 for t=1:N+1) +
        sum(wu[1]*(u[1,t]-u_ref[1,t])^2 + wu[2]*(u[2,t]-u_ref[2,t])^2 for t=1:N))

    # Dynamics constraints across the horizon
    for t = 1:N
        @NLconstraint(mpc, z[1,t+1] == z[1,t] + dt*z[4,t]*cos(z[3,t]+u[1,t]))
        @NLconstraint(mpc, z[2,t+1] == z[2,t] + dt*z[4,t]*sin(z[3,t]+u[1,t]))
        @NLconstraint(mpc, z[3,t+1] == z[3,t] + dt*z[4,t]/lr*sin(u[1,t]))
        @NLconstraint(mpc, z[4,t+1] == z[4,t] + dt*u[2,t])
    end

    # Initial conditions
    @constraint(mpc, z[:,1] .== z_t)

    # Solve the NLP
    solve(mpc)
    # Return the control plan
    # return getvalue(u[:,0])
    #return getvalue(u[:,0]), getvalue(z[:,1])
    return getvalue(u), getvalue(z), u_ref, z_ref, path_dist, mode
end

function generateRef(z_t,path,predicted_dist,N,dt,lr)
    u_ref = Matrix(2,N);
    z_ref = Matrix(4,N+1);
    path_dist = findPathDist(z_t,path,predicted_dist);

    v = z_t[4];
    z_ref[:,1] = [path.itp_x[path_dist]; path.itp_y[path_dist]; path.itp_psi[path_dist]; v];

    dist_t = path_dist;
    for i=1:N
        # how far car should have gone at this time step
        dist_t = dist_t + v*dt;
        # interpolate x, y, psi values based on path.dist array
        x_ref_i = path.itp_x[dist_t];
        y_ref_i = path.itp_y[dist_t];
        psi_ref_i = path.itp_psi[dist_t];

        # will have to adapt for non-static velocities
        z_ref[:,i+1] = [x_ref_i; y_ref_i; psi_ref_i; v];
    end
    # should extend to do some logic here giving end of path if you've gone past
    # it, to stop car and not give NaN reference

    for i=1:N
        psi1 = z_ref[3,i];
        psi2 = z_ref[3,i+1];
        v1 = z_ref[4,i];
        v2 = z_ref[4,i+1];

        # saturate arcsin argument
        sin_beta = lr*(psi2-psi1)/(v1*dt)
        if sin_beta > 1
            sin_beta = 1
        elseif sin_beta < -1
            sin_beta = -1
        end

        beta_k = asin(sin_beta)
        a_k = (v2 - v1)/dt;
        u_ref[:,i] = [beta_k;a_k]
    end
    return u_ref, z_ref, path_dist
end

function generateRefStopping(z_t,path,predicted_dist,N,dt,lr,y_stop)
    u_ref = Matrix(2,N);
    z_ref = Matrix(4,N+1);
    path_dist = findPathDist(z_t,path,predicted_dist);

    # assume we want to do constant deceleration
    y_t = z_t[2]
    v_t = z_t[4]
    t_stop = 2*(y_stop - y_t)/v_t
    a_des = -v_t/t_stop

    z_ref[:,1] = [path.itp_x[path_dist]; path.itp_y[path_dist]; path.itp_psi[path_dist]; v_t];

    dist_t = path_dist;
    v_ref_i = v_t;
    for i=1:N
        # update v_ref with acceleration command
        v_ref_i = v_ref_i + a_des*dt
        # how far car should have gone at this time step
        dist_t = dist_t + v_ref_i*dt
        # interpolate x, y, psi values based on path.dist array
        x_ref_i = path.itp_x[dist_t]
        y_ref_i = path.itp_y[dist_t]
        psi_ref_i = path.itp_psi[dist_t]

        z_ref[:,i+1] = [x_ref_i; y_ref_i; psi_ref_i; v_ref_i]
    end

    for i=1:N
        psi1 = z_ref[3,i];
        psi2 = z_ref[3,i+1];
        v1 = z_ref[4,i];

        # saturate arcsin argument
        sin_beta = lr*(psi2-psi1)/(v1*dt)
        if sin_beta > 1
            sin_beta = 1
        elseif sin_beta < -1
            sin_beta = -1
        end

        beta_k = asin(sin_beta);
        u_ref[:,i] = [beta_k;a_des];
    end
    return u_ref, z_ref, path_dist
end

function generateRefStop(z_t,path,predicted_dist,N,dt,lr,y_stop)
    u_ref = Matrix(2,N);
    z_ref = Matrix(4,N+1);
    path_dist = findPathDist(z_t,path,predicted_dist);

    # current x, y, and psi
    x_t = z_t[1]
    y_t = z_t[2]
    psi_t = z_t[3]

    # set all z_ref to target y, 0 velocity, current x and psi
    for i=1:N+1
        z_ref[:,i] = [x_t; y_stop; psi_t; 0];
    end

    # set all input refs to 0
    for i=1:N
        u_ref[:,i] = [0;0];
    end
    return u_ref, z_ref, path_dist
end

# project current position onto path and return path dist at that point
function findPathDist(z0, path, predicted_dist)
    # Coarse but quick minimization of offset from the path
    # Could do a real optimization problem, haven't tried it

    # pick search range
    min_dist = predicted_dist - 1;
    max_dist = predicted_dist + 1;
    # don't search beyond path ends
    min_dist = max(min_dist, 0);
    max_dist = min(path.dist[end], max_dist);

    # check offset of your position from the path every 1cm in the search space
    dist_search = min_dist:0.01:max_dist;
    offsets = [];
    for i=1:length(dist_search)
        dx = z0[1] - path.itp_x[dist_search[i]];
        dy = z0[2] - path.itp_y[dist_search[i]];
        append!(offsets, sqrt(dx^2 + dy^2));
    end
    # take the minimum offset and return the corresponding path dist
    offset, idx = findmin(offsets);
    dist = dist_search[idx];
    return dist
end

function zdot_fun(z,u,l)
  beta=u[1]
  a=u[2]
  phi=z[3]
  V=z[4]
  xdot=V*cos(phi+beta)
  ydot=V*sin(phi+beta)
  phidot=V/l*sin(beta)
  Vdot=a
  zdot = [xdot;ydot;phidot;Vdot]
  return zdot
end

