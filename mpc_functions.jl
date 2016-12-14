using JuMP, Ipopt, Interpolations

function simulateCarMPCTarget(car_size,nz,nu,N,Lsim,dt,z0,ztarg0,path,zmin,zmax,umin,umax,y_stop)
  # Track variable history for export
  z_cl_hist = zeros(nz,Lsim)
  u_cl_hist = zeros(nu,Lsim-1)
  z_ol_hist = Array{Float64}(nz,N+1,Lsim-1)
  u_ol_hist = Array{Float64}(nu,N,Lsim-1)
  z_ref_hist = Array{Float64}(nz,N+1,Lsim-1)
  u_ref_hist = Array{Float64}(nu,N,Lsim-1)
  z_targ_hist = Array{Float64}(nz,N+1,Lsim-1)
  z_t = z0[:]
  z_targ_t = ztarg0[:]
  # Auxiliary var used for path reference tracking
  path_dist = 0;
  predicted_dist = 0;
  # Starting control mode
  mode = 1;

  print("-Starting MPC solver loop-\n")
  for t = 1:Lsim-1
    print("$t,")
    # Solve MPC problem
    u_vec, z_vec, u_ref, z_ref, path_dist, mode = solveMPCTarget(car_size,nz,nu,N,dt,z_t,z_targ_t,path,zmin,zmax,umin,umax,predicted_dist,y_stop,mode,t)
    # Update history vectors
    z_cl_hist[:,t] = z_t[:]
    u_cl_hist[:,t] = u_vec[:,1]
    z_ol_hist[:,:,t] = z_vec
    u_ol_hist[:,:,t] = u_vec
    z_ref_hist[:,:,t] = z_ref
    u_ref_hist[:,:,t] = u_ref
    z_targ_hist[:,t] = z_targ_t[:]
    # Update model based on dynamics with first MPC inputs
    u_t = u_vec[:,1]
    predicted_dist = path_dist + z_t[4]*dt;
    z_t = z_t + dt * zdot_fun(z_t,u_t,car_size[1])
    z_targ_t = z_targ_t + dt * zdot_fun(z_targ_t,[0;0],car_size[1])
  end
  z_cl_hist[:,Lsim] = z_t[:]
  z_targ_hist[:,Lsim] = z_targ_t[:]
  return u_cl_hist, z_cl_hist, u_ol_hist, z_ol_hist, u_ref_hist, z_ref_hist, z_targ_hist
end

# For running without target vehicle
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

function detectMode(z_t, z_targ_t, y_stop, last_mode)
    # should convert this to real state machine package
    # eg: https://github.com/tinybike/FiniteStateMachine.jl
    stopping_dist = 20
    epsilon_y = 0.3
    epsilon_v = 0.05

    y_t = z_t[2]
    v_t = z_t[4]

    if (y_t >= y_stop - stopping_dist) && (y_t <= (y_stop - epsilon_y))
        # if the car is within stopping_dist of the intersection but not yet within
        # epsilon_y of the stop sign
        # "approaching stop sign"
        mode = 2
        print("mode 2\n")
    elseif (last_mode == 2 || last_mode == 3) && (abs(y_stop - y_t) < epsilon_y) && (v_t >= epsilon_v)
    #elseif (last_mode == 2 || last_mode == 3) && (abs(y_stop - y_t) < epsilon_y) # use this condition to test just full stop
        # within epsilon of the stopping point and v_t still positive
        # "at stop sign, apply full brakes"
        mode = 3
        print("mode 3\n")
    elseif last_mode == 3 && (abs(y_stop - y_t) < epsilon_y) && (v_t < epsilon_v)
        # v_t ~= 0 and at stop sign
        # "enter merge decision mode"
        if shouldWait(z_targ_t)
            mode = 5
        else
            mode = 4
        end

        print("mode $mode\n")
    elseif last_mode == 4
        # decided to go ahead of target
        # "execute merge strategy"
        mode = 4
        print("mode 4\n")
    elseif last_mode == 5
        # decided to wait for target to pass
        # "execute merge strategy"
        mode = 5
        print("mode 5\n")
    else
        mode = 1
        print("mode 1\n")
    end
    return mode
end

function shouldWait(z_targ_t)
    # Heuristics: ego vehicle can perform left turn and clear the intersection
    # in roughly 3 seconds. Check if target will enter intersection within 5
    # seconds, wait if so.
    x_targ = z_targ_t[1]
    v_targ = z_targ_t[4]
    lw = 5.0 # TODO again need to move to an object/class based data structure to stop having to decide between hardcoding constants or passing crazy function argument strings
    time_to_enter = (x_targ - lw)/v_targ
    print(time_to_enter)

    if time_to_enter < 3.3
        return true
    else
        return false
    end
end

function solveMPCTarget(car_size,nz,nu,N,dt,z_t,z_targ_t,path,zmin,zmax,umin,umax,predicted_dist,y_stop,last_mode,timestep)
    lr=car_size[1]
    Lf=car_size[2]
    Lr=car_size[3]
    w=car_size[4]
    lw=5

    # Generate path-following references
    # Idea for full stop: create mode 3, triggered within 0.5m of y_stop
    # This mode will provide a reference of v=0 and y=y_stop for all horizon
    # steps. Need anything else? Change costs in this mode? Try second.
    # Mode 4 will be enterIntersection, triggered by mode=3or4 and v w/in
    # epsilon of 0. This will run the split front/behind merge controller in
    # Georg's paper
    mode = detectMode(z_t, z_targ_t, y_stop, last_mode)
    if mode == 1
        u_ref, z_ref, path_dist = generateRef(z_t,path,predicted_dist,N,dt,lr);
    elseif mode == 2
        u_ref, z_ref, path_dist = generateRefStopping(z_t,path,predicted_dist,N,dt,lr,y_stop);
    elseif mode == 3
        u_ref, z_ref, path_dist = generateRefStop(z_t,path,predicted_dist,N,dt,lr,y_stop);
    elseif mode == 4 # merge before target
        u_ref, z_ref, path_dist = generateRefGoAhead(z_t,z_targ_t,path,predicted_dist,N,dt,lr);
    elseif mode == 5 # merge after target
        u_ref, z_ref, path_dist = generateRefGoBehind(z_t,z_targ_t,path,predicted_dist,N,dt,lr);
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
        wtarg = 0;
    elseif mode == 4
        # Don't penalize v and a deviations much
        wz = [1;1;1;0.1];
        wu = [1;0.1];
        wtarg = 100;
    else
        wz = [1;1;1;1];
        wu = [1;1];
        wtarg = 0;
    end
    # Cost
    @NLobjective(mpc, Min, sum(wz[1]*(z[1,t]-z_ref[1,t])^2 + wz[2]*(z[2,t]-z_ref[2,t])^2 +
        wz[3]*(z[3,t]-z_ref[3,t])^2 + wz[4]*(z[4,t]-z_ref[4,t])^2 for t=1:N+1) +
        sum(wu[1]*(u[1,t]-u_ref[1,t])^2 + wu[2]*(u[2,t]-u_ref[2,t])^2 for t=1:N)
            + sum(wtarg/((z[1,t]-z_targ_t[1])^2) for t=1:N))

    # Dynamics constraints across the horizon
    for t = 1:N
        @NLconstraint(mpc, z[1,t+1] == z[1,t] + dt*z[4,t]*cos(z[3,t]+u[1,t]))
        @NLconstraint(mpc, z[2,t+1] == z[2,t] + dt*z[4,t]*sin(z[3,t]+u[1,t]))
        @NLconstraint(mpc, z[3,t+1] == z[3,t] + dt*z[4,t]/lr*sin(u[1,t]))
        @NLconstraint(mpc, z[4,t+1] == z[4,t] + dt*u[2,t])
    end

    # Initial conditions
    @constraint(mpc, z[:,1] .== z_t)

    # Boundary Constraints
    # Four corners of the car in global coordinates
    #FR =[x+lf*cos(psi)+w/2*sin(psi),y+lf*sin(psi)-w/2*cos(psi)]
    #FL =[x+lf*cos(psi)-w/2*sin(psi),y+lf*sin(psi)+w/2*cos(psi)]
    #BR =[x-lr*cos(psi)+w/2*sin(psi),y-lr*sin(psi)-w/2*cos(psi)]
    #BL =[x-lr*cos(psi)-w/2*sin(psi),y-lr*sin(psi)+w/2*cos(psi)]
    for t = 1:N
        # later should do this with full box model to account for psi
        @constraint(mpc, (z[1,t] + w/2) <= lw) # right hand lane constraint
        # front right tire should not go above the top boundary of the
        # horizontal lane
        #= @NLconstraint(mpc, (z[2,t] + Lf*sin(z[3,t]) - w/2*cos(z[3,t])) <= lw) =#

        if mode <= 3 # driving vertically
            @constraint(mpc, (z[1,t] - w/2) >= 0) # centerline constraint
            @constraint(mpc, (z[2,t] + Lf) <= 0) # don't overshoot stop
        end

        if mode >= 4
            # lambda variables for OR polygon union of vertical lane, horizontal
            # lane, and a bit of tangent in the intersection
            @variable(mpc, lambda[i=1:2,t=1:N+1] >= 0)
            # front left tire must be above imaginary 45degree line connecting
            # the vertical and horizontal centerlines or above the horizontal
            # centerline
            @NLconstraint(mpc,
            lambda[1,t]*(z[1,t]+Lf*cos(z[3,t])-w/2*sin(z[3,t]) + z[2,t]+Lf*sin(z[3,t])+w/2*cos(z[3,t]) + lw) + #45 degree line
            lambda[2,t]*(z[2,t]+Lf*sin(z[3,t])+w/2*cos(z[3,t])) >= 0) # horizontal centerline
            @constraint(mpc, lambda[1,t]+lambda[2,t] == 1)
        end

        # merge ahead of target car safely
        if mode == 4
            x_targ0 = z_targ_t[1]
            v_targ = z_targ_t[4]
            # stay at least 1m ahead of target
            @constraint(mpc, z[1,t]+Lr+1 <= x_targ0 - dt*v_targ*t)
        end
        # wait to merge behind target car
        if mode == 5
        end
    end

    # Solve the NLP
    solve(mpc)
    # Return the control plan
    # return getvalue(u[:,0])
    #return getvalue(u[:,0]), getvalue(z[:,1])
    return getvalue(u), getvalue(z), u_ref, z_ref, path_dist, mode
end

function solveMPC(car_size,nz,nu,N,dt,z_t,path,zmin,zmax,umin,umax,predicted_dist,y_stop,last_mode)
    lr=car_size[1]
    Lf=car_size[2]
    Lr=car_size[3]
    w=car_size[4]

    #fix this to work with new z_targ_t detectMode option, or just refactor all
    #of this code, lots of duplication because rushing to finish project
    mode = detectMode(z_t, y_stop, last_mode)
    if mode == 1
        u_ref, z_ref, path_dist = generateRef(z_t,path,predicted_dist,N,dt,lr);
    elseif mode == 2
        u_ref, z_ref, path_dist = generateRefStopping(z_t,path,predicted_dist,N,dt,lr,y_stop);
    elseif mode == 3
        u_ref, z_ref, path_dist = generateRefStop(z_t,path,predicted_dist,N,dt,lr,y_stop);
    elseif mode == 4
        # not yet implemented for no target vehicle
        u_ref, z_ref, path_dist = generateRef(z_t,path,predicted_dist,N,dt,lr);
    elseif mode == 5
        u_ref, z_ref, path_dist = generateRef(z_t,path,predicted_dist,N,dt,lr);
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

function generateRefGo(z_t,path,predicted_dist,N,dt,lr)
    u_ref = Matrix(2,N);
    z_ref = Matrix(4,N+1);
    path_dist = findPathDist(z_t,path,predicted_dist);

    # TODO refactor this so you don't repeat this constant v
    v = 6;
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
function generateRefGoAhead(z_t,z_targ_t,path,predicted_dist,N,dt,lr)
    u_ref = Matrix(2,N);
    z_ref = Matrix(4,N+1);
    path_dist = findPathDist(z_t,path,predicted_dist);

    # TODO refactor this so you don't repeat this constant v
    v = 6;
    x_targ = z_targ_t[1]
    v_targ = z_targ_t[4]
    z_ref[:,1] = [path.itp_x[path_dist]; path.itp_y[path_dist]; path.itp_psi[path_dist]; v];

    dist_t = path_dist;
    for i=1:N
        speedUp = true;
        safe_x = x_targ - 7 - v_targ*dt*i
        # define first all vars outside of local while
        dist_t = dist_t + v*dt;
        # interpolate x, y, psi values based on path.dist array
        x_ref_i = path.itp_x[dist_t];
        y_ref_i = path.itp_y[dist_t];
        psi_ref_i = path.itp_psi[dist_t];
        # how far car should have gone at this time step
        #= while(speedUp) =#
        #=     dist_t = dist_t + v*dt; =#
        #=     # interpolate x, y, psi values based on path.dist array =#
        #=     x_ref_i = path.itp_x[dist_t]; =#
        #=     y_ref_i = path.itp_y[dist_t]; =#
        #=     psi_ref_i = path.itp_psi[dist_t]; =#
        #=     if x_ref_i > safe_x =#
        #=         speedUp = true =#
        #=         v += 0.5 =#
        #=     else =#
        #=         speedUp = false =#
        #=     end =#
        #= end =#

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
