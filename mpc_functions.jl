using JuMP, Ipopt, Interpolations

function simulateCarMPC(car_size,nz,nu,N,Lsim,dt,z0,path,zmin,zmax,umin,umax,y_stop)
  z_history = zeros(nz,Lsim)
  u_history = zeros(nu,Lsim-1)
  z_ref_history = Array{Float64}(nz,N+1,Lsim-1)
  u_ref_history = Array{Float64}(nu,N,Lsim-1)
  z_t = z0[:]
  # Auxiliary var used for path reference tracking
  path_dist = 0;
  predicted_dist = 0;

  print("-Starting MPC solver loop-\n")
  for t = 1:Lsim-1
    print("$t,")
    u_vec, z_vec, u_ref, z_ref, path_dist = solveMPC(car_size,nz,nu,N,dt,z_t,path,zmin,zmax,umin,umax,predicted_dist,y_stop)
    # Update model based on dynamics with first MPC inputs
    u_t = u_vec[:,1]
    z_history[:,t] = z_t[:]
    u_history[:,t] = u_t[:]
    z_ref_history[:,:,t] = z_ref
    u_ref_history[:,:,t] = u_ref
    predicted_dist = path_dist + z_t[4]*dt;
    z_t = z_t + dt * zdot_fun(z_t,u_t,car_size[1])
  end
  z_history[:,Lsim] = z_t[:]
  return  u_history, z_history, u_ref_history, z_ref_history
end

function detectMode(z_t, y_stop)
    stopping_dist = 20
    y_t = z_t[2]
    dist_to_stop = y_stop - y_t
    if dist_to_stop <= stopping_dist
        mode = 2 # approaching stop sign
    else
        mode = 1
    end
    return mode
end

function solveMPC(car_size,nz,nu,N,dt,z_t,path,zmin,zmax,umin,umax,predicted_dist,y_stop)
    lr=car_size[1]
    Lf=car_size[2]
    Lr=car_size[3]
    w=car_size[4]

    # Generate path-following references
    mode = detectMode(z_t, y_stop)
    if mode == 1
        u_ref, z_ref, path_dist = generateRef(z_t,path,predicted_dist,N,dt,lr);
    elseif mode == 2
        u_ref, z_ref, path_dist = generateRefStopping(z_t,path,predicted_dist,N,dt,lr,y_stop);
    end

    # Create model
    mpc = Model(solver=IpoptSolver(print_level=0))
    @variable(mpc,  zmin[i] <= z[i=1:nz,t=1:N+1] <= zmax[i])
    @variable(mpc,  umin[i] <= u[i=1:nu,t=1:N] <= umax[i])

    # Cost
    Q = eye(4);
    R = eye(2);
    #= @objective(mpc, Min, sum((z[:,t]-z_ref[:,t])'*Q*(z[:,t]-z_ref[:,t]) for =#
    #=     t=1:N)[1] + sum((u[:,t]-u_ref[:,t])'*R*(u[:,t]-u_ref[:,t]) for t=1:N-1)[1]) =#
    @objective(mpc, Min, sum((z[1,t]-z_ref[1,t])^2 + (z[2,t]-z_ref[2,t])^2 +
        (z[3,t]-z_ref[3,t])^2 + (z[4,t]-z_ref[4,t])^2 for t=1:N+1) +
        sum((u[1,t]-u_ref[1,t])^2 + (u[2,t]-u_ref[2,t])^2 for t=1:N))
    #= @objective(mpc, Min, 0) =#

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
    return getvalue(u), getvalue(z), u_ref, z_ref, path_dist
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
        z_1 = z_ref[:,i];
        z_2 = z_ref[:,i+1];
        a_k = (z_2[4] - z_1[4])/dt;
        beta_k = asin(lr*(z_2[3]-z_1[3])/(z_1[4]*dt));
        u_ref[:,i] = [a_k; beta_k];
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
        z_1 = z_ref[:,i];
        z_2 = z_ref[:,i+1];
        beta_k = asin(lr*(z_2[3]-z_1[3])/(z_1[4]*dt));
        u_ref[:,i] = [a_des; beta_k];
    end
    return u_ref, z_ref, path_dist
end

# project current position onto path and return path dist at that point
function findPathDist(z0, path, predicted_dist)
    # Crude but hopefully quick minimization of offset from the path
    # Could incorporate psi distance if this doesn't work well enough
    # Or could expand search range, or do a real optimization problem

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

