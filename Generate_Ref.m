%% Generate Reference
% 
% Generate N steps of state reference points at each timestep of the MPC problem
% ------------------------------------------------------------------------------
% Generate_Ref(z0,curve,N,curve_idx)
% ------------------------------------------------------------------------------
% z0: current state coordinates of the car (x,y,psi,v) ([m], [m], [rad], [m/s])
% curve: struct with properties x, y, psi, dist. Each property is an array of
% those values along the reference curve at sample points. The struct also has
% functions interp_x, interp_y, interp_psi, which generate interpolated values
% of those measures along the curve based on dist.
% predicted_dist: last predicted distance travelled along the curve, defaults to 0
% N: MPC horizon
% dt: MPC time step
% lr: dist from car COM to rear axle
% ------------------------------------------------------------------------------
% z_ref: state reference (4xN+1 vector)
% u_ref: input reference (2xN vector)
% dist: project car's current position onto curve, return curve distance
% The main algorithm should update predicted distance with
% predicted_dist = dist + v*dt before running this function again.
% ------------------------------------------------------------------------------

function [z_ref, u_ref, dist] = Generate_Ref(z0,curve,predicted_dist,N,dt,lr)
    if ~exist('predicted_dist', 'var')
        predicted_dist = 0;
    end
    dist = Find_Path_Dist(z0,curve,predicted_dist);
    v = z0(3);
    z0_proj = [curve.interp_x(dist); curve.interp_y(dist); v; curve.interp_psi(dist)];
    z_ref = [z0_proj];

    dist_k = dist;
    for i=1:N
        % how far car should have gone at this time step
        dist_k = dist_k + v*dt;
        % interpolate x, y, psi values based on curve.dist array
        x_ref_i = curve.interp_x(dist_k);
        y_ref_i = curve.interp_y(dist_k);
        psi_ref_i = curve.interp_psi(dist_k);

        % will have to adapt for non-static velocities
        z_ref = [z_ref, [x_ref_i; y_ref_i; v; psi_ref_i]];
    end
    
    u_ref = [];
    for i=1:N
        z_1 = z_ref(:,i);
        z_2 = z_ref(:,i+1);
        a_k = (z_2(3) - z_1(3))/dt;
        beta_k = asin(lr*(z_2(4)-z_1(4))/(z_1(3)*dt));
        u_ref = [u_ref, [a_k; beta_k]];
    end
end

% project current position onto curve and return curve dist at that point
function dist = Find_Path_Dist(z0, curve, predicted_dist)
    % Crude but hopefully quick minimization of offset from the curve
    % Could incorporate psi distance if this doesn't work well enough
    % Or could expand search range, or do a real optimization problem
    
    % pick search range
    min_dist = predicted_dist - 1;
    max_dist = predicted_dist + 1;
    % don't search beyond path ends
    min_dist = max([min_dist, 0]);
    max_dist = min([curve.dist(end), max_dist]);
    
    % check offset of your position from the curve every 1cm in the search space
    dist_search = min_dist:0.01:max_dist;
    offsets = [];
    for i=1:length(dist_search)
        dx = z0(1) - curve.interp_x(dist_search(i));
        dy = z0(2) - curve.interp_y(dist_search(i));
        offsets = [offsets sqrt(dx^2 + dy^2)];
    end
    % take the minimum offset and return the corresponding curve dist
    [offset,idx] = min(offsets);
    dist = dist_search(idx);
end