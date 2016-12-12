using Interpolations

type Path
    x::Array
    y::Array
    psi::Array
    dist::Array
    itp_x::Interpolations.GriddedInterpolation
    itp_y::Interpolations.GriddedInterpolation
    itp_psi::Interpolations.GriddedInterpolation
end

function importPath()
    path_data = readcsv("path.csv");

    x_arr = path_data[1,:]
    y_arr = path_data[2,:]
    psi_arr = path_data[3,:]
    dist_arr = path_data[4,:]
    itp_x = interpolate(tuple(dist_arr),x_arr,Gridded(Linear()))
    itp_y = interpolate(tuple(dist_arr),y_arr,Gridded(Linear()))
    itp_psi = interpolate(tuple(dist_arr),psi_arr,Gridded(Linear()))

    return Path(x_arr,y_arr,psi_arr,dist_arr,itp_x,itp_y,itp_psi)
end
