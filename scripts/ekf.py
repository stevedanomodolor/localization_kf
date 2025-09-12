import sympy as sp


if __name__ == "__main__":
    
    
    dt = sp.symbols('dt')  # time step
    # state vector symbols
    x, y, z = sp.symbols('x y z')              # position
    vx, vy, vz = sp.symbols('vx vy vz')             # velocity enu
    phi, theta, psi = sp.symbols('phi theta psi')   # orientation
    p, q, r = sp.symbols('p q r')                      # angular body rates

    # state vector
    state_vec = sp.Matrix([x, y, z, vx, vy, vz, phi, theta, psi, p, q, r])

    # control vector symbols(accelerations)
    abx, aby, abz = sp.symbols('abx aby abz') # accelerations in body frame
    p_in, q_in, r_in = sp.symbols('p_in q_in r_in')  # angular body rates in body frame

    # control vector -> imu
    u = sp.Matrix([abx, aby, abz, p_in, q_in, r_in])


    # --- Rotation matrix R_bg (ZYX)  transpose (or inverse) of Diebel ---
    Rbg = sp.Matrix([
        [sp.cos(theta)*sp.cos(psi),
        sp.sin(phi)*sp.sin(theta)*sp.cos(psi) - sp.cos(phi)*sp.sin(psi),
        sp.cos(phi)*sp.sin(theta)*sp.cos(psi) + sp.sin(phi)*sp.sin(psi)],

        [sp.cos(theta)*sp.sin(psi),
        sp.sin(phi)*sp.sin(theta)*sp.sin(psi) + sp.cos(phi)*sp.cos(psi),
        sp.cos(phi)*sp.sin(theta)*sp.sin(psi) - sp.sin(phi)*sp.cos(psi)],

        [-sp.sin(theta),
        sp.cos(theta)*sp.sin(phi),
        sp.cos(theta)*sp.cos(phi)]
    ])
    
    # --- T(phi,theta) matrix for Euler angle rates ---
    T = sp.Matrix([
        [1, sp.sin(phi)*sp.tan(theta), sp.cos(phi)*sp.tan(theta)],
        [0, sp.cos(phi), -sp.sin(phi)],
        [0, sp.sin(phi)/sp.cos(theta), sp.cos(phi)/sp.cos(theta)]
    ])
    
    
    # --- Transition function g(x,u,dt) ---
    a_b = sp.Matrix(u[0:3])   # acceleration in body frame
    w_b = sp.Matrix(u[3:6])  # angular rates in body frame
    a_w = Rbg @ a_b
    W_w = T @ w_b


    # --- State transition function g(x,u,dt) ---
    
    g = sp.Matrix.vstack(
        # Position update (world frame)
        sp.Matrix([x, y, z]) + (sp.Matrix([vx, vy, vz])) * dt,

        # Velocity update (body frame)
        sp.Matrix([vx, vy, vz]) + a_w * dt,

        # Euler angles update (body frame)
        sp.Matrix([phi, theta, psi]) + W_w * dt,

        # Angular rates update (body frame)
        w_b
    )

    # sp.pprint(g)
    

    # compute the jacobian wrt state 
    g_prime = g.jacobian(state_vec)
    print("Jacobian:")
    sp.pprint(g_prime)
    # // print every element of g_prime
    for i in range(g_prime.rows):
        for j in range(g_prime.cols):
            print(f"g_prime[{i},{j}] = {g_prime[i,j]}")
    
    
    ## Gps model 
    # measurement vector symbols
    x_gps, y_gps, z_gps = sp.symbols('x_gps y_gps z_gps')
    vx_enu, vy_enu, vz_enu = sp.symbols('vx_enu vy_enu vz_enu')
    
    # Mesurement model
    h_gps = sp.Matrix([
        x,
        y,
        z,
        vx,
        vy,
        vz
    ])

    h_gps_prime = h_gps.jacobian(state_vec)

    z_gps = sp.Matrix([x_gps, y_gps, z_gps, vx_enu, vy_enu, vz_enu])

    print("GPS Measurement model:")
    sp.pprint(h_gps)
    sp.pprint(h_gps_prime)

    # Mag measurement model
    psi_m = sp.symbols('psi_m')

    z_mag = sp.Matrix([psi_m])

    h_mag = sp.Matrix([psi])

    h_mag_prime = h_mag.jacobian(state_vec)
    
    print("Mag Measurement model:")
    sp.pprint(h_mag)
    sp.pprint(h_mag_prime)

    # Altimeter measurement model
    z_barom, vz_barom = sp.symbols('z_barom vz_barom')

    h_barom = sp.Matrix([z, vz])

    h_barom_prime = h_barom.jacobian(state_vec)

    print("Altimeter Measurement model:")
    sp.pprint(h_barom)
    sp.pprint(h_barom_prime)