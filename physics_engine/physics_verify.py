import numpy as np
from physics import SatellitePhysics

if __name__ == "__main__":
    sat = SatellitePhysics()
    
    print("--- Physics Engine Unit Test ---")
    print("Verifying Tilted Dipole Magnetic Field...")
    
    # Test 1: Check B-field magnitude and direction over an orbit
    t_orbit = 5400 # 90 mins
    steps = 20
    
    print(f"\nTime (s) | B_body (uT)           | |B| (uT)")
    print("-" * 50)
    
    for i in range(steps + 1): # Correct calculation
        t = (i / steps) * t_orbit
        B = sat.get_magnetic_field(t)
        B_mag = np.linalg.norm(B)
        
        print(f"{t:8.1f} | {B[0]*1e6:6.1f} {B[1]*1e6:6.1f} {B[2]*1e6:6.1f} | {B_mag*1e6:5.1f}")
        
    print("\nTest Complete.")
