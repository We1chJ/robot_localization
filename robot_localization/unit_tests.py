import math
from unittest.mock import MagicMock
import rclpy
from pf import Particle, ParticleFilter
from robot_localization.occupancy_field import OccupancyField
import rclpy.client
def test_update_particle_weights():
    
    pf : ParticleFilter = ParticleFilter()
    # Make mock occupancy field for testing
    pf.occupancy_field = OccupancyField(pf)
    # Mock method to return a distance based on x * y for testing purposes
    pf.occupancy_field.get_closest_obstacle_distance = lambda x, y: x * y
    
    pf.particle_cloud = [
        Particle(x=1.0, y=1.0, theta=0.0, weight=1.0),
        Particle(x=2.0, y=2.0, theta=0.0, weight=1.0),
        Particle(x=3.0, y=3.0, theta=0.0, weight=1.0)
    ]
    
    r = [1.5, 2.5, 3.5]
    theta = [0.0, 0.5, 1.0]
    pf.update_particles_with_laser(r, theta)

    # Check that weights are weighted correctly based on the mock occupancy field. (weights should be particle 1 > particle 2 > particle 3)
    assert pf.particle_cloud[0].weight > pf.particle_cloud[1].weight, "Particle 1 should have a higher weight than Particle 2"
    assert pf.particle_cloud[1].weight > pf.particle_cloud[2].weight, "Particle 2 should have a higher weight than Particle 3"

    assert len(pf.particle_cloud) == 3, "Particle cloud should have 3 particles"
    assert all(p.weight > 0 for p in pf.particle_cloud), "All particle weights should be positive"
    assert math.isclose(sum(p.weight for p in pf.particle_cloud), 1.0), "Total weight should be normalized to 1"


def test_normalize_particles():
    rclpy.init()
    rclpy.client.Client.wait_for_service = MagicMock(return_value=True)
    
    pf : ParticleFilter = ParticleFilter()
    
    pf.particle_cloud = [
        Particle(x=1.0, y=1.0, theta=0.0, weight=2.0),
        Particle(x=2.0, y=2.0, theta=0.0, weight=3.0),
        Particle(x=3.0, y=3.0, theta=0.0, weight=5.0)
    ]
    
    # Normalize the particle weights
    pf.normalize_particles()
    total_weight = sum(p.weight for p in pf.particle_cloud)
    assert math.isclose(total_weight, 1.0), "Total weight should be normalized to 1"
    # Ensure particles have the same relative weights after normalization
    assert math.isclose(pf.particle_cloud[0].weight / pf.particle_cloud[1].weight, 2.0 / 3.0), "Relative weights should be preserved after normalization"
    assert math.isclose(pf.particle_cloud[1].weight / pf.particle_cloud[2].weight, 3.0 / 5.0), "Relative weights should be preserved after normalization"

def test_resample_particles():
    rclpy.init()
    rclpy.client.Client.wait_for_service = MagicMock(return_value=True)
    
    pf : ParticleFilter = ParticleFilter()

    # Create starting particles with various weights
    # We need to think about how the resample is made (random weighted choice)
    # For testing, we will create a simple scenario where one particle has a significantly higher weight than the others
    pf.particle_cloud = [
        Particle(x=1.0, y=1.0, theta=0.0, weight=0.01),
        Particle(x=2.0, y=2.0, theta=0.0, weight=0.01),
        Particle(x=3.0, y=3.0, theta=0.0, weight=0.98)
    ]
    
    # Resample the particles
    pf.resample_particles()
    
    # After resampling, we should still have the same number of particles
    assert len(pf.particle_cloud) == 3, "Particle cloud should have 3 particles after resampling"

    # Check that the particles are resampled according to their weights
    # Since the third particle had a much higher weight, we expect it to appear more frequently
    assert any(p.x == 3.0 and p.y == 3.0 for p in pf.particle_cloud), "At least one particle should be the one with the highest weight (This check has a low, low probability of flagging a false positive)"
    # Check that the weights are reset to uniform distribution
    for p in pf.particle_cloud:
        assert math.isclose(p.weight, 1/3), "All particle weights should be equal after resampling"

def test_initialize_particles():
    rclpy.init()
    rclpy.client.Client.wait_for_service = MagicMock(return_value=True)

    pf : ParticleFilter = ParticleFilter()
    # Initialize particles with mean x and y of 5, theta of 0.
    pf.initialize_particle_cloud(xy_theta = (5.0, 5.0, 0.0), num_particles=60)

    #Ensure 60 particles was made
    assert len(pf.particle_cloud) == 60, "Particle cloud should have 60 particles after initialization"
    #Ensure mean is within a reasonable margin of (5, 5)
    mean_x = sum(p.x for p in pf.particle_cloud) / len(pf.particle_cloud)
    mean_y = sum(p.y for p in pf.particle_cloud) / len(pf.particle_cloud)
    assert abs(mean_x - 5.0) < 1.0, "Mean x should be close to 5.0"
    assert abs(mean_y - 5.0) < 1.0, "Mean y should be close to 5.0"
    # Ensure all particles mean theta is within a reasonable margin of 0
    mean_theta = sum(p.theta for p in pf.particle_cloud) / len(pf.particle_cloud)
    assert abs(mean_theta - 0.0) < 0.5, "Mean theta should be close to 0.0"
    # Ensure all particles have equal weights
    for p in pf.particle_cloud:
        assert math.isclose(p.weight, 1/60), "All particle weights should be equal after initialization"

    pf.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    test_update_particle_weights()
    test_normalize_particles()
    test_resample_particles()
    test_initialize_particles()
    print("All tests passed!")
