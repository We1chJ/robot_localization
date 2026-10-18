import math
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from robot_localization.pf import Particle, ParticleFilter


def make_filter():
    """
    Creates an instante of the ParticleFilter class with mocked dependencies for unit testing.
    """
    # FYI: Copilot autocompleted most of this.
    pf = SimpleNamespace(
        particle_cloud=[],
        n_particles=60,
        range_min=0.1,
        range_max=10.0,
        inject_frac=0.0,
        INJECTION_FRACTION=0.3,
        lost_threshold=0.05,
        occupancy_field=SimpleNamespace(
            is_free=MagicMock(return_value=True),
            raycast=MagicMock(side_effect=lambda x, y, theta: x * y),
        ),
        publish_robot_pose=MagicMock(),
        get_logger=MagicMock(),
    )
    for name in ('is_free', 'normalize_particles', 'resample_particles','initialize_particle_cloud', 'update_robot_pose','update_particles_with_laser',):
        setattr(pf, name, MethodType(getattr(ParticleFilter, name), pf))
    return pf


class ParticleFilterTests(unittest.TestCase):
    """
    Unit tests for the ParticleFilter class.
    """
    def setUp(self):
        """
        Set up instance function.
        """
        # Saving random state to ensure reproducibility.
        random_state = np.random.get_state()
        self.addCleanup(np.random.set_state, random_state)
        np.random.seed(0)


        self.pf = make_filter()

    def test_update_particle_weights(self):
        """
        Check that the particle weights are updated correctly based on the laser scan data and that the weights are normalized.
        """
        self.pf.particle_cloud = [
            Particle(x=x, y=1.0, w=1.0 / 3) for x in (1.0, 1.25, 1.5)
        ]
        self.pf.update_particles_with_laser([1.0] * 10, [0.0] * 10)
        weights = [p.w for p in self.pf.particle_cloud]
        self.assertGreater(weights[0], weights[1])
        self.assertGreater(weights[1], weights[2])
        self.assertTrue(all(w > 0 for w in weights))
        self.assertAlmostEqual(sum(weights), 1.0)
        self.assertEqual(self.pf.occupancy_field.raycast.call_count, 30)

    def test_normalize_particles(self):
        """
        I mean do you really want me to explain this one? It just normalizes the weights of the particles so that they sum to 1.
        """
        self.pf.particle_cloud = [Particle(w=w) for w in (2.0, 3.0, 5.0)]
        self.pf.normalize_particles()
        np.testing.assert_allclose([p.w for p in self.pf.particle_cloud], [0.2, 0.3, 0.5])

    def test_normalize_invalid_weights(self):
        """
        Make sure that NaN, negative, and zero weights are handled well.
        """
        self.pf.particle_cloud = [Particle(w=float('nan')), Particle(w=-1.0), Particle(w=0.0)]
        self.pf.normalize_particles()
        np.testing.assert_allclose([p.w for p in self.pf.particle_cloud], [1.0 / 3] * 3)

    def test_resample_particles(self):
        """
        Make sure that resampling only propagates the particles with non-zero weights and that the resulting weights are normalized.
        """
        self.pf.particle_cloud = [
            Particle(x=1.0, y=1.0, w=0.0),
            Particle(x=2.0, y=2.0, w=0.0),
            Particle(x=3.0, y=3.0, w=1.0),
        ]
        # This makes the random number generator always return 0, which means that there will be no noise added to the resampled particles, so they will all be at (3.0, 3.0).
        with patch('robot_localization.pf.np.random.normal', return_value=0.0):
            self.pf.resample_particles()
        self.assertEqual(len(self.pf.particle_cloud), 3)
        for particle in self.pf.particle_cloud:
            self.assertEqual((particle.x, particle.y), (3.0, 3.0))
            self.assertAlmostEqual(particle.w, 1.0 / 3)
        self.assertEqual(len(self.pf.particle_cloud), 3)

    def test_initialize_particles(self):
        """
        Make sure script produces the requested count and normalized weights particles.
        """
        self.pf.initialize_particle_cloud(timestamp=None, xy_theta=(5.0, 5.0, 0.0))
        self.assertEqual(len(self.pf.particle_cloud), 60)
        for particle in self.pf.particle_cloud:
            self.assertAlmostEqual(particle.w, 1.0 / 60)
        self.pf.publish_robot_pose.assert_called_once()

if __name__ == '__main__':
    unittest.main(verbosity=2)
