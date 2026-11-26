"""Project-local Pegasus overrides; no changes to the installed extension."""
import numpy as np
from scipy.spatial.transform import Rotation


class LocalWrenches:
    """Accumulate local-frame wrenches in the native view's actual body order."""

    def __init__(self, body_parts):
        self.rows = {part: index for index, part in enumerate(body_parts)}
        if len(self.rows) != len(body_parts) or '/body' not in self.rows:
            raise ValueError('Direct tensor body paths must be unique and include /body')
        self.data = np.zeros((2, len(body_parts), 3), dtype=np.float32)

    def clear(self):
        self.data.fill(0)

    def force(self, force, position, body_part):
        row = self.rows[body_part]
        self.data[0, row] += np.asarray(force, dtype=np.float32)
        # Native submission uses local position zero. Preserve an arbitrary
        # original local application point by adding its moment about zero.
        self.data[1, row] += np.cross(position, force)

    def torque(self, torque, body_part):
        self.data[1, self.rows[body_part]] += np.asarray(torque, dtype=np.float32)


def direct_vehicle_class(base, device='cuda', io_factory=None):
    """Create overrides before Vehicle.__init__ registers named callbacks."""
    class DirectMultirotor(base):
        def start(self):
            super().start()
            if getattr(self._thrusters, '_num_rotors', 4) != 4:
                raise ValueError('Direct Pegasus tensors currently require a four-rotor vehicle')
            factory = io_factory
            if factory is None:
                from pegasus_tensor_io import WarpTensorIO
                factory = WarpTensorIO
            self._direct_io = factory(self._stage_prefix, device=device)
            self._direct_wrenches = LocalWrenches(self._direct_io.body_parts)
            self._direct_propellers = {}
            self._direct_collecting = False
            print(f'[AS2] Direct Pegasus tensors: {self._stage_prefix}; '
                  f'device={device}; bodies={self._direct_io.body_parts}', flush=True)

        def stop(self):
            self._direct_io = None
            self._direct_collecting = False
            self._direct_propellers = {}
            super().stop()

        def update_state(self, dt, context=None):
            io = getattr(self, '_direct_io', None)
            if io is None:
                return super().update_state(dt, context)
            snapshot = io.read_body_state().astype(np.float64)
            previous_velocity = self._state.linear_velocity
            self._state.position = snapshot[:3].copy()
            self._state.attitude = snapshot[3:7].copy()  # native xyzw
            self._state.linear_velocity = snapshot[7:10].copy()
            inverse = Rotation.from_quat(self._state.attitude).inv()
            self._state.linear_body_velocity = inverse.apply(self._state.linear_velocity)
            self._state.angular_velocity = inverse.apply(snapshot[10:13])
            self._state.linear_acceleration = (
                (self._state.linear_velocity - previous_velocity) / dt if dt > 0 else np.zeros(3))

        def update(self, dt, context=None):
            if getattr(self, '_direct_io', None) is None:
                return super().update(dt, context)
            self._direct_wrenches.clear()
            self._direct_propellers.clear()
            self._direct_collecting = True
            try:
                result = super().update(dt, context)
            finally:
                self._direct_collecting = False
            self.flush_direct_tensors()
            return result

        def apply_force(self, force, pos=(0.0, 0.0, 0.0), body_part='/body'):
            if not getattr(self, '_direct_collecting', False):
                return super().apply_force(force, pos, body_part)
            self._direct_wrenches.force(force, pos, body_part)

        def apply_torque(self, torque, body_part='/body'):
            if not getattr(self, '_direct_collecting', False):
                return super().apply_torque(torque, body_part)
            self._direct_wrenches.torque(torque, body_part)

        def handle_propeller_visual(self, rotor_number, force):
            if not getattr(self, '_direct_collecting', False):
                return super().handle_propeller_visual(rotor_number, force)
            if self._articulation is None or not self._articulation.is_physics_tensor_entity_valid():
                return
            index = self._get_rotor_dof_index(rotor_number)
            if index is None:
                return
            speed = 5.0 if 0.0 < force < 0.1 else (100.0 if force >= 0.1 else 0.0)
            self._direct_propellers[index] = speed * self._thrusters.rot_dir[rotor_number]

        def flush_direct_tensors(self):
            self._direct_io.apply_wrenches(self._direct_wrenches.data)
            if self._direct_propellers:
                indices = list(self._direct_propellers)
                self._articulation.set_dof_velocities(
                    [[self._direct_propellers[index] for index in indices]], dof_indices=indices)

    return DirectMultirotor
