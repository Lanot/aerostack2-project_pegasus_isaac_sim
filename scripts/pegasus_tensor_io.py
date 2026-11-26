"""Preallocated Warp IO for Pegasus and Isaac Sim 6.2 experimental RigidPrim."""
import numpy as np
import warp as wp


@wp.kernel
def pack_body_state(transforms: wp.array2d(dtype=wp.float32),
                    velocities: wp.array2d(dtype=wp.float32),
                    body: int, result: wp.array2d(dtype=wp.float32)):
    column = wp.tid()
    if column < 7:
        result[0, column] = transforms[body, column]
    else:
        result[0, column] = velocities[body, column - 7]


@wp.kernel
def unpack_wrenches(packed: wp.array3d(dtype=wp.float32),
                    forces: wp.array2d(dtype=wp.float32),
                    torques: wp.array2d(dtype=wp.float32)):
    body, axis = wp.tid()
    forces[body, axis] = packed[0, body, axis]
    torques[body, axis] = packed[1, body, axis]


class WarpTensorIO:
    """Reuse native body views, with one state readback and one wrench upload."""

    def __init__(self, prefix, device='cuda', rigid_prim_class=None):
        if rigid_prim_class is None:
            from isaacsim.core.experimental.prims import RigidPrim
            rigid_prim_class = RigidPrim
        paths = [prefix + part for part in ['/body', '/rotor0', '/rotor1', '/rotor2', '/rotor3']]
        self.rigid = rigid_prim_class(paths=paths, resolve_paths=False)
        self.view = getattr(self.rigid, '_physics_rigid_body_view', None)
        if self.view is None or not self.rigid.is_physics_tensor_entity_valid():
            raise RuntimeError('Direct tensors require an initialized Isaac Sim 6.2 Warp RigidPrim view')
        native_paths = list(self.view.prim_paths)
        if len(native_paths) != 5 or set(native_paths) != set(paths):
            raise RuntimeError(f'Unexpected Pegasus direct tensor body paths: {native_paths}')
        # Do not assume native views preserve the caller's requested path order.
        self.body_parts = [path[len(prefix):] for path in native_paths]
        self.body_index = self.body_parts.index('/body')
        self.device = wp.get_device(device)
        self.count = len(native_paths)
        self.indices = wp.array(np.arange(self.count, dtype=np.int32), dtype=wp.int32, device=self.device)
        self.state_device = wp.empty((1, 13), dtype=wp.float32, device=self.device)
        self.state_host = wp.empty((1, 13), dtype=wp.float32, device='cpu', pinned=self.device.is_cuda)
        self.packed_device = wp.empty((2, self.count, 3), dtype=wp.float32, device=self.device)
        self.packed_host = wp.empty((2, self.count, 3), dtype=wp.float32,
                                    device='cpu', pinned=self.device.is_cuda)
        self.host_array = self.packed_host.numpy()
        self.forces = wp.empty((self.count, 3), dtype=wp.float32, device=self.device)
        self.torques = wp.empty((self.count, 3), dtype=wp.float32, device=self.device)
        self.positions = wp.zeros((self.count, 3), dtype=wp.float32, device=self.device)

    def read_body_state(self):
        wp.launch(pack_body_state, dim=13,
                  inputs=[self.view.get_transforms(), self.view.get_velocities(), self.body_index],
                  outputs=[self.state_device], device=self.device)
        wp.copy(self.state_host, self.state_device)
        if self.device.is_cuda:
            wp.synchronize_device(self.device)
        return self.state_host.numpy()[0].copy()

    def apply_wrenches(self, data):
        np.copyto(self.host_array, data)
        wp.copy(self.packed_device, self.packed_host)
        wp.launch(unpack_wrenches, dim=(self.count, 3), inputs=[self.packed_device],
                  outputs=[self.forces, self.torques], device=self.device)
        self.view.apply_forces_and_torques_at_position(
            self.forces, self.torques, self.positions, self.indices, False)
