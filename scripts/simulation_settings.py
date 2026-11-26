"""Apply explicit physics-thread and render overrides without changing the world."""


def apply_simulation_settings(settings, simulation):
    threads = simulation.get('physics_threads', -1)
    if threads >= 0:
        settings.set_int('/persistent/physics/numThreads', threads)
    if simulation.get('render_quality', 'current') == 'fast':
        settings.set('/rtx/rendermode', 'RaytracedLighting')
        settings.set_bool('/rtx/reflections/enabled', False)
        settings.set_bool('/rtx/indirectDiffuse/enabled', False)
        settings.set_bool('/rtx/ambientOcclusion/enabled', False)
    print(f"SIM_SETTINGS physics_device={simulation.get('physics_device', 'cuda')} "
          f"physics_hz={simulation.get('physics_hz', 250)} "
          f"physics_threads={settings.get('/persistent/physics/numThreads')} "
          f"profile={simulation.get('profile', False)} "
          f"render_quality={simulation.get('render_quality', 'current')} "
          f"direct_tensors={simulation.get('direct_tensors', False)}", flush=True)
