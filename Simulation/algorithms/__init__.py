"""
Algorithm scripts live here. Each script imports shared infrastructure like this:

    import sys
    from pathlib import Path

    # Add Simulation/ to path so config and connection_test are importable
    sys.path.insert(0, str(Path(__file__).parent.parent))

    import config
    from connection_test import connect_drone, read_telemetry, connect_drone_ctx

Minimal working example:

    import asyncio, sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from connection_test import connect_drone_ctx, read_telemetry

    async def main():
        async with connect_drone_ctx() as drone:
            telem = await read_telemetry(drone)
            print(telem)

    asyncio.run(main())

No package installation required — sys.path is the convention used across
this repo (consistent with Code/Callibration/ scripts).
"""
