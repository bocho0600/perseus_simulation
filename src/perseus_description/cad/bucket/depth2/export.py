"""Re-export of the bucket at a chosen cad max_depth (the original to_urdf_test.py
used 0). Usage:  python3 export.py 2

RUN THIS FROM A SCRATCH DIRECTORY, with the .env copied in. Merely importing
onshape_robotics_toolkit rewrites ORT.yaml in the current working directory, which
truncates the real one if you run it from ~/Documents/projects/onshape-urdf.
Read-only on Onshape. Writes out<depth>/bucket.urdf and out<depth>/meshes/*.stl.
"""
import sys
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.formats.urdf import URDFSerializer
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities import setup_default_logging

depth = int(sys.argv[1])
URL = "https://qutrc.onshape.com/documents/6a8e1bd7c7e79e401b258e58/w/9300f80289988f1c228085b6/e/92807cb0d7b4a1838620ec3e"
setup_default_logging(file_path=f"d{depth}.log", console_level="WARNING")
client = Client(env=".env")
cad = CAD.from_url(URL, client=client, max_depth=depth)
graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
robot = Robot.from_graph(kinematic_graph=graph, client=client, name="bucket")
URDFSerializer().save(robot, f"out{depth}/bucket.urdf", download_assets=True, mesh_dir=f"out{depth}/meshes")
print("EXPORT_OK depth", depth)
