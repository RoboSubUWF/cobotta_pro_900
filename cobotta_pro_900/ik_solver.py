from omni.isaac.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.nucleus import get_assets_root_path
from typing import Optional
import os

class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(self, robot_articulation: Articulation, end_effector_frame_name: Optional[str] = None) -> None:
        #TODO: change the config path
        # COBOTTA_SCRIPTS_PATH
        # Load a URDF and Lula Robot Description File for this robot:
        assets_root_path = get_assets_root_path()
        mg_extension_path = get_extension_path_from_name("omni.importer.urdf-1.14.1+106.0.0.lx64.r.cp310")
        kinematics_config_dir = os.path.join(mg_extension_path, "/data/urdf/robots/cobotta_pro_900")
        # self._kinematics = LulaKinematicsSolver(robot_description_path= kinematics_config_dir +"/cobotta_pro_900/rmpflow/robot_descriptor.yaml",
        #                                         urdf_path= kinematics_config_dir + "/cobotta_pro_900.urdf")
        self._kinematics = LulaKinematicsSolver(robot_description_path= "/home/ubuntu/.local/share/ov/pkg/isaac-sim-4.2.0/extscache/omni.importer.urdf-1.14.1+106.0.0.lx64.r.cp310/data/urdf/robots/cobotta_pro_900/cobotta_pro_900/rmpflow/robot_descriptor.yaml",
                                                urdf_path= "/home/ubuntu/.local/share/ov/pkg/isaac-sim-4.2.0/extscache/omni.importer.urdf-1.14.1+106.0.0.lx64.r.cp310/data/urdf/robots/cobotta_pro_900/cobotta_pro_900.urdf")
        if end_effector_frame_name is None:
            end_effector_frame_name = "onrobot_rg6_base_link"
        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)
        return
