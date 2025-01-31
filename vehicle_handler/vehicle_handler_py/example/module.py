# Copyright 2024 Maximilian Leitenstern

import vehicle_handler_py as vh
import param_management_py as pmg
import tum_types_py


class SoftwareModule:
    def __init__(self):
        self.vehicle_ = vh.VehicleHandler.from_pkg_config()
        self.module_pmg_ = pmg.ParamValueManager()
        self.set_params()

    def calc_dynamics(self):
        velocity_vector = tum_types_py.common.Vector3D(75.0, 1.0, 0.0)
        tire_radii = self.vehicle_.calc_dynamic_tire_radius(velocity_vector)
        print(
            f"Dynamic tire radii: {tire_radii.front_left}, {tire_radii.front_right}, "
            f"{tire_radii.rear_left}, {tire_radii.rear_right}"
        )

    def vehicle_handler(self):
        return self.vehicle_

    def param_manager(self):
        return self.module_pmg_

    def set_params(self):
        # fmt: off
        self.module_pmg_.declare_parameter("vehicle.dimension.track_width_front", 0.0, pmg.ParameterType.DOUBLE, "")
        self.module_pmg_.declare_parameter("vehicle.dimension.track_width_rear", 0.0, pmg.ParameterType.DOUBLE, "")
        self.module_pmg_.declare_parameter("vehicle.dimension.cog_height", 0.0, pmg.ParameterType.DOUBLE, "")
        self.vehicle_.load_params(self.param_manager())
        # fmt: on


if __name__ == "__main__":
    module = SoftwareModule()
    print(
        f"Track width front: {module.vehicle_handler().vehicle().dimension.track_width_front}"
    )
    print(
        f"Track width rear: {module.param_manager().get_value('vehicle.dimension.track_width_rear').as_double()}"
    )
    module.calc_dynamics()
