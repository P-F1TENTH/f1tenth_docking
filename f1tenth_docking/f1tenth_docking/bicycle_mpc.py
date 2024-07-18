import do_mpc


class BicycleMPC(do_mpc.controller.MPC):
    """Model Predictive Controller with objective function and constraints for the F1/10th car"""

    def __init__(
        self,
        model: do_mpc.model.Model,
        t_step: float,
        solver_max_cpu_time: float,
        n_horizon: int,
        gains: dict,
        bounds: dict,
    ) -> None:

        super().__init__(model)

        self.set_param(
            n_horizon=n_horizon,
            t_step=t_step,
            store_full_solution=False,
            nlpsol_opts={
                "ipopt.max_cpu_time": solver_max_cpu_time,
                "ipopt.print_level": 0,
                "ipopt.sb": "yes",
                "print_time": 0,
            },
        )

        self.bounds["lower", "_x", "x_pos"] = bounds["x_pos"]["lower"]
        self.bounds["upper", "_x", "y_pos"] = bounds["x_pos"]["upper"]

        self.bounds["lower", "_x", "y_pos"] = bounds["y_pos"]["lower"]
        self.bounds["upper", "_x", "y_pos"] = bounds["y_pos"]["upper"]

        self.bounds["lower", "_x", "delta"] = -bounds["delta"]["radians"]
        self.bounds["upper", "_x", "delta"] = bounds["delta"]["radians"]

        self.bounds["lower", "_u", "v"] = bounds["v"]["lower"] * t_step
        self.bounds["upper", "_u", "v"] = bounds["v"]["upper"] * t_step

        self.bounds["lower", "_u", "phi"] = bounds["phi"]["lower"] * t_step
        self.bounds["upper", "_u", "phi"] = bounds["phi"]["upper"] * t_step

        norm = lambda x, min, max: (x - min) / (max - min)
        lterm = (
            (
                norm(
                    (model.x["x_pos"] - model.tvp["set_x_pos"]),
                    bounds["x_pos"]["lower"],
                    bounds["x_pos"]["upper"],
                )
                * gains["pos"]
            )
            ** 2
            + (
                norm(
                    (model.x["y_pos"] - model.tvp["set_y_pos"]),
                    bounds["y_pos"]["lower"],
                    bounds["y_pos"]["upper"],
                )
                * gains["pos"]
            )
            ** 2
            + ((model.x["theta"] - model.tvp["set_theta"]) * gains["theta"]) ** 2
            + ((model.x["delta"] - model.tvp["set_delta"]) * gains["delta"]) ** 2
        )
        mterm = lterm
        self.set_objective(mterm=mterm, lterm=lterm)
        self.set_rterm(v=1e-2, phi=1e-2)

        self.tvp_template = self.get_tvp_template()
        self.set_tvp_fun(lambda _: self.tvp_template)
        self.setup()

    def choose_setpoint(
        self, x_pos: float, y_pos: float, theta: float, delta: float
    ) -> None:
        """Choose setpoint for the MPC controller"""
        self.tvp_template["_tvp", 0 : self.settings.n_horizon + 1, "set_x_pos"] = x_pos
        self.tvp_template["_tvp", 0 : self.settings.n_horizon + 1, "set_y_pos"] = y_pos
        self.tvp_template["_tvp", 0 : self.settings.n_horizon + 1, "set_theta"] = theta
        self.tvp_template["_tvp", 0 : self.settings.n_horizon + 1, "set_delta"] = delta
