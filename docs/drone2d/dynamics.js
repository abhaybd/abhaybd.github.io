// Configuration classes
class PhysicsConfig {
    constructor({
        max_target_dist,
        gravity,
        max_thrust,
        max_init_vel,
        max_init_angular_vel,
        max_init_angle_from_vert,
        thrust_dist,
        mass,
        inertia
    } = {}) {
        this.maxTargetDist = max_target_dist;
        this.gravity = gravity;
        this.maxThrust = max_thrust;
        this.maxInitVel = max_init_vel;
        this.maxInitAngularVel = max_init_angular_vel;
        this.maxInitAngleFromVert = max_init_angle_from_vert;
        this.thrustDist = thrust_dist;
        this.mass = mass;
        this.inertia = inertia;
    }
}

class Drone2DEnvConfig {
    constructor({
        dt = 0.05,
        physics = new PhysicsConfig()
    } = {}) {
        this.dt = dt;
        this.physics = physics;
    }
}

class Drone2DEnv {
    constructor(config = new Drone2DEnvConfig()) {
        this.config = config;
        this.state = new Float32Array(7);  // [pos_x, pos_y, vel_x, vel_y, cos_theta, sin_theta, ang_vel]
    }

    getState() {
        return new Float32Array(this.state);
    }

    setState(state) {
        this.state.set(state);
    }

    step(action, dt) {
        dt = dt ?? this.config.dt;
        const thrustLeft = Math.max(0, Math.min(action[0], this.config.physics.maxThrust));
        const thrustRight = Math.max(0, Math.min(action[1], this.config.physics.maxThrust));

        const upVector = [-this.state[5], this.state[4]];  // Rotate CCW 90 degrees
        const thrustForce = [
            (thrustLeft + thrustRight) * upVector[0],
            (thrustLeft + thrustRight) * upVector[1]
        ];
        const gravityForce = [0, -this.config.physics.gravity * this.config.physics.mass];
        const netForce = [
            thrustForce[0] + gravityForce[0],
            thrustForce[1] + gravityForce[1]
        ];
        const netTorque = (thrustRight - thrustLeft) * this.config.physics.thrustDist;

        const accel = [
            netForce[0] / this.config.physics.mass,
            netForce[1] / this.config.physics.mass
        ];
        const angularAccel = netTorque / this.config.physics.inertia;

        this.state[0] += this.state[2] * dt + 0.5 * accel[0] * dt * dt;
        this.state[1] += this.state[3] * dt + 0.5 * accel[1] * dt * dt;

        this.state[2] += accel[0] * dt;
        this.state[3] += accel[1] * dt;
        this.state[2] = Math.max(-this.config.physics.maxInitVel, 
                                Math.min(this.config.physics.maxInitVel, this.state[2]));
        this.state[3] = Math.max(-this.config.physics.maxInitVel, 
                                Math.min(this.config.physics.maxInitVel, this.state[3]));

        const avgThetaVel = this.state[6] + 0.5 * angularAccel * dt;
        const theta = Math.atan2(this.state[5], this.state[4]);
        const newTheta = theta + avgThetaVel * dt;
        this.state[4] = Math.cos(newTheta);
        this.state[5] = Math.sin(newTheta);

        this.state[6] += angularAccel * dt;
        this.state[6] = Math.max(-this.config.physics.maxInitAngularVel, 
                                Math.min(this.config.physics.maxInitAngularVel, this.state[6]));

        return this.getState();
    }
}

// Export the classes
export { PhysicsConfig, Drone2DEnvConfig, Drone2DEnv };
