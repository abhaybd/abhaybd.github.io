import { Drone2DEnv, Drone2DEnvConfig, PhysicsConfig } from "./dynamics.js";

const PX_PER_M = 96.0 * 1.0; // px per display inch * display inch per env meter

const DRONE_DIMENSIONS = {
    width: 0.5,
    height: 0.15,
    axle_width: 0.02,
    axle_height: 0.075,
    axle_margin: 0.05,
    propeller_radius: 0.15,
    propeller_thickness: 0.02,
    propeller_period: 0.25,
}

const mousePosPx = {x: null, y: null};

const drawDrone = (/** @type {CanvasRenderingContext2D} */ ctx) => {
    const {width, height, axle_width, axle_height, axle_margin, propeller_radius, propeller_thickness, propeller_period} = DRONE_DIMENSIONS;

    ctx.fillStyle = getComputedStyle(document.body).color;
    ctx.fillRect(-width/2, -height/2, width, height);

    ctx.fillRect(-width/2 + axle_margin, -height/2, axle_width, axle_height + height);
    ctx.fillRect(width/2 - axle_margin - axle_width, -height/2, axle_width, axle_height + height);

    const t = Date.now() / 1000;
    const propellerRadius = Math.abs(propeller_radius * Math.sin(t * 2 * Math.PI / propeller_period));
    ctx.fillRect(-width/2 + axle_margin + axle_width/2 - propellerRadius, height/2 + axle_height - propeller_thickness, propellerRadius*2, propeller_thickness);
    ctx.fillRect(width/2 - axle_margin - axle_width/2 - propellerRadius, height/2 + axle_height - propeller_thickness, propellerRadius*2, propeller_thickness);
}

class DroneAnimation {
    constructor(/** @type {HTMLCanvasElement} */ canvas, /** @type {ort.InferenceSession} */ ort_session, /** @type {Drone2DEnv} */ env) {
        this.canvas = canvas;
        this.ort_session = ort_session;
        this.env = env;
        this.last_timestamp = null;
    }

    async animate(timestamp) {
        if (this.last_timestamp === null) {
            this.last_timestamp = timestamp;
        } else {
            const dt = (timestamp - this.last_timestamp) / 1000;
            this.last_timestamp = timestamp;

            const canvas = this.canvas;
            const ort_session = this.ort_session;
            const env = this.env;

            const mousePosX = (mousePosPx.x - window.innerWidth/2) / PX_PER_M;
            const mousePosY = (window.innerHeight/2 - mousePosPx.y) / PX_PER_M;

            let state = env.getState();
            state[0] = mousePosX - state[0];
            state[1] = mousePosY - state[1];

            const data = {"states": new ort.Tensor('float32', state, [1, 7])};
            const result = await ort_session.run(data);
            const action = result["actions"].data;
            for (let i = 0; i < action.length; i++) {
                action[i] = (action[i] + 1) / 2 * env.config.physics.maxThrust;
            }
            state = env.step(action, dt);

            canvas.width = window.innerWidth;
            canvas.height = window.innerHeight;
            const ctx = canvas.getContext("2d");
            ctx.resetTransform();
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.setTransform(PX_PER_M, 0, 0, -PX_PER_M, canvas.width/2, canvas.height/2);
            ctx.transform(state[4], state[5], -state[5], state[4], state[0], state[1]);
            drawDrone(ctx);
        }

        requestAnimationFrame(this.animate.bind(this));
    }
}

const sleep = async (ms) => {
    return new Promise(resolve => setTimeout(resolve, ms));
}

const revealDrone = async (/** @type {HTMLCanvasElement} */ canvas, /** @type {Drone2DEnv} */ env) => {
    const ctx = canvas.getContext("2d");
    // fly on screen from below
    await (async () => {
        const trajectory = [
            {t: 0, x: canvas.width*0.9, y: canvas.height + 2 * PX_PER_M},
            {t: 1, x: canvas.width*0.9, y: canvas.height - 1 * PX_PER_M},
            {t: 2, x: canvas.width*0.9, y: canvas.height - 0.5 * PX_PER_M}
        ]
        for (let i = 0; i < trajectory.length-1; i++) {
            const start = trajectory[i];
            const end = trajectory[i+1];
            const duration = end.t - start.t;
    
            const startTime = performance.now();
            while ((performance.now() - startTime) / 1000 < duration) {
                const t = Math.min((performance.now() - startTime) / 1000 / duration, 1);
                const easedT = 0.5 * (1 - Math.cos(t * Math.PI));
                
                const x = start.x + (end.x - start.x) * easedT;
                const y = start.y + (end.y - start.y) * easedT;
                
                ctx.resetTransform();
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                ctx.setTransform(PX_PER_M, 0, 0, -PX_PER_M, x, y);
                drawDrone(ctx);
                
                await sleep(16); // ~60fps
            }
        }
    })();

    // show dialog while hovering in place
    await (async () => {
        const startTime = performance.now();
        const fadeInAnimDur = 2.0;
        const fadeInTime = 1.0;
        const fadeOutAnimDur = 5.0;
        const fadeOutTime = 1.0;
        const trf = ctx.getTransform();
        while ((performance.now() - startTime) / 1000 < fadeInAnimDur + fadeOutAnimDur || mousePosPx.x === null || mousePosPx.y === null) {
            ctx.resetTransform();
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.setTransform(trf);
            drawDrone(ctx);

            const t = (performance.now() - startTime) / 1000;

            ctx.save();
            if (t < fadeInAnimDur) {
                ctx.globalAlpha = Math.min(1, Math.max(0, t - fadeInAnimDur + fadeInTime) / fadeInTime);
            } else {
                ctx.globalAlpha = Math.min(1, Math.max(0, fadeOutAnimDur - t + fadeOutTime) / fadeOutTime);
            }
            ctx.beginPath();
            ctx.fillStyle = getComputedStyle(document.body).color;
            ctx.ellipse(-2.5, 1, 2.0, 0.5, 0, 0, 2 * Math.PI);
            ctx.fill();

            ctx.beginPath();
            ctx.moveTo(-1, 0.7);
            ctx.lineTo(-0.5, 0.25);
            ctx.lineTo(-0.85, 0.75);
            ctx.fill();

            ctx.resetTransform();
            ctx.font = "18px Arial";
            ctx.fillStyle = getComputedStyle(document.body).backgroundColor;
            const pos_trf = trf.transformPoint(new DOMPoint(-4.2, 0.95));
            ctx.fillText("I was trained with reinforcement learning!", pos_trf.x, pos_trf.y);

            ctx.restore();

            await sleep(16); // ~60fps
        }
    })();

    const trf = ctx.getTransform();
    const pos_px = trf.transformPoint(new DOMPoint(0, 0));
    const x = (pos_px.x - canvas.width/2) / PX_PER_M;
    const y = (canvas.height/2 - pos_px.y) / PX_PER_M;
    env.setState(new Float32Array([x, y, 0.0, 0.0, 1.0, 0.0, 0.0]));
}

const initDrone = async () => {
    const config_str = await (await fetch("drone2d/policy/config.yaml")).text();
    const config = jsyaml.load(config_str);
    
    const envConfig = new Drone2DEnvConfig({dt: config.dt, physics: new PhysicsConfig(config.env.physics)});
    const env = new Drone2DEnv(envConfig);

    const session = await ort.InferenceSession.create("drone2d/policy/policy.onnx");
    
    const body = document.body;
    const canvas = document.createElement("canvas");
    canvas.style.position = "fixed";
    canvas.style.top = "0";
    canvas.style.left = "0";
    canvas.style.width = "100%";
    canvas.style.height = "100%";
    canvas.style.zIndex = "1000";
    canvas.style.pointerEvents = "none";
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    body.appendChild(canvas);

    await revealDrone(canvas, env);

    const animation = new DroneAnimation(canvas, session, env);
    requestAnimationFrame(animation.animate.bind(animation));
}

(() => {
    if (matchMedia("(pointer: fine)").matches) {
        document.documentElement.addEventListener("mousemove", (e) => {
            mousePosPx.x = e.clientX;
            mousePosPx.y = e.clientY;
        });

        setTimeout(initDrone, 5000);
    } else {
        console.log("No fine pointer detected, won't show drone");
    }
})();
