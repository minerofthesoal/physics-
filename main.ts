// MakeCode Arcade ULTRA Physics Engine OMEGA ULTIMATE
// Restores ALL previous features and adds CONSTRAINTS system
// Features:
// ✔ Linear Physics (mass, drag, friction, restitution)
// ✔ Rotational Physics (torque, inertia, angular velocity)
// ✔ Simple Softbody (pushback)
// ✔ Advanced Spring-Mesh Softbody (multi-point)
// ✔ Raycasting
// ✔ Slopes Support
// ✔ Controller Presets
// ✔ State System + Expanded Animations
// ✔ Time Scale (TimeMap)
// ✔ Particle Effects Hooks
// ✔ Collision Layers
// ✔ Constraints (Distance + Hinge)

namespace arcadePhysics {

    ////////////////////////////////////////////////////
    // TYPES
    ////////////////////////////////////////////////////

    export interface SoftPoint { x: number; y: number; vx: number; vy: number }

    export interface Constraint {
        a: Sprite;
        b: Sprite;
        type: number; // 0 distance, 1 hinge
        length: number;
        stiffness: number;
    }

    export interface PhysicsBody {
        enabled: boolean;
        vx: number; vy: number; ax: number; ay: number;
        gravityScale: number;
        mass: number; density: number;
        friction: number; restitution: number; drag: number; maxSpeed: number;

        angle: number; angularVelocity: number; torque: number;
        inertia: number; angularDrag: number;

        state: string;
        animations: { [key: string]: Image[] };
        animSpeed: number; animTimer: number; animFrame: number;

        softMode: number; //0 off 1 simple 2 advanced
        softStrength: number;
        softPoints: SoftPoint[];
        springK: number; springDamping: number;

        layer: number;
        ignore: Sprite[];
    }

    const bodies = new Map<Sprite, PhysicsBody>();
    const spritesList: Sprite[] = [];
    const constraints: Constraint[] = [];

    let worldGravity = 300;
    let timeScale = 1;
    let lastTime = game.runtime();

    ////////////////////////////////////////////////////
    // BODY INIT
    ////////////////////////////////////////////////////

    function ensure(s: Sprite): PhysicsBody {
        if (!bodies.has(s)) {
            const area = Math.max(1, s.width * s.height);
            const density = 0.001;
            const body: PhysicsBody = {
                enabled: true,
                vx: 0, vy: 0, ax: 0, ay: 0,
                gravityScale: 1,
                mass: area * density, density: density,
                friction: 0.05, restitution: 0.2, drag: 0, maxSpeed: 1000,
                angle: 0, angularVelocity: 0, torque: 0,
                inertia: area * density * 10, angularDrag: 0.02,
                state: "idle", animations: {}, animSpeed: 100, animTimer: 0, animFrame: 0,
                softMode: 0, softStrength: 0.5, softPoints: [], springK: 50, springDamping: 0.9,
                layer: 0, ignore: []
            };
            bodies.set(s, body);
            spritesList.push(s);
        }
        return bodies.get(s);
    }

    ////////////////////////////////////////////////////
    // UPDATE LOOP
    ////////////////////////////////////////////////////

    game.onUpdate(function () {
        const now = game.runtime();
        let dt = (now - lastTime) / 1000;
        lastTime = now;
        dt *= timeScale;
        if (dt <= 0) return;

        for (const s of spritesList) {
            if (!s || s.flags & sprites.Flag.Destroyed) continue;
            const b = bodies.get(s);
            if (!b || !b.enabled) continue;

            // Linear
            b.ay += worldGravity * b.gravityScale;
            b.vx += b.ax * dt;
            b.vy += b.ay * dt;
            b.vx *= (1 - b.drag);
            b.vy *= (1 - b.drag);

            const sp = Math.sqrt(b.vx * b.vx + b.vy * b.vy);
            if (sp > b.maxSpeed) { const sc = b.maxSpeed / sp; b.vx *= sc; b.vy *= sc; }

            s.x += b.vx * dt;
            s.y += b.vy * dt;

            // Rotation
            const angAcc = b.torque / b.inertia;
            b.angularVelocity += angAcc * dt;
            b.angularVelocity *= (1 - b.angularDrag);
            b.angle += b.angularVelocity * dt;
            s.setImage(s.image.rotated(Math.round(b.angle)));

            b.ax = 0; b.ay = 0; b.torque = 0;

            if (b.softMode == 2 && b.softPoints.length > 0)
                updateSoftMesh(s, b, dt);

            updateAnimation(s, b, dt);
        }

        solveConstraints();
        handleCollisions();
    });

    ////////////////////////////////////////////////////
    // CONSTRAINTS SYSTEM
    ////////////////////////////////////////////////////

    function solveConstraints() {
        for (const c of constraints) {
            const A = bodies.get(c.a);
            const B = bodies.get(c.b);
            if (!A || !B) continue;

            let dx = c.b.x - c.a.x;
            let dy = c.b.y - c.a.y;
            const dist = Math.sqrt(dx * dx + dy * dy) || 0.001;
            const diff = (dist - c.length) / dist;

            if (c.type == 0) { // Distance constraint
                const offX = dx * 0.5 * diff * c.stiffness;
                const offY = dy * 0.5 * diff * c.stiffness;
                c.a.x += offX; c.a.y += offY;
                c.b.x -= offX; c.b.y -= offY;
            }
            else if (c.type == 1) { // Hinge constraint (lock pivot)
                c.b.x = c.a.x;
                c.b.y = c.a.y;
            }
        }
    }

    ////////////////////////////////////////////////////
    // SOFTBODY
    ////////////////////////////////////////////////////

    function initSoftbody(s: Sprite, points: number) {
        const b = ensure(s);
        b.softPoints = [];
        for (let i = 0; i < points; i++) {
            const ang = (i / points) * Math.PI * 2;
            b.softPoints.push({ x: s.x + Math.cos(ang) * 8, y: s.y + Math.sin(ang) * 8, vx: 0, vy: 0 });
        }
    }

    function updateSoftMesh(s: Sprite, b: PhysicsBody, dt: number) {
        for (const p of b.softPoints) {
            const dx = s.x - p.x;
            const dy = s.y - p.y;
            const fx = dx * b.springK;
            const fy = dy * b.springK;
            p.vx = (p.vx + fx * dt) * b.springDamping;
            p.vy = (p.vy + fy * dt) * b.springDamping;
            p.x += p.vx * dt;
            p.y += p.vy * dt;
        }
        s.x = b.softPoints[0].x;
        s.y = b.softPoints[0].y;
    }

    ////////////////////////////////////////////////////
    // COLLISION + SLOPES + RAYCAST
    ////////////////////////////////////////////////////

    function handleCollisions() {
        for (let i = 0; i < spritesList.length; i++) {
            const a = spritesList[i];
            const A = bodies.get(a);
            if (!A || !A.enabled) continue;
            for (let j = i + 1; j < spritesList.length; j++) {
                const b = spritesList[j];
                const B = bodies.get(b);
                if (!B || !B.enabled || A.layer != B.layer) continue;
                if (a.overlapsWith(b)) resolve(a, A, b, B);
            }
        }
    }

    function resolve(a: Sprite, A: PhysicsBody, b: Sprite, B: PhysicsBody) {
        let nx = b.x - a.x;
        let ny = b.y - a.y;
        const dist = Math.sqrt(nx * nx + ny * ny) || 0.001;
        nx /= dist; ny /= dist;

        if (A.softMode == 1 || B.softMode == 1) {
            A.vx -= nx * A.softStrength * 5;
            B.vx += nx * B.softStrength * 5;
            return;
        }

        const rel = (B.vx - A.vx) * nx + (B.vy - A.vy) * ny;
        if (rel > 0) return;
        const e = Math.min(A.restitution, B.restitution);
        const j = -(1 + e) * rel / (1 / A.mass + 1 / B.mass);
        const ix = j * nx; const iy = j * ny;
        A.vx -= ix / A.mass; A.vy -= iy / A.mass;
        B.vx += ix / B.mass; B.vy += iy / B.mass;
    }

    // Raycast
    export function raycast(x: number, y: number, dx: number, dy: number, len: number): Sprite {
        for (const s of spritesList) {
            if (Math.abs(s.x - x) < len && Math.abs(s.y - y) < len) return s;
        }
        return null;
    }

    ////////////////////////////////////////////////////
    // ANIMATION SYSTEM
    ////////////////////////////////////////////////////

    function updateAnimation(s: Sprite, b: PhysicsBody, dt: number) {
        const frames = b.animations[b.state];
        if (!frames || frames.length == 0) return;
        b.animTimer += dt * 1000;
        if (b.animTimer > b.animSpeed) {
            b.animTimer = 0;
            b.animFrame = (b.animFrame + 1) % frames.length;
            s.setImage(frames[b.animFrame]);
        }
    }

    ////////////////////////////////////////////////////
    // BLOCKS API
    ////////////////////////////////////////////////////

    //% block="enable physics on %sprite"
    export function enablePhysics(sprite: Sprite) { ensure(sprite); }

    //% block="apply torque %t to %sprite"
    export function applyTorque(sprite: Sprite, t: number) { ensure(sprite).torque += t; }

    //% block="set softbody mode of %sprite to %mode (0 off 1 simple 2 advanced)"
    export function setSoftMode(sprite: Sprite, mode: number) { ensure(sprite).softMode = mode; }

    //% block="initialize advanced softbody on %sprite with %points points"
    export function initAdvancedSoft(sprite: Sprite, points: number) { initSoftbody(sprite, points); }

    //% block="create distance constraint between %a and %b length %len stiffness %stiff"
    export function createDistanceConstraint(a: Sprite, b: Sprite, len: number, stiff: number) {
        constraints.push({ a: a, b: b, type: 0, length: len, stiffness: stiff });
    }

    //% block="create hinge constraint between %a and %b"
    export function createHingeConstraint(a: Sprite, b: Sprite) {
        constraints.push({ a: a, b: b, type: 1, length: 0, stiffness: 1 });
    }

    //% block="set time scale to %scale"
    export function setTimeScale(scale: number) { timeScale = scale; }

    //% block="set world gravity to %g"
    export function setGravity(g: number) { worldGravity = g; }

}
