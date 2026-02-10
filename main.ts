//% color="#E63022" weight=100 icon="\uf1e3" block="Jelly Physics V2"
//% groups='["World", "Bodies", "Construction", "Materials", "Joints", "Interaction", "Advanced"]'
namespace jelly {

    // ==================================================================================
    // CONSTANTS & CONFIG
    // ==================================================================================

    const DEFAULT_RADIUS = 5;
    const DEFAULT_MASS = 1.0;
    const DEFAULT_STIFFNESS = 1.0;
    const DEFAULT_FRICTION = 0.99;
    const DEFAULT_BOUNCE = 0.8;
    const MAX_ITERATIONS = 16; // Safety cap
    const BREAK_THRESHOLD_OFF = -1;

    // ==================================================================================
    // MATH HELPERS
    // ==================================================================================

    class Vec2 {
        constructor(public x: number, public y: number) { }

        static distSq(v1: { x: number, y: number }, v2: { x: number, y: number }): number {
            const dx = v1.x - v2.x;
            const dy = v1.y - v2.y;
            return dx * dx + dy * dy;
        }

        static dist(v1: { x: number, y: number }, v2: { x: number, y: number }): number {
            return Math.sqrt(Vec2.distSq(v1, v2));
        }
    }

    // ==================================================================================
    // CORE CLASSES
    // ==================================================================================

    /**
     * A single node in the physics simulation.
     * Represents a mass point that handles position, velocity (implicit), and collisions.
     */
    export class Point {
        x: number;
        y: number;
        oldx: number;
        oldy: number;
        vx: number;
        vy: number;
        pinned: boolean;
        radius: number;
        mass: number;
        gravityScale: number;
        forceX: number;
        forceY: number;
        id: number;

        // Custom user data
        tag: number;

        constructor(x: number, y: number) {
            this.x = x;
            this.y = y;
            this.oldx = x;
            this.oldy = y;
            this.vx = 0;
            this.vy = 0;
            this.pinned = false;
            this.radius = DEFAULT_RADIUS;
            this.mass = DEFAULT_MASS;
            this.gravityScale = 1.0;
            this.forceX = 0;
            this.forceY = 0;
            this.tag = 0;
            this.id = Math.random();
        }

        /**
         * Standard Verlet Integration step.
         * Updates position based on previous position and applied forces.
         */
        update(gravityX: number, gravityY: number, friction: number) {
            if (this.pinned) return;

            // Calculate velocity from history
            this.vx = (this.x - this.oldx) * friction;
            this.vy = (this.y - this.oldy) * friction;

            // F = ma -> a = F/m
            let ax = this.forceX / this.mass;
            let ay = this.forceY / this.mass;

            // Apply Gravity
            ax += gravityX * this.gravityScale;
            ay += gravityY * this.gravityScale;

            // Save current pos as old pos
            this.oldx = this.x;
            this.oldy = this.y;

            // Apply Verlet step
            this.x += this.vx + ax;
            this.y += this.vy + ay;

            // Clear accumulated forces
            this.forceX = 0;
            this.forceY = 0;
        }

        /**
         * Keeps the point within world bounds.
         */
        constrain(bounds: { left: number, top: number, right: number, bottom: number }, bounce: number) {
            if (this.pinned) return;

            let velX = this.x - this.oldx;
            let velY = this.y - this.oldy;

            // Right Wall
            if (this.x > bounds.right - this.radius) {
                this.x = bounds.right - this.radius;
                this.oldx = this.x + (velX * bounce);
            }
            // Left Wall
            else if (this.x < bounds.left + this.radius) {
                this.x = bounds.left + this.radius;
                this.oldx = this.x + (velX * bounce);
            }

            // Floor
            if (this.y > bounds.bottom - this.radius) {
                this.y = bounds.bottom - this.radius;
                this.oldy = this.y + (velY * bounce);
            }
            // Ceiling
            else if (this.y < bounds.top + this.radius) {
                this.y = bounds.top + this.radius;
                this.oldy = this.y + (velY * bounce);
            }
        }
    }

    /**
     * A constraint connecting two Points.
     * Can represent a rigid rod, a spring, or a rope.
     */
    export class Stick {
        p1: Point;
        p2: Point;
        length: number;
        stiffness: number;
        visible: boolean;
        color: number;

        // Advanced properties
        breakingForce: number; // If tension exceeds this, stick breaks
        isDead: boolean;      // Helper for cleanup

        constructor(p1: Point, p2: Point, length: number, stiffness: number) {
            this.p1 = p1;
            this.p2 = p2;
            this.length = length;
            this.stiffness = stiffness;
            this.visible = true;
            this.color = 1;
            this.breakingForce = BREAK_THRESHOLD_OFF;
            this.isDead = false;
        }

        /**
         * Solves the constraint.
         * Moves p1 and p2 to satisfy the target length.
         */
        update() {
            if (this.isDead) return;

            const dx = this.p2.x - this.p1.x;
            const dy = this.p2.y - this.p1.y;
            const distSq = dx * dx + dy * dy;

            // Avoid sqrt if possible, but needed for linear constraint
            const dist = Math.sqrt(distSq);

            if (dist === 0) return; // Prevent divide by zero

            const diff = this.length - dist;

            // Check for breaking
            // Tension roughly equals the displacement
            if (this.breakingForce !== BREAK_THRESHOLD_OFF) {
                if (Math.abs(diff) > this.breakingForce) {
                    this.isDead = true;
                    return;
                }
            }

            // Calculate correction percentage
            // Stiffness limits how much we correct per frame (simulates elasticity)
            const percent = (diff / dist) / 2 * this.stiffness;

            const offsetX = dx * percent;
            const offsetY = dy * percent;

            // Apply correction inversely proportional to mass would be accurate,
            // but for simple jelly physics, equal split (or mass-unaware) is often stable enough.
            // Let's add basic mass weighting for better stability with heavy objects.
            const totalMass = this.p1.mass + this.p2.mass;
            const m1 = this.p2.mass / totalMass; // Swapped because heavier object moves less
            const m2 = this.p1.mass / totalMass;

            if (!this.p1.pinned) {
                this.p1.x -= offsetX * (this.stiffness < 1 ? 1 : 2 * m1);
                this.p1.y -= offsetY * (this.stiffness < 1 ? 1 : 2 * m1);
            }
            if (!this.p2.pinned) {
                this.p2.x += offsetX * (this.stiffness < 1 ? 1 : 2 * m2);
                this.p2.y += offsetY * (this.stiffness < 1 ? 1 : 2 * m2);
            }
        }

        draw() {
            if (this.visible && !this.isDead) {
                screen.drawLine(this.p1.x, this.p1.y, this.p2.x, this.p2.y, this.color);
            }
        }
    }

    /**
     * Physics Body container.
     * Holds a collection of Points and Sticks.
     */
    export class Body {
        points: Point[];
        sticks: Stick[];
        attachedSprite: Sprite;
        color: number;
        drawNodes: boolean;
        drawSticks: boolean;

        // Physics Properties
        isControlled: boolean;
        controlSpeed: number;
        useAerodynamics: boolean;
        dragCoefficient: number;

        // Optimization
        aabb: { minX: number, minY: number, maxX: number, maxY: number };

        constructor() {
            this.points = [];
            this.sticks = [];
            this.color = 1;
            this.drawNodes = true;
            this.drawSticks = true;
            this.isControlled = false;
            this.controlSpeed = 2;
            this.useAerodynamics = true;
            this.dragCoefficient = 0.05;
            this.aabb = { minX: 0, minY: 0, maxX: 0, maxY: 0 };
        }

        /**
         * Adds a point to the body.
         */
        addPoint(x: number, y: number): Point {
            let p = new Point(x, y);
            this.points.push(p);
            return p;
        }

        /**
         * Adds a stick constraint between two points by index.
         */
        addStick(i1: number, i2: number, stiffness: number, length: number): Stick {
            let p1 = this.points[i1];
            let p2 = this.points[i2];
            if (!p1 || !p2) return null;

            if (length === -1) {
                length = Vec2.dist(p1, p2);
            }

            let s = new Stick(p1, p2, length, stiffness);
            s.color = this.color;
            this.sticks.push(s);
            return s;
        }

        /**
         * Internal: Updates the Axis-Aligned Bounding Box.
         * Used for broad-phase collision detection.
         */
        updateAABB() {
            if (this.points.length === 0) return;
            let minX = this.points[0].x - this.points[0].radius;
            let maxX = this.points[0].x + this.points[0].radius;
            let minY = this.points[0].y - this.points[0].radius;
            let maxY = this.points[0].y + this.points[0].radius;

            for (let i = 1; i < this.points.length; i++) {
                let p = this.points[i];
                if (p.x - p.radius < minX) minX = p.x - p.radius;
                if (p.x + p.radius > maxX) maxX = p.x + p.radius;
                if (p.y - p.radius < minY) minY = p.y - p.radius;
                if (p.y + p.radius > maxY) maxY = p.y + p.radius;
            }
            this.aabb.minX = minX;
            this.aabb.maxX = maxX;
            this.aabb.minY = minY;
            this.aabb.maxY = maxY;
        }

        /**
         * Connects every point to every other point.
         * Creates a solid, rigid structure.
         */
        makeRigid() {
            for (let i = 0; i < this.points.length; i++) {
                for (let j = i + 1; j < this.points.length; j++) {
                    let exists = false;
                    for (let s of this.sticks) {
                        if ((s.p1 == this.points[i] && s.p2 == this.points[j]) ||
                            (s.p1 == this.points[j] && s.p2 == this.points[i])) {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists) {
                        let s = this.addStick(i, j, 1.0, -1);
                        s.visible = false; // Hidden internal supports
                    }
                }
            }
        }

        calculateCenter(): { x: number, y: number } {
            let cx = 0, cy = 0;
            if (this.points.length === 0) return { x: 0, y: 0 };
            for (let p of this.points) { cx += p.x; cy += p.y; }
            return { x: cx / this.points.length, y: cy / this.points.length };
        }

        /**
         * Main Update Loop for Body
         */
        update(gx: number, gy: number, friction: number, bounds: any, iterations: number, bounce: number, windX: number, windY: number) {

            // 1. Controls
            if (this.isControlled) {
                let dx = controller.dx(this.controlSpeed);
                let dy = controller.dy(this.controlSpeed);
                if (dx != 0 || dy != 0) {
                    for (let p of this.points) {
                        p.forceX += dx;
                        p.forceY += dy;
                    }
                }
            }

            // 2. Aerodynamics
            if (this.useAerodynamics) {
                for (let p of this.points) {
                    let rvx = p.vx - windX;
                    let rvy = p.vy - windY;
                    p.forceX -= rvx * this.dragCoefficient;
                    p.forceY -= rvy * this.dragCoefficient;
                }
            }

            // 3. Update Points
            for (let p of this.points) {
                p.update(gx, gy, friction);
                p.constrain(bounds, bounce);
            }

            // 4. Update Sticks (Constraint Solver)
            // We cleanup dead sticks first to save performance
            this.sticks = this.sticks.filter(s => !s.isDead);

            for (let i = 0; i < iterations; i++) {
                for (let s of this.sticks) {
                    s.update();
                }
            }

            // 5. Update AABB
            this.updateAABB();

            // 6. Sync Sprite
            if (this.attachedSprite && this.points.length > 0) {
                let c = this.calculateCenter();
                this.attachedSprite.setPosition(c.x, c.y);
            }
        }

        draw() {
            if (this.drawSticks) {
                for (let s of this.sticks) {
                    s.color = this.color; // Ensure stick color matches body
                    s.draw();
                }
            }
            if (this.drawNodes) {
                for (let p of this.points) {
                    // Simple circle drawing for nodes
                    if (p.radius <= 2) {
                        screen.setPixel(p.x, p.y, this.color);
                    } else {
                        // Draw a small cross or box for larger nodes
                        screen.drawLine(p.x - 1, p.y, p.x + 1, p.y, this.color);
                        screen.drawLine(p.x, p.y - 1, p.x, p.y + 1, this.color);
                    }
                }
            }
        }
    }

    // ==================================================================================
    // GLOBAL STATE
    // ==================================================================================

    let bodies: Body[] = [];

    // Physics World Config
    let worldGravityX = 0;
    let worldGravityY = 0.5;
    let worldFriction = 0.99;
    let worldBounce = 0.8;
    let worldIterations = 5;

    // Wind
    let worldWindX = 0;
    let worldWindY = 0;

    // Bounds
    let worldBounds = {
        left: 0,
        top: 0,
        right: screen.width,
        bottom: screen.height
    };

    let debugDraw = true;

    // Interaction
    let grabberSprite: Sprite = null;
    let grabbedPoint: Point = null;
    let grabRange = 25;

    // ==================================================================================
    // COLLISION RESOLUTION
    // ==================================================================================

    function checkAABBOverlap(b1: Body, b2: Body): boolean {
        return (b1.aabb.minX <= b2.aabb.maxX &&
            b1.aabb.maxX >= b2.aabb.minX &&
            b1.aabb.minY <= b2.aabb.maxY &&
            b1.aabb.maxY >= b2.aabb.minY);
    }

    function resolveCollisions() {
        // Broad Phase: AABB Check
        // Narrow Phase: Circle-Circle Check

        for (let i = 0; i < bodies.length; i++) {
            for (let j = i + 1; j < bodies.length; j++) {
                let b1 = bodies[i];
                let b2 = bodies[j];

                // Optimization: Skip if bounding boxes don't overlap
                if (!checkAABBOverlap(b1, b2)) continue;

                // N^2 Check - costly but necessary for detailed soft body collision
                // Could be optimized with spatial hashing for >100 nodes
                for (let p1 of b1.points) {
                    for (let p2 of b2.points) {
                        let dx = p1.x - p2.x;
                        let dy = p1.y - p2.y;

                        // Avoid expensive sqrt until strictly necessary
                        let distSq = dx * dx + dy * dy;
                        let minDist = p1.radius + p2.radius;
                        let minDistSq = minDist * minDist;

                        if (distSq < minDistSq && distSq > 0) {
                            let dist = Math.sqrt(distSq);
                            let overlap = minDist - dist;

                            // Mass weighting for response
                            let totalMass = p1.mass + p2.mass;
                            let r1 = p2.mass / totalMass;
                            let r2 = p1.mass / totalMass;

                            // Normal vector
                            let nx = dx / dist;
                            let ny = dy / dist;

                            // Separate points
                            let fx = nx * overlap;
                            let fy = ny * overlap;

                            // Apply separation
                            if (!p1.pinned) {
                                p1.x += fx * r1;
                                p1.y += fy * r1;

                                // Simple friction impulse
                                // p1.oldx += fx * 0.1;
                                // p1.oldy += fy * 0.1;
                            }
                            if (!p2.pinned) {
                                p2.x -= fx * r2;
                                p2.y -= fy * r2;
                            }
                        }
                    }
                }
            }
        }
    }

    // ==================================================================================
    // LIFECYCLE
    // ==================================================================================

    game.onUpdate(function () {
        // Update all bodies
        for (let b of bodies) {
            b.update(
                worldGravityX,
                worldGravityY,
                worldFriction,
                worldBounds,
                worldIterations,
                worldBounce,
                worldWindX,
                worldWindY
            );
        }

        // Solve collisions
        resolveCollisions();

        // Handle Interaction (Grabbing)
        if (grabberSprite) {
            if (controller.A.isPressed()) {
                if (!grabbedPoint) {
                    let closestDist = grabRange;
                    for (let b of bodies) {
                        for (let p of b.points) {
                            let dist = Vec2.dist(grabberSprite, p);
                            if (dist < closestDist) {
                                closestDist = dist;
                                grabbedPoint = p;
                            }
                        }
                    }
                }

                if (grabbedPoint) {
                    // Teleport point to grabber, reset forces
                    grabbedPoint.x = grabberSprite.x;
                    grabbedPoint.y = grabberSprite.y;
                    grabbedPoint.vx = 0;
                    grabbedPoint.vy = 0;
                    grabbedPoint.oldx = grabberSprite.x;
                    grabbedPoint.oldy = grabberSprite.y;
                }
            } else {
                grabbedPoint = null;
            }
        }
    });

    game.onPaint(function () {
        if (debugDraw) {
            // Draw Bounds
            if (worldBounds.left != 0 || worldBounds.top != 0 ||
                worldBounds.right != screen.width || worldBounds.bottom != screen.height) {
                screen.drawRect(
                    worldBounds.left,
                    worldBounds.top,
                    worldBounds.right - worldBounds.left,
                    worldBounds.bottom - worldBounds.top,
                    12
                );
            }

            for (let b of bodies) {
                b.draw();
            }

            // Draw grab line
            if (grabbedPoint && grabberSprite) {
                screen.drawLine(grabberSprite.x, grabberSprite.y, grabbedPoint.x, grabbedPoint.y, 5);
            }
        }
    });

    // ==================================================================================
    // BLOCK DEFINITIONS
    // ==================================================================================

    // ----------------------------------------------------------------------------------
    // GROUP: WORLD & CONFIG
    // ----------------------------------------------------------------------------------

    /**
     * Sets global physics parameters
     */
    //% group="World"
    //% block="set world gravity x %gx y %gy friction %f bounce %b"
    //% gx.defl=0 gy.defl=0.5 f.defl=0.99 b.defl=0.8
    export function setGlobalPhysics(gx: number, gy: number, f: number, b: number) {
        worldGravityX = gx;
        worldGravityY = gy;
        worldFriction = f;
        worldBounce = b;
    }

    /**
     * Sets global wind
     */
    //% group="World"
    //% block="set world wind x %wx y %wy"
    //% wx.defl=0 wy.defl=0
    export function setGlobalWind(wx: number, wy: number) {
        worldWindX = wx;
        worldWindY = wy;
    }

    /**
     * Sets world boundaries
     */
    //% group="World"
    //% block="set world bounds left %l top %t right %r bottom %b"
    //% l.defl=0 t.defl=0 r.defl=160 b.defl=120
    export function setWorldBounds(l: number, t: number, r: number, b: number) {
        worldBounds.left = l;
        worldBounds.top = t;
        worldBounds.right = r;
        worldBounds.bottom = b;
    }

    // ----------------------------------------------------------------------------------
    // GROUP: CONSTRUCTION (SHAPES)
    // ----------------------------------------------------------------------------------

    /**
     * Creates a new Soft Body Box
     */
    //% group="Construction"
    //% block="create soft box at x %x y %y size %size stiffness %stiffness"
    //% stiffness.defl=0.5
    //% size.defl=40
    export function createSoftBox(x: number, y: number, size: number, stiffness: number): Body {
        let b = new Body();
        b.addPoint(x, y);
        b.addPoint(x + size, y);
        b.addPoint(x + size, y + size);
        b.addPoint(x, y + size);

        // Frame
        b.addStick(0, 1, stiffness, -1);
        b.addStick(1, 2, stiffness, -1);
        b.addStick(2, 3, stiffness, -1);
        b.addStick(3, 0, stiffness, -1);

        // Bracing
        b.addStick(0, 2, stiffness, -1);
        b.addStick(1, 3, stiffness, -1);

        bodies.push(b);
        return b;
    }

    /**
     * Creates a Polygon (Triangle, Pentagon, Hexagon, etc.)
     */
    //% group="Construction"
    //% block="create polygon at x %x y %y sides %sides radius %radius stiffness %stiffness"
    //% sides.defl=5 radius.defl=30 stiffness.defl=0.8
    export function createPolygon(x: number, y: number, sides: number, radius: number, stiffness: number): Body {
        let b = new Body();
        if (sides < 3) sides = 3;

        // Create outer points
        for (let i = 0; i < sides; i++) {
            let angle = (i / sides) * 2 * Math.PI;
            // -Math.PI/2 to start at top
            let px = x + Math.cos(angle - Math.PI / 2) * radius;
            let py = y + Math.sin(angle - Math.PI / 2) * radius;
            b.addPoint(px, py);
        }

        // Center point for stability (spoked wheel structure)
        b.addPoint(x, y);
        let centerIdx = sides;

        // Connect edges
        for (let i = 0; i < sides; i++) {
            let next = (i + 1) % sides;
            b.addStick(i, next, stiffness, -1); // Perimeter
            b.addStick(i, centerIdx, stiffness, -1); // Spoke
        }

        bodies.push(b);
        return b;
    }

    /**
     * Creates a Rope
     */
    //% group="Construction"
    //% block="create rope at x %x y %y length %length segments %segments"
    //% length.defl=50 segments.defl=5
    export function createRope(x: number, y: number, length: number, segments: number): Body {
        let b = new Body();
        let segmentLen = length / segments;

        for (let i = 0; i <= segments; i++) {
            let p = b.addPoint(x, y + (i * segmentLen));
            if (i == 0) p.pinned = true;
            if (i > 0) {
                b.addStick(i - 1, i, 1.0, -1);
            }
        }
        bodies.push(b);
        return b;
    }

    /**
     * Creates a Cloth Grid (Curtain)
     */
    //% group="Construction"
    //% block="create cloth at x %x y %y width %w height %h spacing %spacing"
    //% w.defl=3 h.defl=3 spacing.defl=15
    export function createCloth(x: number, y: number, w: number, h: number, spacing: number): Body {
        let b = new Body();

        // Create grid points
        for (let j = 0; j < h; j++) {
            for (let i = 0; i < w; i++) {
                let p = b.addPoint(x + i * spacing, y + j * spacing);
                // Pin the top row
                if (j == 0) p.pinned = true;
            }
        }

        // Connect horizontal and vertical
        for (let j = 0; j < h; j++) {
            for (let i = 0; i < w; i++) {
                let idx = j * w + i;

                // Connect Right
                if (i < w - 1) {
                    b.addStick(idx, idx + 1, 0.8, -1);
                }
                // Connect Down
                if (j < h - 1) {
                    b.addStick(idx, idx + w, 0.8, -1);
                }
            }
        }

        bodies.push(b);
        return b;
    }

    /**
     * Creates a Bridge between two points
     */
    //% group="Construction"
    //% block="create bridge from x1 %x1 y1 %y1 to x2 %x2 y2 %y2 segments %segments"
    //% segments.defl=8
    export function createBridge(x1: number, y1: number, x2: number, y2: number, segments: number): Body {
        let b = new Body();

        let dx = x2 - x1;
        let dy = y2 - y1;

        for (let i = 0; i <= segments; i++) {
            let t = i / segments;
            let px = x1 + dx * t;
            let py = y1 + dy * t;
            let p = b.addPoint(px, py);

            // Pin start and end
            if (i === 0 || i === segments) p.pinned = true;

            if (i > 0) {
                // Top rail
                b.addStick(i - 1, i, 0.9, -1);
                // Bottom rail offset
                // b.addStick... (Optional: double bridge)
            }
        }

        bodies.push(b);
        return b;
    }

    /**
     * Creates a Ragdoll
     */
    //% group="Construction"
    //% block="create ragdoll at x %x y %y scale %scale"
    //% scale.defl=1.0
    export function createRagdoll(x: number, y: number, scale: number): Body {
        let b = new Body();
        let s = scale * 20;

        // 0-Head, 1-Chest, 2-Hips, 3-LElbow, 4-LHand, 5-RElbow, 6-RHand, 7-LKnee, 8-LFoot, 9-RKnee, 10-RFoot
        let head = b.addPoint(x, y - s * 1.5);
        head.radius = 10 * scale; // Bigger head hitbox

        b.addPoint(x, y);
        b.addPoint(x, y + s);
        b.addPoint(x - s, y);
        b.addPoint(x - s * 1.8, y);
        b.addPoint(x + s, y);
        b.addPoint(x + s * 1.8, y);
        b.addPoint(x - s * 0.5, y + s * 1.8);
        b.addPoint(x - s * 0.5, y + s * 2.6);
        b.addPoint(x + s * 0.5, y + s * 1.8);
        b.addPoint(x + s * 0.5, y + s * 2.6);

        let stiff = 1.0;
        b.addStick(0, 1, stiff, -1);
        b.addStick(1, 2, stiff, -1);
        b.addStick(1, 3, stiff, -1);
        b.addStick(3, 4, stiff, -1);
        b.addStick(1, 5, stiff, -1);
        b.addStick(5, 6, stiff, -1);
        b.addStick(2, 7, stiff, -1);
        b.addStick(7, 8, stiff, -1);
        b.addStick(2, 9, stiff, -1);
        b.addStick(9, 10, stiff, -1);

        b.addStick(0, 2, 0.5, -1).visible = false;
        b.addStick(3, 5, 0.5, -1).visible = false;

        bodies.push(b);
        return b;
    }

    /**
     * Create an empty physics body
     */
    //% group="Construction"
    //% block="create empty body"
    export function createEmptyBody(): Body {
        let b = new Body();
        bodies.push(b);
        return b;
    }

    /**
     * Destroys a body (removes it from the game)
     */
    //% group="Bodies"
    //% block="destroy body %body"
    export function destroyBody(body: Body) {
        if (!body) return;
        bodies.removeElement(body);
        // Clean refs
        body.points = [];
        body.sticks = [];
    }

    // ----------------------------------------------------------------------------------
    // GROUP: MATERIALS
    // ----------------------------------------------------------------------------------

    export enum MaterialPreset {
        //% block="Jelly"
        Jelly,
        //% block="Steel"
        Steel,
        //% block="Cloth"
        Cloth,
        //% block="Rubber"
        Rubber,
        //% block="Wood"
        Wood
    }

    /**
     * Applies a material preset to a body
     */
    //% group="Materials"
    //% block="apply material %preset to %body"
    export function applyMaterial(preset: MaterialPreset, body: Body) {
        if (!body) return;

        let stiffness = 1.0;
        let mass = 1.0;
        let drag = 0.05;
        let bounce = 0.8;

        switch (preset) {
            case MaterialPreset.Jelly:
                stiffness = 0.3;
                mass = 1.0;
                bounce = 0.9;
                break;
            case MaterialPreset.Steel:
                stiffness = 1.0;
                mass = 5.0;
                bounce = 0.2;
                body.makeRigid(); // Steel doesn't wobble
                break;
            case MaterialPreset.Cloth:
                stiffness = 0.8;
                mass = 0.5;
                drag = 0.2; // Catch air
                bounce = 0.1;
                break;
            case MaterialPreset.Rubber:
                stiffness = 0.7;
                mass = 1.2;
                bounce = 0.95;
                break;
            case MaterialPreset.Wood:
                stiffness = 0.9;
                mass = 2.0;
                bounce = 0.3;
                break;
        }

        // Apply
        for (let s of body.sticks) s.stiffness = stiffness;
        for (let p of body.points) p.mass = mass / body.points.length;
        body.dragCoefficient = drag;
        // Note: Bounce is currently global in Point.constrain, 
        // implementing per-body bounce requires changing Point class constraint logic.
        // For now, we mainly affect stiffness and mass.
    }

    // ----------------------------------------------------------------------------------
    // GROUP: BODIES & CONFIG
    // ----------------------------------------------------------------------------------

    /**
     * Set mass for a specific body
     */
    //% group="Bodies"
    //% block="set body %body mass %mass"
    //% mass.defl=1.0
    export function setBodyMass(body: Body, mass: number) {
        if (!body) return;
        let pmass = mass / body.points.length;
        for (let p of body.points) {
            p.mass = pmass;
        }
    }

    /**
     * Scales gravity for a specific body
     */
    //% group="Bodies"
    //% block="set body %body gravity scale %scale"
    //% scale.defl=1.0
    export function setBodyGravityScale(body: Body, scale: number) {
        if (!body) return;
        for (let p of body.points) {
            p.gravityScale = scale;
        }
    }

    /**
     * Enable aerodynamics for a body
     */
    //% group="Bodies"
    //% block="set body %body aerodynamics %on drag %drag"
    //% on.shadow=toggleOnOff on.defl=true
    //% drag.defl=0.05
    export function setBodyAerodynamics(body: Body, on: boolean, drag: number) {
        if (!body) return;
        body.useAerodynamics = on;
        body.dragCoefficient = drag;
    }

    /**
     * Make a body rigid by cross-bracing it
     */
    //% group="Bodies"
    //% block="make body %body rigid"
    export function makeBodyRigid(body: Body) {
        if (!body) return;
        body.makeRigid();
    }

    /**
     * Teleports a body to a new location
     */
    //% group="Bodies"
    //% block="teleport body %body to x %x y %y"
    export function teleportBody(body: Body, x: number, y: number) {
        if (!body || body.points.length === 0) return;
        let c = body.calculateCenter();
        let dx = x - c.x;
        let dy = y - c.y;

        for (let p of body.points) {
            p.x += dx;
            p.y += dy;
            p.oldx += dx;
            p.oldy += dy;
        }
    }

    /**
     * Get X position of body center
     */
    //% group="Bodies"
    //% block="get body %body x"
    export function getBodyX(body: Body): number {
        if (!body) return 0;
        return body.calculateCenter().x;
    }

    /**
     * Get Y position of body center
     */
    //% group="Bodies"
    //% block="get body %body y"
    export function getBodyY(body: Body): number {
        if (!body) return 0;
        return body.calculateCenter().y;
    }

    // ----------------------------------------------------------------------------------
    // GROUP: INTERACTION & CONTROLS
    // ----------------------------------------------------------------------------------

    /**
     * Controls a body with the player controller (D-Pad)
     */
    //% group="Interaction"
    //% block="move body %body with controller speed %speed"
    //% speed.defl=5
    export function moveBodyWithController(body: Body, speed: number) {
        if (!body) return;
        body.isControlled = true;
        body.controlSpeed = speed;
    }

    /**
     * Apply a force (push) to a body
     */
    //% group="Interaction"
    //% block="apply force to %body x %fx y %fy"
    export function applyForce(body: Body, fx: number, fy: number) {
        if (!body) return;
        for (let p of body.points) {
            p.forceX += fx;
            p.forceY += fy;
        }
    }

    /**
     * Explodes at a point, pushing bodies away and potentially breaking sticks
     */
    //% group="Interaction"
    //% block="explode at x %x y %y radius %radius force %force"
    //% radius.defl=50 force.defl=15
    export function explode(x: number, y: number, radius: number, force: number) {
        for (let b of bodies) {
            for (let p of b.points) {
                let dx = p.x - x;
                let dy = p.y - y;
                let dist = Math.sqrt(dx * dx + dy * dy);

                if (dist < radius && dist > 0) {
                    let f = (1 - (dist / radius)) * force;
                    // Apply blast impulse
                    p.forceX += (dx / dist) * f;
                    p.forceY += (dy / dist) * f;
                }
            }
        }
    }

    /**
     * Enable dragging with a sprite (A button)
     */
    //% group="Interaction"
    //% block="enable dragging with sprite %sprite"
    export function enableDragging(sprite: Sprite) {
        grabberSprite = sprite;
    }

    // ----------------------------------------------------------------------------------
    // GROUP: JOINTS & ADVANCED CONSTRUCTION
    // ----------------------------------------------------------------------------------

    /**
     * Creates a custom hitbox by setting a specific node's radius.
     * Use index 0 for single-point bodies.
     */
    //% group="Joints"
    //% block="set node radius for %body at index %index to %radius"
    //% radius.defl=10
    export function setNodeRadius(body: Body, index: number, radius: number) {
        if (body && body.points[index]) {
            body.points[index].radius = radius;
        }
    }

    /**
     * Pins or unpins a specific node
     */
    //% group="Joints"
    //% block="set node pinned for %body at index %index to %pinned"
    //% pinned.shadow=toggleOnOff
    export function setNodePinned(body: Body, index: number, pinned: boolean) {
        if (body && body.points[index]) {
            body.points[index].pinned = pinned;
        }
    }

    /**
     * Add a node to a body manually
     */
    //% group="Joints"
    //% block="add node to %body at x %x y %y"
    export function addNode(body: Body, x: number, y: number) {
        if (body) body.addPoint(x, y);
    }

    /**
     * Connect two nodes manually
     */
    //% group="Joints"
    //% block="connect nodes in %body from %i1 to %i2 stiffness %stiffness"
    //% stiffness.defl=1.0
    export function connectNodes(body: Body, i1: number, i2: number, stiffness: number) {
        if (body) body.addStick(i1, i2, stiffness, -1);
    }

    /**
     * Sets the breaking force for a connection.
     * Use -1 for unbreakable.
     */
    //% group="Joints"
    //% block="set connection break force in %body from %i1 to %i2 force %force"
    //% force.defl=50
    export function setConnectionBreakForce(body: Body, i1: number, i2: number, force: number) {
        if (!body) return;
        // Find the stick connecting these two points
        let p1 = body.points[i1];
        let p2 = body.points[i2];
        if (!p1 || !p2) return;

        for (let s of body.sticks) {
            if ((s.p1 == p1 && s.p2 == p2) || (s.p1 == p2 && s.p2 == p1)) {
                s.breakingForce = force;
                return;
            }
        }
    }

    // ----------------------------------------------------------------------------------
    // GROUP: ADVANCED TOOLS (RAYCAST)
    // ----------------------------------------------------------------------------------

    /**
     * Raycast against all bodies. Returns true if hit.
     */
    //% group="Advanced"
    //% block="raycast hit from x %x1 y %y1 to x %x2 y %y2"
    export function raycastHit(x1: number, y1: number, x2: number, y2: number): boolean {
        // Line-Circle intersection check for all nodes
        // This is a naive implementation but works for simple checks

        for (let b of bodies) {
            // AABB Check first for optimization
            if (Math.min(x1, x2) > b.aabb.maxX || Math.max(x1, x2) < b.aabb.minX ||
                Math.min(y1, y2) > b.aabb.maxY || Math.max(y1, y2) < b.aabb.minY) {
                continue;
            }

            for (let p of b.points) {
                // Check distance from point to line segment
                // A = (x1, y1), B = (x2, y2), P = (p.x, p.y)
                // Project P onto AB, clamp to segment, check dist

                let l2 = Vec2.distSq({ x: x1, y: y1 }, { x: x2, y: y2 });
                if (l2 == 0) continue; // Ray is a point

                let t = ((p.x - x1) * (x2 - x1) + (p.y - y1) * (y2 - y1)) / l2;
                t = Math.max(0, Math.min(1, t));

                let projX = x1 + t * (x2 - x1);
                let projY = y1 + t * (y2 - y1);

                let dSq = Vec2.distSq({ x: p.x, y: p.y }, { x: projX, y: projY });

                if (dSq < p.radius * p.radius) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Attaches a Sprite to a Physics Body
     */
    //% group="Interaction"
    //% block="attach sprite %sprite to body %body"
    export function attachSprite(sprite: Sprite, body: Body) {
        if (!body) return;
        body.attachedSprite = sprite;
    }

    /**
     * Toggle Debug Drawing
     */
    //% group="Interaction"
    //% block="set debug draw %on"
    //% on.shadow=toggleOnOff
    //% on.defl=true
    export function setDebugDraw(on: boolean) {
        debugDraw = on;
    }

    /**
     * Set Body Color
     */
    //% group="Bodies"
    //% block="set body %body color %color"
    //% color.shadow="colorindexpicker"
    export function setBodyColor(body: Body, color: number) {
        if (body) body.color = color;
    }
}