//% color="#E63022" weight=100 icon="\uf1e3" block="Jelly Physics V3"
//% groups='["World", "Bodies", "Construction", "Materials", "Joints", "Interaction", "Visuals", "Events", "Advanced"]'
namespace jelly {

    // ==================================================================================
    // CONSTANTS & CONFIG
    // ==================================================================================

    const DEFAULT_RADIUS = 5;
    const DEFAULT_MASS = 1.0;
    const DEFAULT_STIFFNESS = 1.0;
    const DEFAULT_FRICTION = 0.99;
    const DEFAULT_BOUNCE = 0.8;
    const MAX_ITERATIONS = 16;
    const BREAK_THRESHOLD_OFF = -1;

    // Internal Event IDs
    const EVT_COLLISION = 6600;
    const EVT_BREAK = 6601;

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

        static angle(v1: { x: number, y: number }, v2: { x: number, y: number }): number {
            return Math.atan2(v2.y - v1.y, v2.x - v1.x);
        }

        static dot(v1: Vec2, v2: Vec2): number {
            return v1.x * v2.x + v1.y * v2.y;
        }

        static cross(v1: Vec2, v2: Vec2): number {
            return v1.x * v2.y - v1.y * v2.x;
        }

        static normalize(v: Vec2): Vec2 {
            let len = Math.sqrt(v.x * v.x + v.y * v.y);
            if (len === 0) return new Vec2(0, 0);
            return new Vec2(v.x / len, v.y / len);
        }

        static sub(v1: Vec2, v2: Vec2): Vec2 {
            return new Vec2(v1.x - v2.x, v1.y - v2.y);
        }

        static add(v1: Vec2, v2: Vec2): Vec2 {
            return new Vec2(v1.x + v2.x, v1.y + v2.y);
        }

        static scale(v: Vec2, s: number): Vec2 {
            return new Vec2(v.x * s, v.y * s);
        }
    }

    // ==================================================================================
    // CORE CLASSES
    // ==================================================================================

    /**
     * A single node in the physics simulation.
     */
    export class Point {
        x: number;
        y: number;
        oldx: number;
        oldy: number;
        vx: number;
        vy: number;
        pinned: boolean;
        visible: boolean; // Toggleable visibility
        radius: number;
        mass: number;
        gravityScale: number;
        forceX: number;
        forceY: number;
        id: number;
        tag: number;

        constructor(x: number, y: number) {
            this.x = x;
            this.y = y;
            this.oldx = x;
            this.oldy = y;
            this.vx = 0;
            this.vy = 0;
            this.pinned = false;
            this.visible = true;
            this.radius = DEFAULT_RADIUS;
            this.mass = DEFAULT_MASS;
            this.gravityScale = 1.0;
            this.forceX = 0;
            this.forceY = 0;
            this.tag = 0;
            this.id = Math.random();
        }

        update(gravityX: number, gravityY: number, friction: number, waterLevel: number) {
            if (this.pinned) return;

            // Verlet Integration
            this.vx = (this.x - this.oldx) * friction;
            this.vy = (this.y - this.oldy) * friction;

            // Water Buoyancy
            if (waterLevel !== 0 && this.y > waterLevel) {
                // Apply upward force (Anti-gravity + lift)
                this.forceY -= (gravityY * this.mass * 1.5);

                // Increased Drag in water
                this.vx *= 0.90;
                this.vy *= 0.90;
            }

            // F = ma -> a = F/m
            let ax = this.forceX / this.mass;
            let ay = this.forceY / this.mass;

            // Apply Gravity
            ax += gravityX * this.gravityScale;
            ay += gravityY * this.gravityScale;

            this.oldx = this.x;
            this.oldy = this.y;

            this.x += this.vx + ax;
            this.y += this.vy + ay;

            // Reset forces
            this.forceX = 0;
            this.forceY = 0;
        }

        constrain(bounds: { left: number, top: number, right: number, bottom: number }, bounce: number) {
            if (this.pinned) return;

            let velX = this.x - this.oldx;
            let velY = this.y - this.oldy;

            if (this.x > bounds.right - this.radius) {
                this.x = bounds.right - this.radius;
                this.oldx = this.x + (velX * bounce);
            } else if (this.x < bounds.left + this.radius) {
                this.x = bounds.left + this.radius;
                this.oldx = this.x + (velX * bounce);
            }

            if (this.y > bounds.bottom - this.radius) {
                this.y = bounds.bottom - this.radius;
                this.oldy = this.y + (velY * bounce);
            } else if (this.y < bounds.top + this.radius) {
                this.y = bounds.top + this.radius;
                this.oldy = this.y + (velY * bounce);
            }
        }
    }

    export class Stick {
        p1: Point;
        p2: Point;
        length: number;
        stiffness: number;
        visible: boolean;
        color: number;
        breakingForce: number;
        isDead: boolean;

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

        update() {
            if (this.isDead) return;

            const dx = this.p2.x - this.p1.x;
            const dy = this.p2.y - this.p1.y;
            const distSq = dx * dx + dy * dy;
            const dist = Math.sqrt(distSq);

            if (dist === 0) return;

            const diff = this.length - dist;

            // Check for breaking
            if (this.breakingForce !== BREAK_THRESHOLD_OFF) {
                if (Math.abs(diff) > this.breakingForce) {
                    this.isDead = true;
                    // Fire break event logic here if needed
                    return;
                }
            }

            const percent = (diff / dist) / 2 * this.stiffness;
            const offsetX = dx * percent;
            const offsetY = dy * percent;

            const totalMass = this.p1.mass + this.p2.mass;
            const m1 = this.p2.mass / totalMass;
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
     */
    export class Body {
        id: number;
        points: Point[];
        sticks: Stick[];

        // Sprite integration
        attachedSprite: Sprite;
        spriteScaleBaseX: number;
        spriteScaleBaseY: number;
        visuals: {
            rotate: boolean,
            squash: boolean,
            offsetX: number,
            offsetY: number
        };

        color: number;
        fillColor: number; // For solid rendering
        drawNodes: boolean;
        drawSticks: boolean;
        drawFill: boolean; // New solid fill mode

        // Physics Properties
        isControlled: boolean;
        controlSpeed: number;
        useAerodynamics: boolean;
        dragCoefficient: number;
        motorSpeed: number;
        pressure: number; // Gas Law Pressure

        // Optimization
        aabb: { minX: number, minY: number, maxX: number, maxY: number };

        // Callback Refs
        onCollisionHandler: () => void;
        onBreakHandler: () => void;

        constructor() {
            this.id = Math.random();
            this.points = [];
            this.sticks = [];
            this.color = 1;
            this.fillColor = 0;
            this.drawNodes = true;
            this.drawSticks = true;
            this.drawFill = false;
            this.isControlled = false;
            this.controlSpeed = 2;
            this.useAerodynamics = true;
            this.dragCoefficient = 0.05;
            this.motorSpeed = 0;
            this.pressure = 0;
            this.aabb = { minX: 0, minY: 0, maxX: 0, maxY: 0 };
            this.visuals = {
                rotate: false,
                squash: false,
                offsetX: 0,
                offsetY: 0
            };
            this.spriteScaleBaseX = 1;
            this.spriteScaleBaseY = 1;
        }

        addPoint(x: number, y: number): Point {
            let p = new Point(x, y);
            this.points.push(p);
            return p;
        }

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
                        s.visible = false;
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
         * Calculates Polygon Volume (Area) for pressure
         */
        calculateVolume(): number {
            let area = 0;
            let j = this.points.length - 1;
            for (let i = 0; i < this.points.length; i++) {
                area += (this.points[j].x + this.points[i].x) * (this.points[j].y - this.points[i].y);
                j = i;
            }
            return Math.abs(area / 2);
        }

        update(gx: number, gy: number, friction: number, bounds: any, iterations: number, bounce: number, windX: number, windY: number, waterLevel: number) {

            let center = this.calculateCenter();

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

            // 2. Motor (Spin)
            if (this.motorSpeed !== 0) {
                for (let p of this.points) {
                    let dx = p.x - center.x;
                    let dy = p.y - center.y;
                    p.forceX += -dy * this.motorSpeed * 0.1;
                    p.forceY += dx * this.motorSpeed * 0.1;
                }
            }

            // 3. Pressure (Inflation)
            if (this.pressure !== 0 && this.points.length > 2) {
                // For a closed loop polygon, apply pressure normal to edges
                for (let i = 0; i < this.points.length; i++) {
                    let p1 = this.points[i];
                    let p2 = this.points[(i + 1) % this.points.length];

                    let dx = p2.x - p1.x;
                    let dy = p2.y - p1.y;
                    let dist = Math.sqrt(dx * dx + dy * dy);

                    // Normal vector (perpendicular)
                    let nx = -dy / dist;
                    let ny = dx / dist;

                    // Pressure Force = Normal * Length * PressureAmount
                    let force = this.pressure * dist;

                    // Apply to both points
                    p1.forceX += nx * force * 0.5;
                    p1.forceY += ny * force * 0.5;
                    p2.forceX += nx * force * 0.5;
                    p2.forceY += ny * force * 0.5;
                }
            }

            // 4. Aerodynamics
            if (this.useAerodynamics) {
                for (let p of this.points) {
                    let rvx = p.vx - windX;
                    let rvy = p.vy - windY;
                    p.forceX -= rvx * this.dragCoefficient;
                    p.forceY -= rvy * this.dragCoefficient;
                }
            }

            // 5. Update Points
            for (let p of this.points) {
                p.update(gx, gy, friction, waterLevel);
                p.constrain(bounds, bounce);
            }

            // 6. Update Sticks
            let brokenSticks = false;
            this.sticks = this.sticks.filter(s => {
                if (s.isDead) {
                    brokenSticks = true;
                    // Spawn debris on break
                    // (Simple particle effect)
                    // We can't easily access main scope particle func here without exposing it.
                    // But we can trigger the handler.
                }
                return !s.isDead;
            });

            if (brokenSticks && this.onBreakHandler) {
                this.onBreakHandler();
            }

            for (let i = 0; i < iterations; i++) {
                for (let s of this.sticks) {
                    s.update();
                }
            }

            // 7. Update AABB
            this.updateAABB();

            // 8. Sync Sprite
            if (this.attachedSprite && this.points.length > 0) {
                let c = this.calculateCenter();
                this.attachedSprite.setPosition(c.x + this.visuals.offsetX, c.y + this.visuals.offsetY);

                if (this.visuals.rotate && this.points.length >= 2) {
                    // Placeholder for rotation logic if extensions allow
                }
            }
        }

        /**
         * Advanced Rendering: Polygon Scanline Fill
         * This is expensive in MakeCode Arcade TS, but requested for "skin".
         */
        drawFilled() {
            if (this.points.length < 3) return;

            // Simple approach: Triangle Fan from center
            let c = this.calculateCenter();
            for (let i = 0; i < this.points.length; i++) {
                let p1 = this.points[i];
                let p2 = this.points[(i + 1) % this.points.length];

                // We use a custom triangle filler helper
                fillTriangle(c.x, c.y, p1.x, p1.y, p2.x, p2.y, this.fillColor);
            }
        }

        draw() {
            if (this.drawFill) {
                this.drawFilled();
            }

            if (this.drawSticks) {
                for (let s of this.sticks) {
                    s.color = this.color;
                    s.draw();
                }
            }
            if (this.drawNodes) {
                for (let p of this.points) {
                    if (!p.visible) continue;

                    if (p.radius <= 2) {
                        screen.setPixel(p.x, p.y, this.color);
                    } else {
                        screen.drawLine(p.x - 1, p.y, p.x + 1, p.y, this.color);
                        screen.drawLine(p.x, p.y - 1, p.x, p.y + 1, this.color);
                    }
                }
            }
        }
    }

    // ==================================================================================
    // HELPER: RASTERIZATION
    // ==================================================================================

    // Scanline Triangle Filler
    function fillTriangle(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, c: number) {
        // Sort vertices by Y
        if (y1 > y2) { let tx = x1; let ty = y1; x1 = x2; y1 = y2; x2 = tx; y2 = ty; }
        if (y1 > y3) { let tx = x1; let ty = y1; x1 = x3; y1 = y3; x3 = tx; y3 = ty; }
        if (y2 > y3) { let tx = x2; let ty = y2; x2 = x3; y2 = y3; x3 = tx; y3 = ty; }

        let dx1 = (x2 - x1) / (y2 - y1);
        let dx2 = (x3 - x1) / (y3 - y1);
        let dx3 = (x3 - x2) / (y3 - y2);

        let sx = x1, ex = x1;

        // Top part
        for (let y = y1; y < y2; y++) {
            screen.drawLine(sx, y, ex, y, c);
            sx += dx1;
            ex += dx2;
        }

        // Bottom part
        sx = x2;
        // ex is already at the right place on the long edge
        for (let y = y2; y < y3; y++) {
            screen.drawLine(sx, y, ex, y, c);
            sx += dx3;
            ex += dx2;
        }
    }

    // ==================================================================================
    // GLOBAL STATE
    // ==================================================================================

    let bodies: Body[] = [];

    let worldGravityX = 0;
    let worldGravityY = 0.5;
    let worldFriction = 0.99;
    let worldBounce = 0.8;
    let worldIterations = 5;
    let worldWindX = 0;
    let worldWindY = 0;
    let worldWaterLevel = 0;

    let worldBounds = {
        left: 0,
        top: 0,
        right: screen.width,
        bottom: screen.height
    };

    let debugDraw = true;
    let drawVectors = false;
    let grabberSprite: Sprite = null;
    let grabbedPoint: Point = null;
    let grabRange = 25;

    // Event State
    let collisionHandlers: { bodyA: Body, bodyB: Body, handler: () => void }[] = [];

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
        for (let i = 0; i < bodies.length; i++) {
            for (let j = i + 1; j < bodies.length; j++) {
                let b1 = bodies[i];
                let b2 = bodies[j];

                if (!checkAABBOverlap(b1, b2)) continue;

                let hasCollided = false;

                for (let p1 of b1.points) {
                    for (let p2 of b2.points) {
                        let dx = p1.x - p2.x;
                        let dy = p1.y - p2.y;
                        let distSq = dx * dx + dy * dy;
                        let minDist = p1.radius + p2.radius;
                        let minDistSq = minDist * minDist;

                        if (distSq < minDistSq && distSq > 0) {
                            hasCollided = true;
                            let dist = Math.sqrt(distSq);
                            let overlap = minDist - dist;

                            let totalMass = p1.mass + p2.mass;
                            let r1 = p2.mass / totalMass;
                            let r2 = p1.mass / totalMass;

                            let nx = dx / dist;
                            let ny = dy / dist;

                            let fx = nx * overlap;
                            let fy = ny * overlap;

                            if (!p1.pinned) {
                                p1.x += fx * r1;
                                p1.y += fy * r1;
                            }
                            if (!p2.pinned) {
                                p2.x -= fx * r2;
                                p2.y -= fy * r2;
                            }
                        }
                    }
                }

                // Fire Collision Events if registered
                if (hasCollided) {
                    // Check general handlers
                    if (b1.onCollisionHandler) b1.onCollisionHandler();
                    if (b2.onCollisionHandler) b2.onCollisionHandler();

                    // Check specific pair handlers
                    for (let h of collisionHandlers) {
                        if ((h.bodyA == b1 && h.bodyB == b2) || (h.bodyA == b2 && h.bodyB == b1)) {
                            h.handler();
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
        for (let b of bodies) {
            b.update(
                worldGravityX,
                worldGravityY,
                worldFriction,
                worldBounds,
                worldIterations,
                worldBounce,
                worldWindX,
                worldWindY,
                worldWaterLevel
            );
        }

        resolveCollisions();

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

            // Draw Water Level
            if (worldWaterLevel > 0) {
                screen.fillRect(0, worldWaterLevel, screen.width, screen.height - worldWaterLevel, 9); // Blue tint
            }

            for (let b of bodies) {
                b.draw();

                // Vector Debug
                if (drawVectors) {
                    for (let p of b.points) {
                        screen.drawLine(p.x, p.y, p.x + p.vx * 5, p.y + p.vy * 5, 5);
                    }
                }
            }

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
     * Sets water level (Y coordinate). 0 to disable.
     */
    //% group="World"
    //% block="set water level at y %y"
    //% y.defl=100
    export function setWaterLevel(y: number) {
        worldWaterLevel = y;
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

        b.addStick(0, 1, stiffness, -1);
        b.addStick(1, 2, stiffness, -1);
        b.addStick(2, 3, stiffness, -1);
        b.addStick(3, 0, stiffness, -1);

        b.addStick(0, 2, stiffness, -1);
        b.addStick(1, 3, stiffness, -1);

        bodies.push(b);
        return b;
    }

    /**
     * Creates a physics body from a Sprite
     */
    //% group="Construction"
    //% block="create physics body from sprite %sprite stiffness %stiffness"
    //% stiffness.defl=0.5
    export function createBodyFromSprite(sprite: Sprite, stiffness: number): Body {
        if (!sprite) return null;

        let x = sprite.x - sprite.width / 2;
        let y = sprite.y - sprite.height / 2;

        let b = new Body();
        b.addPoint(x, y); // TL
        b.addPoint(x + sprite.width, y); // TR
        b.addPoint(x + sprite.width, y + sprite.height); // BR
        b.addPoint(x, y + sprite.height); // BL

        b.addStick(0, 1, stiffness, -1);
        b.addStick(1, 2, stiffness, -1);
        b.addStick(2, 3, stiffness, -1);
        b.addStick(3, 0, stiffness, -1);

        b.addStick(0, 2, stiffness, -1);
        b.addStick(1, 3, stiffness, -1);

        b.attachedSprite = sprite;
        b.spriteScaleBaseX = sprite.width;
        b.spriteScaleBaseY = sprite.height;
        b.drawNodes = false;
        b.drawSticks = false;

        bodies.push(b);
        return b;
    }

    /**
     * Creates a Polygon
     */
    //% group="Construction"
    //% block="create polygon at x %x y %y sides %sides radius %radius stiffness %stiffness"
    //% sides.defl=5 radius.defl=30 stiffness.defl=0.8
    export function createPolygon(x: number, y: number, sides: number, radius: number, stiffness: number): Body {
        let b = new Body();
        if (sides < 3) sides = 3;

        for (let i = 0; i < sides; i++) {
            let angle = (i / sides) * 2 * Math.PI;
            let px = x + Math.cos(angle - Math.PI / 2) * radius;
            let py = y + Math.sin(angle - Math.PI / 2) * radius;
            b.addPoint(px, py);
        }

        b.addPoint(x, y);
        let centerIdx = sides;

        for (let i = 0; i < sides; i++) {
            let next = (i + 1) % sides;
            b.addStick(i, next, stiffness, -1);
            b.addStick(i, centerIdx, stiffness, -1);
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
     * Creates a Cloth Grid
     */
    //% group="Construction"
    //% block="create cloth at x %x y %y width %w height %h spacing %spacing"
    //% w.defl=3 h.defl=3 spacing.defl=15
    export function createCloth(x: number, y: number, w: number, h: number, spacing: number): Body {
        let b = new Body();

        for (let j = 0; j < h; j++) {
            for (let i = 0; i < w; i++) {
                let p = b.addPoint(x + i * spacing, y + j * spacing);
                if (j == 0) p.pinned = true;
            }
        }

        for (let j = 0; j < h; j++) {
            for (let i = 0; i < w; i++) {
                let idx = j * w + i;
                if (i < w - 1) b.addStick(idx, idx + 1, 0.8, -1);
                if (j < h - 1) b.addStick(idx, idx + w, 0.8, -1);
            }
        }

        bodies.push(b);
        return b;
    }

    /**
     * Creates a Bridge
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
            if (i === 0 || i === segments) p.pinned = true;
            if (i > 0) b.addStick(i - 1, i, 0.9, -1);
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

        b.addPoint(x, y - s * 1.5);
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
     * Destroys a body
     */
    //% group="Bodies"
    //% block="destroy body %body"
    export function destroyBody(body: Body) {
        if (!body) return;
        bodies.removeElement(body);
        body.points = [];
        body.sticks = [];
        if (body.attachedSprite) body.attachedSprite.destroy();
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

        switch (preset) {
            case MaterialPreset.Jelly:
                stiffness = 0.3; mass = 1.0; break;
            case MaterialPreset.Steel:
                stiffness = 1.0; mass = 5.0; body.makeRigid(); break;
            case MaterialPreset.Cloth:
                stiffness = 0.8; mass = 0.5; drag = 0.2; break;
            case MaterialPreset.Rubber:
                stiffness = 0.7; mass = 1.2; break;
            case MaterialPreset.Wood:
                stiffness = 0.9; mass = 2.0; break;
        }

        for (let s of body.sticks) s.stiffness = stiffness;
        for (let p of body.points) p.mass = mass / body.points.length;
        body.dragCoefficient = drag;
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
     * Make a body rigid
     */
    //% group="Bodies"
    //% block="make body %body rigid"
    export function makeBodyRigid(body: Body) {
        if (!body) return;
        body.makeRigid();
    }

    /**
     * Make a body spin (Motor)
     */
    //% group="Bodies"
    //% block="spin body %body speed %speed"
    //% speed.defl=5
    export function spinBody(body: Body, speed: number) {
        if (!body) return;
        body.motorSpeed = speed;
    }

    /**
     * Scale a body (Grow/Shrink)
     */
    //% group="Bodies"
    //% block="scale body %body by %factor"
    //% factor.defl=1.1
    export function scaleBody(body: Body, factor: number) {
        if (!body || body.points.length === 0) return;
        let c = body.calculateCenter();

        for (let p of body.points) {
            let dx = p.x - c.x;
            let dy = p.y - c.y;
            p.x = c.x + dx * factor;
            p.y = c.y + dy * factor;
            p.oldx = p.x - (p.vx * factor);
            p.oldy = p.y - (p.vy * factor);
        }

        for (let s of body.sticks) {
            s.length *= factor;
        }
    }

    /**
     * Teleports a body
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
     * Set Body Pressure (Inflation)
     */
    //% group="Bodies"
    //% block="set body %body pressure %pressure"
    //% pressure.defl=0
    export function setBodyPressure(body: Body, pressure: number) {
        if (!body) return;
        body.pressure = pressure;
    }

    // ----------------------------------------------------------------------------------
    // GROUP: INTERACTION & CONTROLS
    // ----------------------------------------------------------------------------------

    /**
     * Controls a body with D-Pad
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
     * Apply a force
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
     * Explode at point
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
                    p.forceX += (dx / dist) * f;
                    p.forceY += (dy / dist) * f;
                }
            }
        }
    }

    /**
     * Enable dragging
     */
    //% group="Interaction"
    //% block="enable dragging with sprite %sprite"
    export function enableDragging(sprite: Sprite) {
        grabberSprite = sprite;
    }

    /**
     * Attaches a Sprite to a Physics Body with visual options
     */
    //% group="Interaction"
    //% block="attach sprite %sprite to body %body with offset x %offX y %offY"
    export function attachSprite(sprite: Sprite, body: Body, offX: number = 0, offY: number = 0) {
        if (!body) return;
        body.attachedSprite = sprite;
        body.visuals.offsetX = offX;
        body.visuals.offsetY = offY;
        body.visuals.rotate = false;
        body.visuals.squash = false;
    }

    /**
     * Configures sprite deformation visualization
     */
    //% group="Interaction"
    //% block="set body %body visuals rotate %rotate squash %squash"
    //% rotate.shadow=toggleOnOff squash.shadow=toggleOnOff
    export function setBodyVisuals(body: Body, rotate: boolean, squash: boolean) {
        if (!body) return;
        body.visuals.rotate = rotate;
        body.visuals.squash = squash;
    }

    // ----------------------------------------------------------------------------------
    // GROUP: JOINTS & ADVANCED CONSTRUCTION
    // ----------------------------------------------------------------------------------

    /**
     * Creates a custom hitbox by setting radius
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
     */
    //% group="Joints"
    //% block="set connection break force in %body from %i1 to %i2 force %force"
    //% force.defl=50
    export function setConnectionBreakForce(body: Body, i1: number, i2: number, force: number) {
        if (!body) return;
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
    // GROUP: VISUALS (SEGMENT HIDING)
    // ----------------------------------------------------------------------------------

    /**
     * Hides or shows a specific stick segment
     */
    //% group="Visuals"
    //% block="set body %body stick at index %index visible %visible"
    //% visible.shadow=toggleOnOff visible.defl=true
    export function setStickVisible(body: Body, index: number, visible: boolean) {
        if (body && body.sticks[index]) {
            body.sticks[index].visible = visible;
        }
    }

    /**
     * Hides or shows a specific node point
     */
    //% group="Visuals"
    //% block="set body %body node at index %index visible %visible"
    //% visible.shadow=toggleOnOff visible.defl=true
    export function setNodeVisible(body: Body, index: number, visible: boolean) {
        if (body && body.points[index]) {
            body.points[index].visible = visible;
        }
    }

    /**
     * Enables solid color filling for a body (Scanline Renderer)
     */
    //% group="Visuals"
    //% block="set body %body fill enabled %enabled color %color"
    //% enabled.shadow=toggleOnOff enabled.defl=false
    //% color.shadow="colorindexpicker"
    export function setBodyFill(body: Body, enabled: boolean, color: number) {
        if (!body) return;
        body.drawFill = enabled;
        body.fillColor = color;
    }

    // ----------------------------------------------------------------------------------
    // GROUP: EVENTS
    // ----------------------------------------------------------------------------------

    /**
     * Run code when two specific bodies collide
     */
    //% group="Events"
    //% block="on collision between %body1 and %body2"
    export function onBodyCollision(body1: Body, body2: Body, handler: () => void) {
        if (!body1 || !body2) return;
        collisionHandlers.push({ bodyA: body1, bodyB: body2, handler: handler });
    }

    /**
     * Run code when any part of a body breaks
     */
    //% group="Events"
    //% block="on body %body stick broken"
    export function onBodyStickBroken(body: Body, handler: () => void) {
        if (!body) return;
        body.onBreakHandler = handler;
    }

    // ----------------------------------------------------------------------------------
    // GROUP: ADVANCED TOOLS
    // ----------------------------------------------------------------------------------

    /**
     * Raycast against all bodies. Returns true if hit.
     */
    //% group="Advanced"
    //% block="raycast hit from x %x1 y %y1 to x %x2 y %y2"
    export function raycastHit(x1: number, y1: number, x2: number, y2: number): boolean {
        for (let b of bodies) {
            if (Math.min(x1, x2) > b.aabb.maxX || Math.max(x1, x2) < b.aabb.minX ||
                Math.min(y1, y2) > b.aabb.maxY || Math.max(y1, y2) < b.aabb.minY) {
                continue;
            }

            for (let p of b.points) {
                let l2 = Vec2.distSq({ x: x1, y: y1 }, { x: x2, y: y2 });
                if (l2 == 0) continue;

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
     * Draws a laser that stops at the first physics body hit
     */
    //% group="Advanced"
    //% block="draw laser from x %x1 y %y1 to x %x2 y %y2 color %color"
    //% color.shadow="colorindexpicker"
    export function drawLaser(x1: number, y1: number, x2: number, y2: number, color: number) {
        let hitX = x2;
        let hitY = y2;
        let minDistSq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

        for (let b of bodies) {
            // Check bounding box
            if (Math.min(x1, x2) > b.aabb.maxX || Math.max(x1, x2) < b.aabb.minX ||
                Math.min(y1, y2) > b.aabb.maxY || Math.max(y1, y2) < b.aabb.minY) {
                continue;
            }

            for (let p of b.points) {
                // Check intersection with circle
                // Math for Ray-Circle intersection
                let dx = x2 - x1;
                let dy = y2 - y1;
                let fx = x1 - p.x;
                let fy = y1 - p.y;

                let a = dx * dx + dy * dy;
                let b_coeff = 2 * (fx * dx + fy * dy);
                let c = (fx * fx + fy * fy) - p.radius * p.radius;

                let discriminant = b_coeff * b_coeff - 4 * a * c;
                if (discriminant >= 0) {
                    // Hit
                    discriminant = Math.sqrt(discriminant);
                    let t1 = (-b_coeff - discriminant) / (2 * a);

                    if (t1 >= 0 && t1 <= 1) {
                        let hitDistSq = t1 * t1 * a;
                        if (hitDistSq < minDistSq) {
                            minDistSq = hitDistSq;
                            hitX = x1 + t1 * dx;
                            hitY = y1 + t1 * dy;
                        }
                    }
                }
            }
        }
        screen.drawLine(x1, y1, hitX, hitY, color);
        screen.fillCircle(hitX, hitY, 2, color);
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