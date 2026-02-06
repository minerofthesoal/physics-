//% color="#E63022" weight=100 icon="\uf1e3" block="Jelly Physics"
namespace jelly {

    // --- Core Classes ---

    export class Point {
        x: number;
        y: number;
        oldx: number;
        oldy: number;
        vx: number;
        vy: number;
        pinned: boolean;
        radius: number;

        constructor(x: number, y: number) {
            this.x = x;
            this.y = y;
            this.oldx = x;
            this.oldy = y;
            this.vx = 0;
            this.vy = 0;
            this.pinned = false;
            this.radius = 5; // collision radius
        }

        update(gravity: number, friction: number) {
            if (this.pinned) return;

            // Calculate velocity based on position history (Verlet Integration)
            this.vx = (this.x - this.oldx) * friction;
            this.vy = (this.y - this.oldy) * friction;

            this.oldx = this.x;
            this.oldy = this.y;

            this.x += this.vx;
            this.y += this.vy + gravity;
        }

        constrain(width: number, height: number, bounce: number) {
            if (this.pinned) return;

            // Wall collisions
            if (this.x > width - this.radius) {
                this.x = width - this.radius;
                this.oldx = this.x + (this.vx * bounce);
            } else if (this.x < this.radius) {
                this.x = this.radius;
                this.oldx = this.x + (this.vx * bounce);
            }

            // Floor/Ceiling collisions
            if (this.y > height - this.radius) {
                this.y = height - this.radius;
                this.oldy = this.y + (this.vy * bounce);
            } else if (this.y < this.radius) {
                this.y = this.radius;
                this.oldy = this.y + (this.vy * bounce);
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

        constructor(p1: Point, p2: Point, length: number, stiffness: number) {
            this.p1 = p1;
            this.p2 = p2;
            this.length = length;
            this.stiffness = stiffness;
            this.visible = true;
            this.color = 1;
        }

        update() {
            const dx = this.p2.x - this.p1.x;
            const dy = this.p2.y - this.p1.y;
            const dist = Math.sqrt(dx * dx + dy * dy);

            if (dist === 0) return;

            const diff = this.length - dist;
            const percent = (diff / dist) / 2;

            const offsetX = dx * percent * this.stiffness;
            const offsetY = dy * percent * this.stiffness;

            if (!this.p1.pinned) {
                this.p1.x -= offsetX;
                this.p1.y -= offsetY;
            }
            if (!this.p2.pinned) {
                this.p2.x += offsetX;
                this.p2.y += offsetY;
            }
        }

        draw() {
            if (this.visible) {
                screen.drawLine(this.p1.x, this.p1.y, this.p2.x, this.p2.y, this.color);
            }
        }
    }

    export class Body {
        points: Point[];
        sticks: Stick[];
        attachedSprite: Sprite;
        rotateSprite: boolean;
        color: number;
        drawNodes: boolean;
        drawSticks: boolean;

        // Control properties
        isControlled: boolean;
        controlSpeed: number;

        constructor() {
            this.points = [];
            this.sticks = [];
            this.rotateSprite = false;
            this.color = 1;
            this.drawNodes = true;
            this.drawSticks = true;
            this.isControlled = false;
            this.controlSpeed = 2;
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
                let dx = p2.x - p1.x;
                let dy = p2.y - p1.y;
                length = Math.sqrt(dx * dx + dy * dy);
            }

            let s = new Stick(p1, p2, length, stiffness);
            s.color = this.color;
            this.sticks.push(s);
            return s;
        }

        update(gravity: number, friction: number, width: number, height: number, iterations: number, bounce: number) {
            // Player Control Logic
            if (this.isControlled) {
                let dx = controller.dx(this.controlSpeed);
                let dy = controller.dy(this.controlSpeed);

                if (dx != 0 || dy != 0) {
                    // Apply force to all points slightly
                    for (let p of this.points) {
                        p.x += dx * 0.1;
                        p.y += dy * 0.1;
                    }
                }
            }

            for (let p of this.points) {
                p.update(gravity, friction);
                p.constrain(width, height, bounce);
            }

            for (let i = 0; i < iterations; i++) {
                for (let s of this.sticks) {
                    s.update();
                }
            }

            // Sprite Sync
            if (this.attachedSprite && this.points.length > 0) {
                let cx = 0, cy = 0;
                for (let p of this.points) { cx += p.x; cy += p.y; }
                cx /= this.points.length;
                cy /= this.points.length;
                this.attachedSprite.setPosition(cx, cy);
            }
        }

        draw() {
            if (this.drawSticks) {
                for (let s of this.sticks) {
                    s.color = this.color;
                    s.draw();
                }
            }
            if (this.drawNodes) {
                for (let p of this.points) {
                    screen.setPixel(p.x, p.y, this.color);
                    screen.setPixel(p.x + 1, p.y, this.color);
                    screen.setPixel(p.x, p.y + 1, this.color);
                    screen.setPixel(p.x + 1, p.y + 1, this.color);
                }
            }
        }
    }

    // --- Global State ---

    let bodies: Body[] = [];
    let gravity = 0.5;
    let friction = 0.99;
    let bounce = 0.9;
    let iterations = 5;
    let debugDraw = true;

    // Interaction
    let grabberSprite: Sprite = null;
    let grabbedPoint: Point = null;
    let grabRange = 20;

    // --- Collision Logic ---

    function resolveCollisions() {
        // Simple point-to-point repulsion between different bodies
        for (let i = 0; i < bodies.length; i++) {
            for (let j = i + 1; j < bodies.length; j++) {
                let b1 = bodies[i];
                let b2 = bodies[j];

                for (let p1 of b1.points) {
                    for (let p2 of b2.points) {
                        let dx = p1.x - p2.x;
                        let dy = p1.y - p2.y;
                        let distSq = dx * dx + dy * dy;
                        let minDist = p1.radius + p2.radius;

                        if (distSq < minDist * minDist && distSq > 0) {
                            let dist = Math.sqrt(distSq);
                            let overlap = minDist - dist;
                            let force = overlap / 2; // Split the push

                            let fx = (dx / dist) * force;
                            let fy = (dy / dist) * force;

                            if (!p1.pinned) {
                                p1.x += fx;
                                p1.y += fy;
                            }
                            if (!p2.pinned) {
                                p2.x -= fx;
                                p2.y -= fy;
                            }
                        }
                    }
                }
            }
        }
    }

    // --- Lifecycle ---

    game.onUpdate(function () {
        for (let b of bodies) {
            b.update(gravity, friction, screen.width, screen.height, iterations, bounce);
        }
        resolveCollisions();

        // Handle Grabber
        if (grabberSprite) {
            if (controller.A.isPressed()) {
                if (!grabbedPoint) {
                    // Try to find a point close to sprite
                    let closestDist = grabRange;
                    for (let b of bodies) {
                        for (let p of b.points) {
                            let dx = grabberSprite.x - p.x;
                            let dy = grabberSprite.y - p.y;
                            let d = Math.sqrt(dx * dx + dy * dy);
                            if (d < closestDist) {
                                closestDist = d;
                                grabbedPoint = p;
                            }
                        }
                    }
                }

                // If holding a point, move it
                if (grabbedPoint) {
                    grabbedPoint.x = grabberSprite.x;
                    grabbedPoint.y = grabberSprite.y;
                    grabbedPoint.vx = 0;
                    grabbedPoint.vy = 0;
                }
            } else {
                grabbedPoint = null;
            }
        }
    });

    game.onPaint(function () {
        if (debugDraw) {
            for (let b of bodies) {
                b.draw();
            }
        }
    });

    // --- Block Definitions ---

    // ---------------------------------------------------
    // CREATION
    // ---------------------------------------------------

    /**
     * Creates a new Soft Body Box
     */
    //% group="Creation"
    //% block="create soft box at x %x y %y size %size stiffness %stiffness"
    //% stiffness.defl=0.5
    //% size.defl=40
    export function createSoftBox(x: number, y: number, size: number, stiffness: number): Body {
        let b = new Body();
        b.addPoint(x, y);
        b.addPoint(x + size, y);
        b.addPoint(x + size, y + size);
        b.addPoint(x, y + size);

        // -1 indicates auto-length
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
     * Creates a Rope
     */
    //% group="Creation"
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
     * Creates a Ragdoll
     */
    //% group="Creation"
    //% block="create ragdoll at x %x y %y scale %scale"
    //% scale.defl=1.0
    export function createRagdoll(x: number, y: number, scale: number): Body {
        let b = new Body();
        let s = scale * 20;

        // 0: Head
        b.addPoint(x, y - s * 1.5);
        // 1: Chest
        b.addPoint(x, y);
        // 2: Hips
        b.addPoint(x, y + s);
        // 3: Left Elbow
        b.addPoint(x - s, y);
        // 4: Left Hand
        b.addPoint(x - s * 1.8, y);
        // 5: Right Elbow
        b.addPoint(x + s, y);
        // 6: Right Hand
        b.addPoint(x + s * 1.8, y);
        // 7: Left Knee
        b.addPoint(x - s * 0.5, y + s * 1.8);
        // 8: Left Foot
        b.addPoint(x - s * 0.5, y + s * 2.6);
        // 9: Right Knee
        b.addPoint(x + s * 0.5, y + s * 1.8);
        // 10: Right Foot
        b.addPoint(x + s * 0.5, y + s * 2.6);

        let stiff = 1.0;

        // Neck / Torso
        b.addStick(0, 1, stiff, -1);
        b.addStick(1, 2, stiff, -1);

        // Arms
        b.addStick(1, 3, stiff, -1);
        b.addStick(3, 4, stiff, -1);
        b.addStick(1, 5, stiff, -1);
        b.addStick(5, 6, stiff, -1);

        // Legs
        b.addStick(2, 7, stiff, -1);
        b.addStick(7, 8, stiff, -1);
        b.addStick(2, 9, stiff, -1);
        b.addStick(9, 10, stiff, -1);

        // Bracing (Invisible constraints to keep shape)
        b.addStick(0, 2, 0.5, -1).visible = false; // Head to hips
        b.addStick(3, 5, 0.5, -1).visible = false; // Shoulder to shoulder

        bodies.push(b);
        return b;
    }

    /**
     * Create an empty physics body
     */
    //% group="Creation"
    //% block="create empty body"
    export function createEmptyBody(): Body {
        let b = new Body();
        bodies.push(b);
        return b;
    }

    // ---------------------------------------------------
    // CONTROLS & MOVEMENT
    // ---------------------------------------------------

    /**
     * Controls a body with the player controller (D-Pad)
     */
    //% group="Controls"
    //% block="move body %body with controller speed %speed"
    //% speed.defl=100
    export function moveBodyWithController(body: Body, speed: number) {
        if (!body) return;
        body.isControlled = true;
        body.controlSpeed = speed;
    }

    /**
     * Apply a force (push) to a body
     */
    //% group="Controls"
    //% block="apply force to %body x %fx y %fy"
    export function applyForce(body: Body, fx: number, fy: number) {
        if (!body) return;
        for (let p of body.points) {
            // We modify the 'x' directly which acts as a velocity change in Verlet
            // But modifying 'oldx' is safer for impulse
            p.x += fx;
            p.y += fy;
        }
    }

    /**
     * Teleports a body to a new location
     */
    //% group="Controls"
    //% block="teleport body %body to x %x y %y"
    export function teleportBody(body: Body, x: number, y: number) {
        if (!body || body.points.length === 0) return;

        // Find center
        let cx = 0, cy = 0;
        for (let p of body.points) { cx += p.x; cy += p.y; }
        cx /= body.points.length;
        cy /= body.points.length;

        let dx = x - cx;
        let dy = y - cy;

        for (let p of body.points) {
            p.x += dx;
            p.y += dy;
            p.oldx += dx; // Reset velocity
            p.oldy += dy;
        }
    }

    /**
     * Enable dragging with a sprite (A button)
     */
    //% group="Controls"
    //% block="enable dragging with sprite %sprite"
    export function enableDragging(sprite: Sprite) {
        grabberSprite = sprite;
    }

    // ---------------------------------------------------
    // CUSTOMIZATION
    // ---------------------------------------------------

    /**
     * Attaches a Sprite to a Physics Body
     */
    //% group="Customization"
    //% block="attach sprite %sprite to body %body"
    export function attachSprite(sprite: Sprite, body: Body) {
        if (!body) return;
        body.attachedSprite = sprite;
    }

    /**
     * Sets physics parameters
     */
    //% group="Settings"
    //% block="set physics gravity %g friction %f bounce %b"
    //% g.defl=0.5 f.defl=0.99 b.defl=0.9
    export function setPhysics(g: number, f: number, b: number) {
        gravity = g;
        friction = f;
        bounce = b;
    }

    /**
     * Toggle Debug Drawing
     */
    //% group="Settings"
    //% block="set debug draw %on"
    //% on.shadow=toggleOnOff
    //% on.defl=true
    export function setDebugDraw(on: boolean) {
        debugDraw = on;
    }

    /**
     * Set Body Color
     */
    //% group="Customization"
    //% block="set body %body color %color"
    //% color.shadow="colorindexpicker"
    export function setBodyColor(body: Body, color: number) {
        if (body) body.color = color;
    }

    /**
     * Add a node to a body
     */
    //% group="Advanced"
    //% block="add node to %body at x %x y %y"
    export function addNode(body: Body, x: number, y: number) {
        if (body) body.addPoint(x, y);
    }

    /**
     * Connect two nodes
     */
    //% group="Advanced"
    //% block="connect nodes in %body from index %i1 to %i2 stiffness %stiffness"
    //% stiffness.defl=1.0
    export function connectNodes(body: Body, i1: number, i2: number, stiffness: number) {
        if (body) body.addStick(i1, i2, stiffness, -1);
    }
}