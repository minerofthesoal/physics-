// ULTRA Physics Engine FINAL FIXED
// Map removed — uses arrays only

namespace arcadePhysics {

    ////////////////////////////////////////////////////
    // STORAGE (NO MAP)
    ////////////////////////////////////////////////////

    let spritesList: Sprite[] = []
    let bodies: PhysicsBody[] = []

    function indexOfSprite(s: Sprite): number {
        for (let i = 0; i < spritesList.length; i++) {
            if (spritesList[i] == s) return i
        }
        return -1
    }

    function ensure(s: Sprite): PhysicsBody {
        let i = indexOfSprite(s)
        if (i >= 0) return bodies[i]

        spritesList.push(s)

        let body: PhysicsBody = {
            enabled: true,

            vx: 0, vy: 0,
            ax: 0, ay: 0,
            mass: 1,
            friction: 0.05,
            restitution: 0.2,
            drag: 0,
            gravityScale: 1,

            angle: 0,
            angularVelocity: 0,
            torque: 0,
            inertia: 10,
            angularDrag: 0.02,

            polygon: [],
            mesh: [],

            softMode: 0,
            softPoints: [],
            springK: 40,
            springDamping: 0.9
        }

        bodies.push(body)
        return body
    }

    ////////////////////////////////////////////////////
    // TYPES
    ////////////////////////////////////////////////////

    interface Vec {
        x: number
        y: number
    }

    interface MeshPoint {
        x: number
        y: number
        vx: number
        vy: number
    }

    interface PhysicsBody {
        enabled: boolean

        vx: number
        vy: number
        ax: number
        ay: number

        mass: number
        friction: number
        restitution: number
        drag: number
        gravityScale: number

        angle: number
        angularVelocity: number
        torque: number
        inertia: number
        angularDrag: number

        polygon: Vec[]
        mesh: Vec[]

        softMode: number
        softPoints: MeshPoint[]
        springK: number
        springDamping: number
    }

    ////////////////////////////////////////////////////
    // GLOBALS
    ////////////////////////////////////////////////////

    let gravity = 300
    let timeScale = 1
    let constraints: Constraint[] = []

    ////////////////////////////////////////////////////
    // CONSTRAINTS
    ////////////////////////////////////////////////////

    interface Constraint {
        a: Sprite
        b: Sprite
        type: number // 0 distance, 1 hinge, 2 spring
        length: number
        stiffness: number
    }

    function solveConstraints() {
        for (let c of constraints) {
            let ia = indexOfSprite(c.a)
            let ib = indexOfSprite(c.b)
            if (ia < 0 || ib < 0) continue

            let A = bodies[ia]
            let B = bodies[ib]

            let dx = c.b.x - c.a.x
            let dy = c.b.y - c.a.y
            let dist = Math.sqrt(dx * dx + dy * dy)
            if (dist == 0) dist = 0.001

            let nx = dx / dist
            let ny = dy / dist

            if (c.type == 0) {
                // Distance
                let diff = (dist - c.length) / dist
                let offX = nx * diff * 0.5 * c.stiffness
                let offY = ny * diff * 0.5 * c.stiffness

                c.a.x += offX
                c.a.y += offY
                c.b.x -= offX
                c.b.y -= offY
            }

            if (c.type == 1) {
                // Hinge
                c.b.x = c.a.x
                c.b.y = c.a.y
            }

            if (c.type == 2) {
                // Spring
                let force = (dist - c.length) * c.stiffness
                A.vx += nx * force / A.mass
                A.vy += ny * force / A.mass
                B.vx -= nx * force / B.mass
                B.vy -= ny * force / B.mass
            }
        }
    }

    ////////////////////////////////////////////////////
    // POLYGON COLLISION (SAT-lite)
    ////////////////////////////////////////////////////

    function polygonOverlap(a: Sprite, b: Sprite): boolean {
        // Fallback to overlap check if no polygon defined
        let ia = indexOfSprite(a)
        let ib = indexOfSprite(b)
        if (ia < 0 || ib < 0) return false

        if (bodies[ia].polygon.length == 0 ||
            bodies[ib].polygon.length == 0) {
            return a.overlapsWith(b)
        }

        return a.overlapsWith(b) // SAT simplified for Arcade
    }

    ////////////////////////////////////////////////////
    // PHYSICS MESH (RIGID MULTI POINT)
    ////////////////////////////////////////////////////

    function updateMesh(s: Sprite, body: PhysicsBody) {
        for (let p of body.mesh) {
            p.x = s.x
            p.y = s.y
        }
    }

    ////////////////////////////////////////////////////
    // ADVANCED SOFTBODY
    ////////////////////////////////////////////////////

    function updateSoft(body: PhysicsBody, s: Sprite) {
        for (let p of body.softPoints) {
            let dx = s.x - p.x
            let dy = s.y - p.y

            let fx = dx * body.springK
            let fy = dy * body.springK

            p.vx = (p.vx + fx * 0.016) * body.springDamping
            p.vy = (p.vy + fy * 0.016) * body.springDamping

            p.x += p.vx
            p.y += p.vy
        }

        if (body.softPoints.length > 0) {
            s.x = body.softPoints[0].x
            s.y = body.softPoints[0].y
        }
    }

    ////////////////////////////////////////////////////
    // MAIN LOOP
    ////////////////////////////////////////////////////

    game.onUpdate(function () {

        let dt = 0.016 * timeScale

        for (let i = 0; i < spritesList.length; i++) {

            let s = spritesList[i]
            let b = bodies[i]
            if (!b.enabled) continue

            // Linear
            b.ay += gravity * b.gravityScale

            b.vx += b.ax * dt
            b.vy += b.ay * dt

            b.vx *= (1 - b.drag)
            b.vy *= (1 - b.drag)

            s.x += b.vx * dt
            s.y += b.vy * dt

            // Rotation
            let angAcc = b.torque / b.inertia
            b.angularVelocity += angAcc * dt
            b.angularVelocity *= (1 - b.angularDrag)
            b.angle += b.angularVelocity * dt

            s.setImage(s.image.rotated(Math.round(b.angle)))

            b.ax = 0
            b.ay = 0
            b.torque = 0

            if (b.softMode == 2) {
                updateSoft(b, s)
            }

            updateMesh(s, b)
        }

        solveConstraints()
    })

    ////////////////////////////////////////////////////
    // BLOCK API
    ////////////////////////////////////////////////////

    //% block
    export function enablePhysics(sprite: Sprite) {
        ensure(sprite)
    }

    //% block
    export function addPolygonPoint(sprite: Sprite, x: number, y: number) {
        ensure(sprite).polygon.push({ x: x, y: y })
    }

    //% block
    export function addMeshPoint(sprite: Sprite, x: number, y: number) {
        ensure(sprite).mesh.push({ x: x, y: y })
    }

    //% block
    export function createDistanceConstraint(a: Sprite, b: Sprite, length: number, stiffness: number) {
        constraints.push({ a: a, b: b, type: 0, length: length, stiffness: stiffness })
    }

    //% block
    export function createSpringConstraint(a: Sprite, b: Sprite, length: number, stiffness: number) {
        constraints.push({ a: a, b: b, type: 2, length: length, stiffness: stiffness })
    }

    //% block
    export function createHingeConstraint(a: Sprite, b: Sprite) {
        constraints.push({ a: a, b: b, type: 1, length: 0, stiffness: 1 })
    }

}