/**
 * Moving faces and letting the solid follow (`tweakFacesWithHistory`, facade/src/bezier_tweak.cpp).
 * Every expected number is worked out by hand from the geometry, not read off a run.
 */
import { afterAll, afterEach, beforeAll, describe, expect, it } from "vitest";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const BOUND = 1_000_000;

// eslint-disable-next-line @typescript-eslint/no-explicit-any
let kernel: any;

beforeAll(async () => {
    const jsPath = resolve(__dirname, "../dist/occt-wasm.js");
    const wasmPath = resolve(__dirname, "../dist/occt-wasm.wasm");
    const createModule = (await import(jsPath)).default;
    const Module = await createModule({
        locateFile: (p: string) => (p.endsWith(".wasm") ? wasmPath : p),
    });
    const mod = await import(resolve(__dirname, "../ts/src/index.ts"));
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    kernel = new (mod.OcctKernel as any)(Module);
}, 30_000);

afterEach(() => kernel.releaseAll());
afterAll(() => kernel[Symbol.dispose]());

type V = { x: number; y: number; z: number };
const IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0];
const shift = (x: number, y: number, z: number) => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z];
/** A turn about a line through `at`, parallel to x. */
const turnAboutX = (angle: number, at: V) => {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    // R·p + t with t = at − R·at.
    const ty = at.y - (c * at.y - s * at.z);
    const tz = at.z - (s * at.y + c * at.z);
    return [1, 0, 0, 0, 0, c, -s, ty, 0, s, c, tz];
};

/** A turn about a line through `at`, parallel to y; positive carries +z towards +x. */
const turnAboutY = (angle: number, at: V) => {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    const tx = at.x - (c * at.x + s * at.z);
    const tz = at.z - (-s * at.x + c * at.z);
    return [c, 0, s, tx, 0, 1, 0, 0, -s, 0, c, tz];
};

/** A 10 mm cube with a 2 mm radius hole straight down through its middle. */
function cubeWithHole(): number {
    const box = kernel.makeBox(10, 10, 10);
    const pin = kernel.translate(kernel.makeCylinder(2, 10), 5, 5, 0);
    return kernel.cut(box, pin);
}

/** The hole's wall: the one round face. The top's centre of mass lies on the hole's axis too. */
function holeOf(part: number): number {
    const round = kernel.getSubShapes(part, "face").filter((f: number) => kernel.surfaceType(f) === "cylinder");
    expect(round).toHaveLength(1);
    return round[0];
}

function faceAt(shape: number, where: (c: V) => boolean): number {
    for (const face of kernel.getSubShapes(shape, "face")) {
        if (where(kernel.getSurfaceCenterOfMass(face))) return face;
    }
    throw new Error("no such face");
}

function hashes(shape: number): number[] {
    return kernel.getSubShapes(shape, "face").map((f: number) => kernel.hashCode(f, BOUND));
}

function tweak(shape: number, faces: number[], motions: number[][], offsets: number[]) {
    return kernel.tweakFacesWithHistory(shape, faces, motions.flat(), offsets, hashes(shape), BOUND);
}

describe("tweak: faces move and the solid follows", () => {
    it("pulls the top of a box up", () => {
        const box = kernel.makeBox(10, 10, 10);
        const top = faceAt(box, (c) => Math.abs(c.z - 10) < 1e-6);
        const evo = tweak(box, [top], [shift(0, 0, 5)], [0]);
        expect(kernel.isValid(evo.result)).toBe(true);
        expect(kernel.getVolume(evo.result)).toBeCloseTo(1500, 6);
        expect(evo.modified).toContain(kernel.hashCode(top, BOUND));
    });

    it("tilts the top of a box about its middle, which keeps the volume", () => {
        const box = kernel.makeBox(10, 10, 10);
        const top = faceAt(box, (c) => Math.abs(c.z - 10) < 1e-6);
        const evo = tweak(box, [top], [turnAboutX(0.2, { x: 5, y: 5, z: 10 })], [0]);
        expect(kernel.isValid(evo.result)).toBe(true);
        expect(kernel.getVolume(evo.result)).toBeCloseTo(1000, 6);
    });

    it("lets a chamfer beside the top grow with it, along its own slope", () => {
        const box = kernel.makeBox(10, 10, 10);
        const edge = kernel
            .getSubShapes(box, "edge")
            .find((e: number) => {
                const c = kernel.getLinearCenterOfMass(e);
                return Math.abs(c.y - 10) < 1e-6 && Math.abs(c.z - 10) < 1e-6;
            });
        const chamfered = kernel.chamfer(box, [edge], 2);
        expect(kernel.getVolume(chamfered)).toBeCloseTo(980, 6);
        const top = faceAt(chamfered, (c) => Math.abs(c.z - 10) < 1e-6);
        const evo = tweak(chamfered, [top], [shift(0, 0, 3)], [0]);
        expect(kernel.isValid(evo.result)).toBe(true);
        // The 45° chamfer runs on up: the top shrinks from 8 to 5 deep over the 3 mm, so the
        // added slab is 10 × 3 × (8 + 5) / 2 = 195. A step instead would add 240.
        expect(kernel.getVolume(evo.result)).toBeCloseTo(1175, 6);
    });

    it("offsets a cylinder's side, which changes its radius", () => {
        const cylinder = kernel.makeCylinder(5, 10);
        const side = faceAt(cylinder, (c) => Math.abs(c.z - 5) < 1e-6);
        const evo = tweak(cylinder, [side], [IDENTITY], [2]);
        expect(kernel.isValid(evo.result)).toBe(true);
        expect(kernel.getVolume(evo.result)).toBeCloseTo(Math.PI * 49 * 10, 4);
    });

    it("pulls up the top of a cylinder, its round edge found again", () => {
        const cylinder = kernel.makeCylinder(5, 10);
        const top = faceAt(cylinder, (c) => Math.abs(c.z - 10) < 1e-6);
        const evo = tweak(cylinder, [top], [shift(0, 0, 4)], [0]);
        expect(kernel.isValid(evo.result)).toBe(true);
        expect(kernel.getVolume(evo.result)).toBeCloseTo(Math.PI * 25 * 14, 4);
    });

    it("refuses a move with no solid at the end of it", () => {
        const box = kernel.makeBox(10, 10, 10);
        const top = faceAt(box, (c) => Math.abs(c.z - 10) < 1e-6);
        // Turned a quarter, the top would stand parallel to two of its neighbours.
        expect(() => tweak(box, [top], [turnAboutX(Math.PI / 2, { x: 5, y: 5, z: 10 })], [0])).toThrow();
    });

    it("pulls up a box whose upright edges are rounded, the rounds growing with it", () => {
        const box = kernel.makeBox(10, 10, 10);
        const upright = kernel.getSubShapes(box, "edge").filter((e: number) => {
            const c = kernel.getLinearCenterOfMass(e);
            return Math.abs(c.z - 5) < 1e-6;
        });
        const rounded = kernel.fillet(box, upright, 2);
        const top = faceAt(rounded, (c) => Math.abs(c.z - 10) < 1e-6);
        const evo = tweak(rounded, [top], [shift(0, 0, 5)], [0]);
        expect(kernel.isValid(evo.result)).toBe(true);
        expect(kernel.getVolume(evo.result)).toBeCloseTo((100 - 4 * (4 - Math.PI)) * 15, 4);
    });

    it("pulls up the top of a part with a hole, and the hole runs on through", () => {
        const part = cubeWithHole();
        const top = faceAt(part, (c) => Math.abs(c.z - 10) < 1e-6);
        const evo = tweak(part, [top], [shift(0, 0, 5)], [0]);
        expect(kernel.isValid(evo.result)).toBe(true);
        expect(kernel.getVolume(evo.result)).toBeCloseTo((100 - 4 * Math.PI) * 15, 4);
    });

    it("moves a hole sideways inside its part", () => {
        const part = cubeWithHole();
        const hole = holeOf(part);
        const evo = tweak(part, [hole], [shift(2, 0, 0)], [0]);
        expect(kernel.isValid(evo.result)).toBe(true);
        expect(kernel.getVolume(evo.result)).toBeCloseTo((100 - 4 * Math.PI) * 10, 4);
        const moved = kernel.getSubShapes(evo.result, "face").filter((f: number) => kernel.surfaceType(f) === "cylinder");
        expect(moved).toHaveLength(1);
        expect(kernel.getSurfaceCenterOfMass(moved[0]).x).toBeCloseTo(7, 6);
    });

    it("offsets a hole outward from the material, which makes it smaller", () => {
        const part = cubeWithHole();
        const hole = holeOf(part);
        // Outward from the solid is towards the hole's axis: a hole of radius 1 is left.
        const evo = tweak(part, [hole], [IDENTITY], [1]);
        expect(kernel.isValid(evo.result)).toBe(true);
        expect(kernel.getVolume(evo.result)).toBeCloseTo((100 - Math.PI) * 10, 4);
    });

    it("moves two faces at once", () => {
        const box = kernel.makeBox(10, 10, 10);
        const top = faceAt(box, (c) => Math.abs(c.z - 10) < 1e-6);
        const side = faceAt(box, (c) => Math.abs(c.x - 10) < 1e-6);
        const evo = tweak(box, [top, side], [shift(0, 0, 2), shift(3, 0, 0)], [0, 0]);
        expect(kernel.isValid(evo.result)).toBe(true);
        expect(kernel.getVolume(evo.result)).toBeCloseTo(13 * 10 * 12, 6);
    });

    it("leans a side face out about its bottom edge, a draft", () => {
        const box = kernel.makeBox(10, 10, 10);
        const side = faceAt(box, (c) => Math.abs(c.x - 10) < 1e-6);
        const evo = tweak(box, [side], [turnAboutY(0.1, { x: 10, y: 0, z: 0 })], [0]);
        expect(kernel.isValid(evo.result)).toBe(true);
        // A wedge 10 high, 10·tan(0.1) wide at the top, 10 deep.
        expect(kernel.getVolume(evo.result)).toBeCloseTo(1000 + 500 * Math.tan(0.1), 6);
    });

    it("refuses to push the top down past a chamfer, which would turn the chamfer inside out", () => {
        const box = kernel.makeBox(10, 10, 10);
        const edge = kernel.getSubShapes(box, "edge").find((e: number) => {
            const c = kernel.getLinearCenterOfMass(e);
            return Math.abs(c.y - 10) < 1e-6 && Math.abs(c.z - 10) < 1e-6;
        });
        const chamfered = kernel.chamfer(box, [edge], 2);
        const top = faceAt(chamfered, (c) => Math.abs(c.z - 10) < 1e-6);
        expect(() => tweak(chamfered, [top], [shift(0, 0, -3)], [0])).toThrow();
    });

    it("slides the top of a part with a hole along its own plane, which changes nothing", () => {
        const part = cubeWithHole();
        const top = faceAt(part, (c) => Math.abs(c.z - 10) < 1e-6);
        const evo = tweak(part, [top], [shift(2, 0, 0)], [0]);
        expect(kernel.isValid(evo.result)).toBe(true);
        expect(kernel.getVolume(evo.result)).toBeCloseTo((100 - 4 * Math.PI) * 10, 6);
    });

    it("pulls up a box whose top edges are rounded, the roundings carried with the top", () => {
        const box = kernel.makeBox(10, 10, 10);
        const rim = kernel.getSubShapes(box, "edge").filter((e: number) => {
            const c = kernel.getLinearCenterOfMass(e);
            return Math.abs(c.z - 10) < 1e-6;
        });
        const rounded = kernel.fillet(box, rim, 2);
        const top = faceAt(rounded, (c) => Math.abs(c.z - 10) < 1e-6);
        // The roundings tangent to the top move with it: the caller says so, as the app does.
        const roundings = kernel
            .getSubShapes(rounded, "face")
            .filter((f: number) => kernel.surfaceType(f) !== "plane");
        const faces = [top, ...roundings];
        const before = kernel.getVolume(rounded);
        const evo = tweak(rounded, faces, faces.map(() => shift(0, 0, 3)), faces.map(() => 0));
        expect(kernel.isValid(evo.result)).toBe(true);
        // The walls grow by 3 straight up all round: the footprint, unchanged, times 3.
        expect(kernel.getVolume(evo.result)).toBeCloseTo(before + 100 * 3, 4);
    });

    it("removes a chamfer, the faces beside it growing over the gap, with history", () => {
        const box = kernel.makeBox(10, 10, 10);
        const edge = kernel.getSubShapes(box, "edge").find((e: number) => {
            const c = kernel.getLinearCenterOfMass(e);
            return Math.abs(c.y - 10) < 1e-6 && Math.abs(c.z - 10) < 1e-6;
        });
        const chamfered = kernel.chamfer(box, [edge], 2);
        const slope = faceAt(chamfered, (c) => Math.abs(c.y + c.z - 18) < 1e-6 && c.y > 8.5);
        const evo = kernel.defeatureWithHistory(chamfered, [slope], hashes(chamfered), BOUND);
        expect(kernel.isValid(evo.result)).toBe(true);
        expect(kernel.getVolume(evo.result)).toBeCloseTo(1000, 6);
        expect(evo.deleted).toContain(kernel.hashCode(slope, BOUND));
    });
});
